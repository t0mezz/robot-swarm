#pragma once
// ─── Pairwise robot collision avoidance ───────────────────────────────────────
//
// Strategy: DODGE, don't brake.
//
// The original version scaled both robots' speed down as they closed. That
// prevented contact but did not resolve anything — two robots approaching each
// other both ramped to a crawl and stalled nose to nose, each waiting for a gap
// the other was no longer moving to create. Slowing is not avoidance; it is a
// deferred collision.
//
// So the speed ramp is gone. Instead, exactly ONE robot of a conflicting pair is
// nominated as the dodger and told to make a hard, committed detour, while the
// other is explicitly designated `priority` and left at full speed. Keeping the
// priority robot fast is the point: it clears the shared area, which is what
// actually ends the conflict.
//
// Who dodges — a robot qualifies only if it is BOTH under way and closing on
// the other (heading dot line-of-centres > faceDot):
//   one qualifies   : that one dodges, whatever the IDs. Covers a mover meeting
//                     a parked robot, and an overtake, where the robot behind is
//                     the only one with a problem.
//   both qualify    : the higher ID dodges, the lower ID holds its line. This is
//                     the head-on case the ID rule exists for.
//   neither         : no conflict, even if the discs overlap.
// Deciding by ID *before* testing who is closing gets an overtake exactly wrong:
// it nominates the robot in front, which is driving away and correctly declines
// to dodge, so nobody does and the robot behind — never slowed — rear-ends it.
//
// The dodge is LATCHED per pair. Once triggered it holds — same side, same
// commitment — until the pair separates past safeMm or the dodge times out. This
// is not an optimization: the trigger test looks at the dodger's heading, and
// the dodge itself changes that heading, so a stateless version cancels itself
// the moment it starts working and the robot chatters in and out of the detour.
//
// O(N²/2) pair scan — fine for the 32-robot ceiling; if the swarm ever outgrows
// that, this is the loop to put a spatial hash in front of.
//
// Templated on the pose type so it works directly with ArucoTracker's RobotPose
// without a per-frame copy; any struct with float .x/.y/.yaw members fits.

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "goto_controller.h"   // normAngle, clampf

namespace swarmctl {

// Chassis diameter of a Pololu 3pi+ 2040, millimetres. Every distance below is
// really a multiple of this — two robots "touch" at a centre-to-centre distance
// of one diameter, so a threshold only means something relative to it.
//
// NOT measured on the floor: taken from the published 3pi+ chassis size. Confirm
// with calipers (and against the ArUco scale, since minDist is a marker-centre
// distance, not a hull distance) before treating the ratios as tuned.
inline constexpr float ROBOT_DIAMETER_MM = 98.0f;

// ── Parameters ────────────────────────────────────────────────────────────────

struct AvoidParams {
    // Trigger distance. Each robot owns a danger disc of radius dangerMm/2, and
    // a pair conflicts when those discs overlap — which is exactly
    // centre-to-centre < dangerMm. Kept as the pair distance rather than a
    // per-robot radius so it stays directly comparable to minDist.
    //
    // This is NOT a free tuning knob. A dodge needs enough runway to physically
    // turn, so the trigger distance is bounded from below by the robot's own
    // speed and turn rate:
    //
    //     dangerMm  >  closingSpeed x timeToTurn90
    //               =  (2 x MAX_SPEED x mmPerUnit) x (90 / turnRateDegPerSec)
    //
    // Set it below that and the dodge cannot complete no matter how aggressive
    // the blend — the pair simply closes faster than either can turn out of the
    // way. At MAX_SPEED 60 that works out to ~340mm of pure turning distance,
    // which is why the value here is much larger than it looks like it needs to
    // be. If the arena is too small for that, the fix is to lower MAX_SPEED, not
    // this constant: they are two ends of the same inequality.
    float dangerMm = 500.0f;

    // Release distance. Deliberately larger than dangerMm: this hysteresis is
    // what stops a dodge from cancelling the instant it starts to work.
    float safeMm   = 1000.0f;

    // Fraction of the goal vector handed to the detour while dodging. High on
    // purpose — a timid arc reads as "drifting toward the obstacle slowly".
    float blend    = 0.95f;

    // Dot-product threshold for "this robot is closing on the other". cos(~75°).
    // Only tested at trigger time; a latched dodge is released by distance.
    float faceDot  = 0.25f;

    // Speed multiplier for the DODGER only. Near full speed by design: the
    // detour needs momentum to get around, and the whole point of this rework is
    // that slowing down is not how the conflict gets resolved. The priority
    // robot is never scaled at all.
    float dodgeSpeedFrac = 0.70f;

    // Last-resort stop for a failed dodge, applied to the CHARGING robot only —
    // see the emergency block in update() for why it is not the dodger and not
    // both. ~1.6 robot diameters: at 1.2 D the sim still grazed contact in a
    // perpendicular crossing, since the stop has to happen early enough to bleed
    // off the approach.
    float emergencyMm = 160.0f;

    // Give up on a latched dodge after this long. Without it a robot whose goal
    // sits behind a permanently parked obstacle orbits it forever. On timeout
    // the pair releases, normal control resumes, and emergencyMm backstops.
    float maxDodgeS = 4.0f;
};

// ── Per-robot avoidance advice ────────────────────────────────────────────────

struct AvoidState {
    float minDist   = 1e6f;   // distance to nearest robot (HUD / diagnostics)
    bool  dodging   = false;  // this robot must execute the detour below
    bool  priority  = false;  // protected: hold the line, do NOT slow down
    float arcDx     = 0.f;    // unit detour direction, world frame
    float arcDy     = 0.f;
    bool  emergency = false;  // last-resort stop; set on the CHARGING robot
};

// ── The engine ────────────────────────────────────────────────────────────────
//
// Stateful, because the dodge latch is. One instance per controller; call
// update() once per control iteration.
//
// isMoving(id) tells the scan whether a robot is under way — it decides who
// yields. Callers typically define it as "has a goal, and is further from it
// than the arrival radius".

class AvoidanceEngine {
public:
    explicit AvoidanceEngine(const AvoidParams& p = {}) : p_(p) {}

    const AvoidParams& params() const { return p_; }
    void setParams(const AvoidParams& p) { p_ = p; }
    void reset() { latches_.clear(); }

    template <typename PoseMap>
    std::unordered_map<int, AvoidState> update(
        const PoseMap& poses,
        const std::function<bool(int)>& isMoving,
        float dt)
    {
        std::unordered_map<int, AvoidState> out;
        for (auto& [id, _] : poses) out[id] = {};

        std::vector<int> ids;
        ids.reserve(poses.size());
        for (auto& [id, _] : poses) ids.push_back(id);
        std::sort(ids.begin(), ids.end());

        for (auto& [key, l] : latches_) l.touched = false;

        // Nearest-threat detour per robot, kept as the fallback for when the
        // summed arcs of several threats cancel out (see the normalize pass).
        struct Fallback { float dist = 1e6f; float dx = 0.f, dy = 0.f; };
        std::unordered_map<int, Fallback> fallback;

        for (size_t i = 0; i < ids.size(); i++) {
            for (size_t j = i + 1; j < ids.size(); j++) {
                const int lo = ids[i], hi = ids[j];   // lo < hi ⟹ lo has priority
                const auto& pLo = poses.at(lo);
                const auto& pHi = poses.at(hi);

                const float dx   = pHi.x - pLo.x;
                const float dy   = pHi.y - pLo.y;
                const float dist = std::sqrt(dx * dx + dy * dy);

                const Key key{lo, hi};

                // Out of range: release any latch and ignore the pair entirely.
                if (dist >= p_.safeMm) { latches_.erase(key); continue; }

                out[lo].minDist = std::min(out[lo].minDist, dist);
                out[hi].minDist = std::min(out[hi].minDist, dist);

                if (dist < 1e-4f) continue;   // coincident poses: no usable normal
                const float nx = dx / dist, ny = dy / dist;   // unit lo→hi

                auto it = latches_.find(key);

                if (it == latches_.end()) {
                    // ── Trigger ───────────────────────────────────────────────
                    // Discs must actually overlap, and the nominated dodger must
                    // be heading into the other robot.
                    if (dist >= p_.dangerMm) continue;

                    // Who is actually on a collision course? A robot only
                    // qualifies as dodger if it is BOTH under way and closing on
                    // the other — nominating first and testing afterwards picks
                    // the wrong robot in an overtake (the one in front is higher
                    // ID but driving away, so it correctly declines to dodge
                    // while the one behind, which is never slowed, rams it).
                    const float loHx = std::cos(pLo.yaw * (float)M_PI / 180.f);
                    const float loHy = std::sin(pLo.yaw * (float)M_PI / 180.f);
                    const float hiHx = std::cos(pHi.yaw * (float)M_PI / 180.f);
                    const float hiHy = std::sin(pHi.yaw * (float)M_PI / 180.f);

                    const bool loClosing = isMoving(lo) && (loHx *  nx + loHy *  ny) > p_.faceDot;
                    const bool hiClosing = isMoving(hi) && (hiHx * -nx + hiHy * -ny) > p_.faceDot;

                    if (!loClosing && !hiClosing) continue;   // no conflict

                    // ID priority is the tiebreak among robots that are BOTH
                    // closing (the head-on case the rule was written for), not
                    // a rule applied before knowing who the aggressor is.
                    int dodger;
                    if (loClosing && hiClosing) dodger = hi;
                    else                        dodger = loClosing ? lo : hi;

                    const float hx = (dodger == lo) ? loHx : hiHx;
                    const float hy = (dodger == lo) ? loHy : hiHy;

                    // Commit to a SIDE and hold that — not to a world-frame
                    // vector. Which side is chosen is fixed here (whichever
                    // keeps the dodger moving forward rather than reversing into
                    // the detour), but the actual detour direction is recomputed
                    // from the live line of centres every frame below.
                    //
                    // Latching the vector instead looks equivalent and is not:
                    // in a crossing the bearing between the two robots rotates
                    // as they converge, so a frozen vector goes stale and the
                    // dodge stops pointing anywhere useful. In sim that pinned
                    // perpendicular encounters at ~55mm separation regardless of
                    // trigger distance or blend — a parameter-invariant failure,
                    // which is what gives it away as structural. Head-on hides
                    // the bug because that bearing barely moves.
                    Latch l;
                    l.dodger = dodger;
                    l.side   = (hx * -ny + hy * nx) >= 0.f ? +1 : -1;
                    it = latches_.emplace(key, l).first;
                }

                // ── Latched: hold until released by distance or timeout ───────
                Latch& l = it->second;

                // ...but a nomination is only valid while the dodger can still
                // act on it. If it has arrived at its goal and parked, the latch
                // is pointing at a robot that will never move, while the
                // priority robot — never slowed — keeps driving into it. Drop
                // the latch so the trigger logic re-nominates on the spot,
                // which hands the dodge to the robot that is still moving.
                //
                // The timeout alone is not enough here: it recovers eventually,
                // but the pair closed to 56mm (inside contact) during the wait,
                // versus 108mm for the same geometry with neither robot parking.
                // Note the reverse case is already safe — when the PRIORITY
                // robot parks the latch stays correct, since the dodger is still
                // the one moving and still the one that has to get around.
                if (!isMoving(l.dodger)) { latches_.erase(it); continue; }

                l.age += dt;
                if (l.age > p_.maxDodgeS) { latches_.erase(it); continue; }
                l.touched = true;

                const int dodger = l.dodger;
                const int other  = (dodger == lo) ? hi : lo;

                // Detour recomputed from the CURRENT bearing, on the latched
                // side: perpendicular to the live line of centres.
                const float arcDx = (float)l.side * -ny;
                const float arcDy = (float)l.side *  nx;

                out[dodger].dodging = true;
                out[dodger].arcDx  += arcDx;
                out[dodger].arcDy  += arcDy;

                // Emergency: stop the robot WITHOUT a plan, let the one WITH a
                // plan carry it out. Inside emergencyMm the dodge has failed and
                // the priority robot is the one still charging, so halting it
                // removes the closing speed while the dodger — which already has
                // a committed detour — keeps maneuvering and opens the gap. Once
                // it does, this clears and the priority robot resumes.
                //
                // Both other options were simulated and both are worse:
                // stopping only the dodger leaves the priority robot to drive
                // through it (head-on closed to 6mm), and stopping BOTH is an
                // inescapable freeze — neither robot can move, so the distance
                // never changes and the emergency never lifts (a perpendicular
                // crossing locked solid at 110mm and stayed there). An emergency
                // rule needs an escape, and only the dodger can provide one.
                // ...but only if it is actually CLOSING. In an overtake the
                // priority robot is the one in front, driving away — stopping it
                // there deletes the separation it was creating and lets the
                // dodger behind catch right up (sim: 128mm clearance collapsed
                // to 55mm). Stop a robot that is charging, never one that is
                // already leaving.
                if (dist < p_.emergencyMm) {
                    const float ohx = std::cos(poses.at(other).yaw * (float)M_PI / 180.f);
                    const float ohy = std::sin(poses.at(other).yaw * (float)M_PI / 180.f);
                    const float tox = (other == lo) ?  nx : -nx;
                    const float toy = (other == lo) ?  ny : -ny;
                    if (ohx * tox + ohy * toy > 0.f) out[other].emergency = true;
                }

                // The protected robot. Marked even if it is itself dodging
                // someone else — applyAvoidance() resolves that, dodging wins.
                out[other].priority = true;

                auto& fb = fallback[dodger];
                if (dist < fb.dist) { fb.dist = dist; fb.dx = arcDx; fb.dy = arcDy; }
            }
        }

        // Drop latches for pairs that no longer exist (robot left the arena,
        // marker lost) — otherwise the map grows without bound over a long run.
        for (auto it = latches_.begin(); it != latches_.end(); ) {
            if (!it->second.touched) it = latches_.erase(it);
            else                     ++it;
        }

        for (auto& [id, st] : out) {
            if (!st.dodging) continue;
            const float len = std::sqrt(st.arcDx * st.arcDx + st.arcDy * st.arcDy);
            if (len > 1e-3f) {
                st.arcDx /= len;
                st.arcDy /= len;
            } else {
                // Opposing threats cancelled. Without the old speed ramp there
                // is no safety net behind a zero-length detour, so fall back to
                // dodging the nearest one and accept being wrong about the rest.
                auto f = fallback.find(id);
                if (f != fallback.end()) { st.arcDx = f->second.dx; st.arcDy = f->second.dy; }
                else                     { st.dodging = false; }
            }
        }
        return out;
    }

private:
    using Key = std::pair<int, int>;   // {lo, hi}, lo < hi

    struct Latch {
        int   dodger  = -1;
        int   side    = +1;   // which way around: +1 = left of the lo→hi normal
        float age     = 0.f;
        bool  touched = false;
    };

    AvoidParams          p_;
    std::map<Key, Latch> latches_;
};

// ── Applying the advice ───────────────────────────────────────────────────────

struct AvoidAction {
    bool  hardStop = false;  // caller must zero the motors and skip the controller
    float maxSpd   = 0.f;    // speed cap — UNSCALED unless this robot is dodging
    float arcX     = 0.f;    // detour direction to bend the goal vector toward
    float arcY     = 0.f;
    float arcBlend = 0.f;    // how much of it to apply
};

// baseMaxSpd is the robot's normal cap (MAX_SPEED * per-robot multiplier).
//
// Note what is NOT here any more: there is no proximity speed ramp. A robot that
// is not dodging comes out at full speed even with a neighbour well inside
// safeMm. That is deliberate — the priority robot clearing the area is what
// resolves the conflict, and slowing it down is what used to deadlock the pair.
inline AvoidAction applyAvoidance(const AvoidState& av, float baseMaxSpd,
                                  const AvoidParams& p = {})
{
    AvoidAction a;
    a.maxSpd = baseMaxSpd;

    // Emergency is checked first, before the dodging/priority split: it is set
    // on BOTH robots of a failed dodge, so a priority robot must be able to stop.
    if (av.emergency) {
        a.hardStop = true;
        a.maxSpd   = 0.f;
        return a;
    }

    // Dodging is checked before priority: a robot can be both (dodging robot A
    // while being the protected one for robot B), and the detour must win.
    if (!av.dodging) return a;

    a.maxSpd   = baseMaxSpd * p.dodgeSpeedFrac;
    a.arcX     = av.arcDx;
    a.arcY     = av.arcDy;
    a.arcBlend = p.blend;
    return a;
}

} // namespace swarmctl
