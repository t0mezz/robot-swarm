// ring.h — the ring bookkeeping for the Sugiyama experiment: who is on the
// ring, in what order they sit, how far apart they are, and what speed each
// vehicle's model wants next. Plus the run-state machine that decides when any
// of that is allowed to reach a motor.
//
// Split out of tools/vision/car_following.cpp for the same reason
// car_following.h was: everything here is arithmetic and bookkeeping over
// angles and timestamps, none of it needs OpenCV, pylon or SwarmClient, and
// all of it used to be the part that misbehaved on hardware. The tool is left
// with vision, control and I/O.
//
// Deliberately free of OpenCV, SwarmClient and any I/O, so it is directly
// unit-testable (tests/test_car_following.cpp) — same discipline as
// lib/ArucoTracker/pose_hub.h.
//
// ── Why the model owns its speed ─────────────────────────────────────────────
//
// The vehicle's speed is the model's own state here, integrated by cfStep()
// and never overwritten by the tracker. Vision supplies the *positions*, and
// therefore the gaps; the gaps are what couple the models to each other and to
// physical reality, exactly as in the paper.
//
// The first version instead re-seeded each vehicle's speed from the measured
// one on every tick. Because cfStep() returns `speed + a*dt` for the
// second-order models, the command was then never more than one Euler step
// above what the robot had *already* achieved — and a robot's speed lags its
// command by the motor and PID dynamics, so the ring could not accelerate away
// from standstill at all. With FVDM at the page's defaults, four robots on a
// 300 mm ring and --time-scale 4, the first tick asks for 9.7 m/s^2 over
// simDt = 0.025 s, i.e. 0.24 simulated m/s, which is 2.7 mm/s of real motion —
// below the tool's own floor for commanding a motor. The wheels stayed still,
// so the measurement stayed zero, so the next tick asked for the same 2.7 mm/s.
//
// The measured speed is still computed, because it is the honest read on
// whether the robots are keeping up with the model, but it is reported, not
// fed back.

#pragma once

#include "car_following.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

// Degrees, wrapped to (-180, 180].
inline float cfNormAngleDeg(float a) {
    a = std::fmod(a + 180.f, 360.f);
    if (a < 0.f) a += 360.f;
    return a - 180.f;
}

// Degrees, wrapped to [0, 360).
inline float cfWrap360(float a) {
    a = std::fmod(a, 360.f);
    return a < 0.f ? a + 360.f : a;
}

constexpr float CF_DEG2RAD = 3.14159265358979323846f / 180.f;

// ── One vehicle on the ring ──────────────────────────────────────────────────

struct CfRingCar {
    int    id        = -1;
    float  angleDeg  = 0.f;   // [0,360) CCW from the ring centre, last seen
    double lastSeenS = 0.0;
    bool   visible   = false; // seen during the most recent frame

    float  speed     = 0.f;   // simulated m/s — the model's state (see header)
    float  measured  = 0.f;   // simulated m/s — from vision, reported only
    float  gap       = 0.f;   // simulated m, clear distance to the predecessor

    // Speed is a difference between consecutive model ticks, so a vehicle that
    // missed one has to re-seed rather than divide a whole dropout's worth of
    // travel by a single tick.
    uint64_t tickSeen     = 0;
    float    prevAngleDeg = 0.f;

    // Snapshot of `speed` taken before any model runs, so every vehicle steps
    // off the same tick rather than seeing an already-updated predecessor.
    float speedIn = 0.f;
};

struct CfRingConfig {
    float dirSign       = 1.f;              // +1 = counter-clockwise
    float simLengthM    = -1.f;             // >0 pins the virtual ring length
    float paperSpacingM = 230.f / 22.f;     // metres per vehicle in the experiment
    int   pinnedCount   = -1;               // >0 pins the vehicle count (--count)

    // How long a vehicle keeps its place on the ring after its last detection.
    // Dropouts of a frame or two are routine and a vehicle that vanished from
    // the ring would hand its follower the gap to the vehicle *beyond* it —
    // roughly twice the real one — and the follower would accelerate into a
    // robot that is still physically there.
    double holdS   = 1.0;

    // A changed vehicle count has to hold this long before it rescales the
    // virtual ring. Without it a single dropped detection moves the ring
    // length by 1/N, which rescales every gap and every speed the models see
    // (simPerMm divides by the count) for one frame and back again.
    double settleS = 1.0;
};

// ── The ring ─────────────────────────────────────────────────────────────────
//
// Per frame:  beginFrame(); observe(...) per detected robot; endFrame(t).
// Per tick:   step(model, params, simDt, radiusMm, noiseFn).

class CfRing {
public:
    CfRing() = default;
    explicit CfRing(const CfRingConfig& cfg) : cfg_(cfg) {}

    const CfRingConfig& config() const { return cfg_; }
    void setConfig(const CfRingConfig& cfg) { cfg_ = cfg; }

    // ── Observation ──────────────────────────────────────────────────────────

    void beginFrame() {
        for (auto& [id, c] : cars_) c.visible = false;
    }

    void observe(int id, float angleDeg, double tSec) {
        CfRingCar& c = cars_[id];
        if (c.id < 0) { c.id = id; c.prevAngleDeg = cfWrap360(angleDeg); }
        c.angleDeg  = cfWrap360(angleDeg);
        c.lastSeenS = tSec;
        c.visible   = true;
    }

    // Expires stale vehicles, re-sorts the ring and advances the roster
    // debounce. Everything below is only valid once this has run.
    void endFrame(double tSec) {
        for (auto it = cars_.begin(); it != cars_.end(); )
            it = (tSec - it->second.lastSeenS) > cfg_.holdS ? cars_.erase(it) : std::next(it);

        order_.clear();
        order_.reserve(cars_.size());
        for (auto& [id, c] : cars_) order_.push_back(id);
        std::sort(order_.begin(), order_.end(), [this](int a, int b) {
            return cars_[a].angleDeg < cars_[b].angleDeg;
        });

        // Roster debounce. The count only commits once it has held.
        const int live = (int)cars_.size();
        if (live != pendingCount_) { pendingCount_ = live; pendingSince_ = tSec; }
        if (live != rosterCount_ && (tSec - pendingSince_) >= cfg_.settleS) {
            rosterCount_   = live;
            rosterChanged_ = true;
        }
    }

    // ── Roster and scale ─────────────────────────────────────────────────────

    // The vehicle count the virtual ring is sized for: pinned by --count, or
    // the debounced live count.
    int rosterCount() const {
        return cfg_.pinnedCount > 0 ? cfg_.pinnedCount : rosterCount_;
    }

    int  liveCount() const { return (int)cars_.size(); }
    int  visibleCount() const {
        int n = 0;
        for (auto& [id, c] : cars_) if (c.visible) ++n;
        return n;
    }

    // One-shot: true once after the roster settles on a new count, so the
    // caller can report the rescale rather than printing it every frame.
    bool takeRosterChange() {
        bool ch = rosterChanged_;
        rosterChanged_ = false;
        return ch;
    }

    // Length of the virtual ring in simulated metres. Density is what the
    // scale factor preserves — metres per vehicle, not absolute size — so the
    // virtual ring grows with the roster unless --sim-length pins it.
    float simLengthM() const {
        if (cfg_.simLengthM > 0.f) return cfg_.simLengthM;
        return (float)rosterCount() * cfg_.paperSpacingM;
    }

    // Simulated metres per world unit (mm) for a ring of this radius.
    // Zero until there is a settled roster and a radius to divide by — which
    // is also what scaleReady() reports.
    float simPerMm(float radiusMm) const {
        if (radiusMm <= 0.f || rosterCount() <= 0) return 0.f;
        return simLengthM() / (2.f * 3.14159265358979323846f * radiusMm);
    }

    bool scaleReady(float radiusMm) const { return simPerMm(radiusMm) > 0.f; }

    // ── Model ────────────────────────────────────────────────────────────────

    // One model tick over the whole ring. `simDt` is the model's own (time
    // dilated) step; `noise()` supplies a unit gaussian and is only called
    // when params.sigma > 0.
    template <class NoiseFn>
    void step(CfModel m, const CfParams& p, float simDt, float radiusMm,
              NoiseFn&& noise) {
        const int M = (int)order_.size();
        if (M == 0 || simDt <= 0.f) return;

        const float spm = simPerMm(radiusMm);
        if (spm <= 0.f) return;

        ++tick_;

        // Pass 1: measure and snapshot. Every vehicle's gap and speed have to
        // be read before any model runs, or a vehicle would see its
        // predecessor's already-updated speed instead of this tick's.
        for (int i = 0; i < M; ++i) {
            CfRingCar& c = cars_[order_[i]];

            if (c.visible && c.tickSeen + 1 == tick_) {
                // Simulated metres per *simulated* second: the same dilation
                // the commanded speed is divided by on the way out, which is
                // what keeps the loop self-consistent.
                c.measured = cfg_.dirSign * cfNormAngleDeg(c.angleDeg - c.prevAngleDeg)
                             * CF_DEG2RAD * radiusMm * spm / simDt;
            }
            if (c.visible) {
                c.prevAngleDeg = c.angleDeg;
                c.tickSeen     = tick_;
            }

            // Angular distance to the predecessor along the direction of
            // travel, as a clear bumper-to-bumper distance in simulated metres.
            float gapDeg = 360.f;
            if (M > 1) {
                int j = (i + (cfg_.dirSign > 0.f ? 1 : M - 1)) % M;
                gapDeg = cfg_.dirSign > 0.f ? cars_[order_[j]].angleDeg - c.angleDeg
                                            : c.angleDeg - cars_[order_[j]].angleDeg;
                if (gapDeg < 0.f) gapDeg += 360.f;
            }
            c.gap     = gapDeg * CF_DEG2RAD * radiusMm * spm - p.carSize;
            c.speedIn = c.speed;
        }

        // Pass 2: step every model off that snapshot.
        for (int i = 0; i < M; ++i) {
            CfRingCar& c = cars_[order_[i]];
            const CfRingCar& pred =
                cars_[order_[(i + (cfg_.dirSign > 0.f ? 1 : M - 1)) % M]];

            // A vehicle that is not currently detected is not being commanded
            // either, so its model is held rather than integrated: it keeps
            // its place on the ring for its follower's gap, and resumes from
            // the speed it had when it was last actually driving.
            if (!c.visible) continue;

            CfInput in{c.gap, c.speedIn, pred.speedIn, pred.gap};
            float v = cfStep(m, in, p, simDt, p.sigma > 0.f ? noise() : 0.f);

            // The robots only ever go forwards around the ring: a model that
            // undershoots into reverse would have them driving into their
            // follower.
            c.speed = std::max(0.f, std::min(v, p.speedMax));
        }
    }

    // Back to rest, keeping the roster — what a re-arm does, so the next run
    // starts from standstill the way the experiment's setup does.
    void rest() {
        for (auto& [id, c] : cars_) {
            c.speed = c.measured = c.gap = c.speedIn = 0.f;
            c.tickSeen = 0;   // forces the next tick to re-seed the measurement
        }
    }

    void clear() {
        cars_.clear();
        order_.clear();
        rosterCount_ = pendingCount_ = 0;
        rosterChanged_ = false;
        tick_ = 0;
    }

    // ── Access ───────────────────────────────────────────────────────────────

    const std::vector<int>& order() const { return order_; }

    const CfRingCar* car(int id) const {
        auto it = cars_.find(id);
        return it == cars_.end() ? nullptr : &it->second;
    }

    bool has(int id) const { return cars_.count(id) != 0; }
    uint64_t tick() const { return tick_; }

private:
    CfRingConfig                        cfg_;
    std::unordered_map<int, CfRingCar>  cars_;
    std::vector<int>                    order_;

    int      rosterCount_   = 0;
    int      pendingCount_  = 0;
    double   pendingSince_  = 0.0;
    bool     rosterChanged_ = false;
    uint64_t tick_          = 0;
};

// ── Run state ────────────────────────────────────────────────────────────────
//
// The experiment has three phases and the tool used to have one: it started
// driving the moment the camera opened, on whatever the ring geometry happened
// to be and whichever robots happened to be detected in the first frame.
//
//   Setup    — robots are tracked and the ring can be edited, motors held at
//              zero. This is where a run starts and where a stop returns to.
//   Running  — the models step and the robots drive, until stopped.
//
// A start cue is *latched*, not obeyed on the spot: cues arrive from the
// NetLogo page, a key in the debug view or a line on stdin, none of which
// know whether the hub is up, the roster has settled or the ring is set. The
// run begins on the first frame where it can, and says once what it is
// waiting for.

enum class CfPhase { Setup, Running };

enum class CfRunEvent {
    None,
    Waiting,   // a start cue is latched but the rig is not ready yet
    Started,
    Stopped,
};

class CfRunState {
public:
    CfPhase phase()   const { return phase_; }
    bool    running() const { return phase_ == CfPhase::Running; }
    bool    pending() const { return wantStart_; }

    // Where the last cue came from ("page", "key", "stdin", "--start", ...),
    // for the log line the caller prints.
    const char* source() const { return source_; }

    void requestStart(const char* src) {
        source_    = src;
        wantStart_ = true;
        told_      = false;
    }

    void requestStop(const char* src) {
        source_    = src;
        wantStart_ = false;
        stopping_  = phase_ == CfPhase::Running;
        phase_     = CfPhase::Setup;
    }

    void toggle(const char* src) {
        if (phase_ == CfPhase::Running || wantStart_) requestStop(src);
        else                                          requestStart(src);
    }

    // Call once per frame. `ready` is the caller's readiness test; `why` is
    // what it is waiting for, reported once per cue.
    CfRunEvent update(bool ready) {
        if (stopping_) { stopping_ = false; return CfRunEvent::Stopped; }
        if (!wantStart_ || phase_ == CfPhase::Running) return CfRunEvent::None;
        if (!ready) {
            if (told_) return CfRunEvent::None;
            told_ = true;
            return CfRunEvent::Waiting;
        }
        phase_ = CfPhase::Running;
        return CfRunEvent::Started;
    }

private:
    CfPhase     phase_     = CfPhase::Setup;
    bool        wantStart_ = false;
    bool        stopping_  = false;
    bool        told_      = false;
    const char* source_    = "";
};
