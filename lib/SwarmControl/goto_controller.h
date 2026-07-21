#pragma once
// ─── Shared differential-drive go-to-point controller ─────────────────────────
//
// Extracted from the inline control loops that were copy-pasted across
// circle_demo / shape_demo / drag_drop_demo / wingman / vision_controller.
// The *shape* of the computation is shared; the gains are NOT — every demo was
// tuned against real hardware and keeps its own GotoParams. Do not "unify" the
// constants below into one blessed set: MAX_SPEED alone legitimately ranges
// from 51.7 (vision_controller, shape_demo) to 100.0 (circle_demo).
//
// The control law, in one place:
//
//   angleErr  = normAngle(atan2(dy,dx) - yaw)
//   headingSc = 1 - clamp(|angleErr|/90,0,1)^2     ← don't drive sideways
//   brakeSc   = clamp((dist-arrival)/brakeSpan,0,1) ← ease into a terminal stop
//   forward   = clamp(kDist*dist, 0, maxSpd) * headingSc * brakeSc
//   turn      = clamp(turnFF + kAngle*angleErr + kYawD*dAngleErr, ±maxTurn)
//   left/right= clamp(forward ± turn, -100, 100)
//
// Header-only, no OpenCV dependency — takes a plain (dx, dy, yaw) so it can be
// unit-tested without a camera. See tests/test_goto_controller.cpp.

#include <cmath>
#include <cstdint>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace swarmctl {

// ── Math helpers (were duplicated verbatim in all five demos) ─────────────────

inline float normAngle(float a) {
    while (a >  180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

// ── Motor command ─────────────────────────────────────────────────────────────

struct MotorCmd {
    int8_t left  = 0;
    int8_t right = 0;
};

// ── Controller parameters ─────────────────────────────────────────────────────
// Defaults match drag_drop_demo, the most complete of the extracted loops.
// Every caller should still set these explicitly — see the warning above.

struct GotoParams {
    float kDist     = 0.40f;   // forward P-gain (motor units per mm of error)
    float kAngle    = 0.50f;   // heading P-gain
    float kYawD     = 0.08f;   // heading D-gain; 0 disables the derivative term
    float maxTurn   = 16.0f;   // turn magnitude cap, motor units
    float arrivalMm = 20.0f;   // used as the brake ramp's zero point

    // Terminal braking. Path-following callers (shape_demo) set brake=false:
    // a looping path has no terminal stop to ease into, and the carrot is
    // always at least LOOKAHEAD_MM away, so a brake term would only sap speed.
    bool  brake       = true;
    float brakeSpanMm = 0.f;   // ramp width above arrivalMm; 0 ⇒ use arrivalMm

    // Turn slew limit (circle_demo). 0 disables. Bounds how fast `turn` may
    // change per second so a heading spike can't step-change the motors.
    float maxTurnRate = 0.f;

    // Derivative-term guard: dAngleErr is clamped to ±this (deg/s) so a single
    // bad ArUco frame can't inject a huge spike through kYawD.
    float maxAngleRate = 300.f;
};

// ── Per-robot controller memory ───────────────────────────────────────────────
// The D term and the slew limiter are stateful, so each robot needs its own.

struct GotoState {
    float prevAngleErr = 0.f;
    bool  hasPrev      = false;
    float lastTurn     = 0.f;
};

// ── The controller ────────────────────────────────────────────────────────────
//
// dx, dy   goal minus current position, world mm
// yawDeg   robot heading, degrees CCW from world +X (smoothed — see YawSmoother)
// maxSpd   speed cap for THIS robot this frame, motor units. Callers fold their
//          per-robot speed multiplier and any avoidance slow-down in here.
// dt       control loop dt, seconds (clamp it before calling)
// turnFF   turn feed-forward, motor units — circle_demo's orbit term. The
//          caller pre-scales it (e.g. by headingSc) if that's the intent.
//
// Arrival is deliberately NOT handled here: callers stop for different reasons
// (drag_drop clears the goal, shape_demo advances the carrot), and they already
// compute `dist` for their HUD. Check `dist < arrivalMm` and skip the call.

inline MotorCmd computeGoto(float dx, float dy, float yawDeg, float maxSpd,
                            const GotoParams& p, GotoState& st, float dt,
                            float turnFF = 0.f)
{
    const float dist     = std::sqrt(dx * dx + dy * dy);
    const float tgtAngle = std::atan2(dy, dx) * 180.f / (float)M_PI;
    const float angleErr = normAngle(tgtAngle - yawDeg);

    // Quadratic heading scale: full speed when aimed at the goal, zero at 90°
    // off. Squaring (rather than a linear taper) keeps the robot moving through
    // small heading errors instead of crawling.
    const float headingN  = clampf(std::fabs(angleErr) / 90.f, 0.f, 1.f);
    const float headingSc = 1.f - headingN * headingN;

    float brakeSc = 1.f;
    if (p.brake) {
        const float span = p.brakeSpanMm > 0.f ? p.brakeSpanMm : p.arrivalMm;
        brakeSc = clampf((dist - p.arrivalMm) / span, 0.f, 1.f);
    }

    // Derivative of heading error. dt is the caller's clamped controlDt, so
    // this stays finite even on a stalled frame.
    float dAngleErr = 0.f;
    if (st.hasPrev && dt > 0.f) {
        dAngleErr = clampf(normAngle(angleErr - st.prevAngleErr) / dt,
                           -p.maxAngleRate, p.maxAngleRate);
    }
    st.prevAngleErr = angleErr;
    st.hasPrev      = true;

    const float forward = clampf(p.kDist * dist, 0.f, maxSpd) * headingSc * brakeSc;

    float turn = clampf(turnFF + p.kAngle * angleErr + p.kYawD * dAngleErr,
                        -p.maxTurn, p.maxTurn);

    if (p.maxTurnRate > 0.f && dt > 0.f) {
        const float maxStep = p.maxTurnRate * dt;
        turn = clampf(turn, st.lastTurn - maxStep, st.lastTurn + maxStep);
    }
    st.lastTurn = turn;

    MotorCmd cmd;
    cmd.left  = (int8_t)clampf(forward + turn, -100.f, 100.f);
    cmd.right = (int8_t)clampf(forward - turn, -100.f, 100.f);
    return cmd;
}

// ── Yaw smoothing ─────────────────────────────────────────────────────────────
//
// Low-pass on the tracked heading, specified as a time-constant (seconds), NOT
// a fixed per-frame EMA coefficient: each frame the alpha is derived as
// dt/(tau+dt) from the actual frame dt, so the filter holds the same memory in
// *time* at any loop rate. A fixed per-frame alpha silently over-smooths
// (heading lag → limit-cycle) when the loop runs slower than it was tuned for —
// e.g. on a weaker PC or at higher camera resolution. 0.50s is the value found
// to kill that oscillation in circle_demo; lower toward ~0.10s on a fast
// camera/PC if the lag visibly cuts corners.
//
// Deltas are wrapped through normAngle, so this filter is correct across the
// ±180° seam — a plain EMA on raw degrees would spin the long way round.

class YawSmoother {
public:
    explicit YawSmoother(float tauS = 0.50f) : tau_(tauS) {}

    // The accumulator is re-wrapped to ±180 each step (shape_demo's form). Both
    // pre-extraction variants — wrapped and unwrapped — are equivalent for every
    // consumer, since they all normalize a *difference* of angles; keeping the
    // stored value canonical just avoids unbounded drift on a robot that spins
    // the same way for a long time.
    float update(int id, float rawYawDeg, float dt) {
        auto [it, fresh] = yaw_.emplace(id, rawYawDeg);
        if (!fresh) {
            const float alpha = dt / (tau_ + dt);
            const float delta = normAngle(rawYawDeg - it->second);
            it->second = normAngle(it->second + alpha * delta);
        }
        return it->second;
    }

    void forget(int id) { yaw_.erase(id); }
    void clear()        { yaw_.clear(); }
    float tau() const   { return tau_; }

private:
    float tau_;
    std::unordered_map<int, float> yaw_;
};

} // namespace swarmctl
