// battery_log.h — the camera-free half of tools/vision/battery_log.cpp.
//
// The tool drives one robot around the saved ring at a fixed motor command
// until its battery is flat, logging camera-measured speed next to the
// battery voltage the robot reports. Everything here is the bookkeeping that
// does not need a camera, a hub or OpenCV, so it can be unit-tested
// (tests/test_battery_log.cpp) — the same split as lib/CarFollowing/.
//
//   BlSchedule        — the orbit/rest duty cycle
//   BlRowAccumulator  — per-frame poses -> one log row (tangential speed)
//   BlStop            — when a run is over (low voltage, stall, lost robot)
//   blRollingMedian   — the smoothed trace on the tool's speed plot
//
// Why the measurement is meaningful at all: the robots run open-loop
// (ODOMETRY_ENABLED = False in src/robots/uart_controller.py), so a motor
// command is a fixed PWM duty and the wheels' speed follows the battery
// voltage. With the encoder PID on, the robot would hold its speed until the
// motors saturate and this log would show nothing until the very end.

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

// ── Phases ───────────────────────────────────────────────────────────────────

enum class BlPhase { Setup, Seek, Orbit, Rest, Done };

inline const char* blPhaseName(BlPhase p) {
    switch (p) {
        case BlPhase::Setup: return "setup";
        case BlPhase::Seek:  return "seek";
        case BlPhase::Orbit: return "orbit";
        case BlPhase::Rest:  return "rest";
        case BlPhase::Done:  return "done";
    }
    return "?";
}

// ── Duty cycle ───────────────────────────────────────────────────────────────
// A rest window holds the motors at zero so the log also records the battery's
// resting voltage. The robot samples its battery every 2 s (BAT_INTERVAL_MS),
// so a rest has to be several of those long to catch more than one reading.
// The gap between loaded and resting voltage is a second signal — it is the
// cell's internal resistance times the motor current.
//
// Only the Orbit -> Rest -> Seek edges live here. Seek -> Orbit is geometric
// (the robot reached the ring), which is the tool's to decide. A rest ends in
// Seek rather than Orbit so a robot that drifted while stopped is put back on
// the ring before its speed is measured again.

struct BlSchedule {
    double restEveryS = 120.0;   // orbit time between rests; <= 0 disables rests
    double restForS   = 12.0;

    BlPhase next(BlPhase p, double inPhaseS) const {
        if (restEveryS <= 0.0) return p;
        if (p == BlPhase::Orbit && inPhaseS >= restEveryS) return BlPhase::Rest;
        if (p == BlPhase::Rest  && inPhaseS >= restForS)   return BlPhase::Seek;
        return p;
    }
};

// ── One log row ──────────────────────────────────────────────────────────────
// Speed is each frame's displacement projected onto the ring's tangent at the
// midpoint of that step, summed over the row and divided by the time those
// steps covered. Not the sum of |displacement|: position noise always adds to
// a magnitude, so hypot() of per-frame deltas reads fast by an amount that
// depends on the frame rate, while a signed projection averages the noise
// out. Radial wobble from the heading controller drops out the same way.
//
// A frame without a pose breaks the chain — the next pose starts a new pair
// rather than being differenced across the gap — and the gap's time is not
// counted, so a dropout neither inflates nor deflates the speed.

struct BlRow {
    double dtS          = 0.0;    // wall time the row spans
    int    nFrames      = 0;      // frames with a pose
    int    nPairs       = 0;      // consecutive-pose pairs that fed speedMms
    float  speedMms     = NAN;    // along the direction of travel; NAN without a pair
    float  radialErrMm  = NAN;    // mean (distance from centre - radius)
    float  absTurn      = NAN;    // mean |turn| the controller commanded
    bool   visible() const { return nFrames > 0; }
};

class BlRowAccumulator {
public:
    // dirSign +1 = counter-clockwise, as in car_following.cpp.
    void begin(double t) { *this = BlRowAccumulator(); start_ = t; }

    void frame(bool havePose, float x, float y,
               float cx, float cy, float radius, float dirSign, float turn, double t) {
        if (!havePose) { havePrev_ = false; return; }
        ++nFrames_;
        float dx = x - cx, dy = y - cy;
        sumRadial_ += std::hypot(dx, dy) - radius;
        sumAbsTurn_ += std::fabs(turn);
        if (havePrev_ && t > prevT_) {
            float mx = 0.5f * (x + prevX_) - cx, my = 0.5f * (y + prevY_) - cy;
            float m  = std::hypot(mx, my);
            if (m >= 1.f) {
                float tx = dirSign * -my / m, ty = dirSign * mx / m;
                sumTan_  += (x - prevX_) * tx + (y - prevY_) * ty;
                sumTime_ += t - prevT_;
                ++nPairs_;
            }
        }
        havePrev_ = true; prevX_ = x; prevY_ = y; prevT_ = t;
    }

    // Closes the row at t and starts the next one there. The last pose is
    // carried over, so the step that straddles two rows is counted in the
    // second one instead of being dropped.
    BlRow take(double t) {
        BlRow r;
        r.dtS     = t - start_;
        r.nFrames = nFrames_;
        r.nPairs  = nPairs_;
        if (nPairs_ > 0 && sumTime_ > 0.0) r.speedMms = (float)(sumTan_ / sumTime_);
        if (nFrames_ > 0) {
            r.radialErrMm = (float)(sumRadial_ / nFrames_);
            r.absTurn     = (float)(sumAbsTurn_ / nFrames_);
        }
        bool hp = havePrev_; float px = prevX_, py = prevY_; double pt = prevT_;
        begin(t);
        havePrev_ = hp; prevX_ = px; prevY_ = py; prevT_ = pt;
        return r;
    }

    double startS() const { return start_; }

private:
    double start_ = 0.0;
    int    nFrames_ = 0, nPairs_ = 0;
    double sumTan_ = 0.0, sumTime_ = 0.0, sumRadial_ = 0.0, sumAbsTurn_ = 0.0;
    bool   havePrev_ = false;
    float  prevX_ = 0.f, prevY_ = 0.f;
    double prevT_ = 0.0;
};

// ── Stop criteria ────────────────────────────────────────────────────────────
// Voltage is debounced by *time* below the threshold rather than by a count
// of readings: telemetry repeats the robot's last 2-second sample many times
// over, so the PC cannot tell a new reading from a repeated one.
//
// The stall baseline is the mean orbit speed over the first baselineS of
// orbiting. A stall is orbit rows below stallFrac of that for stallS without
// a break — long enough that a bump or a reseek does not end a run that took
// hours to get there.

enum class BlStopReason { None, LowVoltage, LowBatteryFlag, Stall, SeekTimeout, RobotLost };

inline const char* blStopReasonName(BlStopReason r) {
    switch (r) {
        case BlStopReason::None:           return "none";
        case BlStopReason::LowVoltage:     return "low voltage";
        case BlStopReason::LowBatteryFlag: return "low-battery flag";
        case BlStopReason::Stall:          return "stalled";
        case BlStopReason::SeekTimeout:    return "never reached the ring";
        case BlStopReason::RobotLost:      return "robot out of view";
    }
    return "?";
}

struct BlStopConfig {
    int    stopMv      = 4000;   // 1.0 V/cell on four NiMH AAA
    double lowHoldS    = 6.0;    // ~3 of the robot's 2 s samples
    double baselineS   = 60.0;
    float  stallFrac   = 0.2f;
    double stallS      = 10.0;
    double seekTimeoutS = 30.0;
    double lostS       = 60.0;
};

class BlStop {
public:
    explicit BlStop(BlStopConfig c = {}) : cfg_(c) {}

    // The latest battery telemetry. lowFlag is STATUS_LOW_BATTERY — nothing
    // in the firmware sets it today, but a run should end if something does.
    void battery(double t, int mv, bool valid, bool lowFlag) {
        if (lowFlag) { stop(BlStopReason::LowBatteryFlag); return; }
        if (!valid) return;
        if (mv <= cfg_.stopMv) {
            if (lowSince_ < 0.0) lowSince_ = t;
            if (t - lowSince_ >= cfg_.lowHoldS) stop(BlStopReason::LowVoltage);
        } else {
            lowSince_ = -1.0;
        }
    }

    // One finished row. orbitS is the run's accumulated orbit time at the end
    // of the row (not wall time), so rests and reseeks do not eat the
    // baseline window.
    void row(double t, BlPhase phase, const BlRow& r, double orbitS) {
        if (phase != BlPhase::Orbit || std::isnan(r.speedMms)) { slowSince_ = -1.0; return; }
        if (orbitS <= cfg_.baselineS) {
            baseSum_ += r.speedMms * r.dtS;
            baseTime_ += r.dtS;
            return;
        }
        float base = baseline();
        if (base <= 0.f) return;
        if (r.speedMms < cfg_.stallFrac * base) {
            if (slowSince_ < 0.0) slowSince_ = t - r.dtS;
            if (t - slowSince_ >= cfg_.stallS) stop(BlStopReason::Stall);
        } else {
            slowSince_ = -1.0;
        }
    }

    // Called every frame: catches a seek that never arrives and a robot that
    // has left the arena, neither of which produces orbit rows.
    void tick(BlPhase phase, double inPhaseS, double unseenS) {
        if (phase == BlPhase::Seek && inPhaseS >= cfg_.seekTimeoutS) stop(BlStopReason::SeekTimeout);
        if (phase != BlPhase::Setup && unseenS >= cfg_.lostS)        stop(BlStopReason::RobotLost);
    }

    float baseline() const { return baseTime_ > 0.0 ? (float)(baseSum_ / baseTime_) : 0.f; }
    BlStopReason reason() const { return reason_; }
    bool stopped() const { return reason_ != BlStopReason::None; }
    const BlStopConfig& config() const { return cfg_; }

private:
    void stop(BlStopReason r) { if (reason_ == BlStopReason::None) reason_ = r; }

    BlStopConfig cfg_;
    BlStopReason reason_ = BlStopReason::None;
    double lowSince_  = -1.0;
    double slowSince_ = -1.0;
    double baseSum_ = 0.0, baseTime_ = 0.0;
};

// ── Plot smoothing ───────────────────────────────────────────────────────────
// Trailing (causal) median over windowS: out[i] is the median of every
// non-NaN v[j] with t[i] - windowS < t[j] <= t[i]. A median rather than a
// mean so a reseek's slow row or one bad detection does not bend the trace.
// NaN where the window holds nothing.

inline std::vector<float> blRollingMedian(const std::vector<double>& t,
                                          const std::vector<float>& v, double windowS) {
    std::vector<float> out(v.size(), NAN);
    std::vector<float> win;
    size_t lo = 0;
    for (size_t i = 0; i < v.size() && i < t.size(); ++i) {
        while (lo < i && t[lo] <= t[i] - windowS) ++lo;
        win.clear();
        for (size_t j = lo; j <= i; ++j)
            if (!std::isnan(v[j])) win.push_back(v[j]);
        if (win.empty()) continue;
        size_t mid = win.size() / 2;
        std::nth_element(win.begin(), win.begin() + mid, win.end());
        float m = win[mid];
        if (win.size() % 2 == 0) {
            float lower = *std::max_element(win.begin(), win.begin() + mid);
            m = 0.5f * (m + lower);
        }
        out[i] = m;
    }
    return out;
}
