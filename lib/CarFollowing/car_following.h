// car_following.h — the seven car-following models of the Sugiyama et al.
// (2007) ring-road experiment, as pure functions.
//
// Transcribed from the NetLogo source embedded in
// tools/car-following-models/Experiment_by_Sugiyama_et_al.__2007_.html
// (the `set-dynamics` procedure and the "HOW IT WORKS" section). Keeping the
// same names and constants as that page is deliberate: the tool can then be
// run side by side with the page for the same model and parameters.
//
// Units are SI throughout — metres and seconds, exactly as in the paper.
// A physical ring of robots is mapped onto this scale by the caller; see
// tools/vision/car_following.cpp.
//
// Deliberately free of OpenCV, SwarmClient and any I/O, so it is directly
// unit-testable (tests/test_car_following.cpp) and costs a subscriber
// nothing — same discipline as lib/ArucoTracker/pose_hub.h.

#pragma once

#include <algorithm>
#include <cmath>
#include <strings.h>   // strcasecmp

enum class CfModel { Reuschel, Pipes, OVM, CfOVM, FVDM, ATG, IDM };

// Order matches the NetLogo `Model` chooser; the strings are its option
// labels verbatim, so a value read off the live page parses directly.
inline const char* cfModelName(CfModel m) {
    switch (m) {
        case CfModel::Reuschel: return "Reuschel";
        case CfModel::Pipes:    return "Pipes";
        case CfModel::OVM:      return "OVM";
        case CfModel::CfOVM:    return "CF-OVM";
        case CfModel::FVDM:     return "FVDM";
        case CfModel::ATG:      return "ATG";
        case CfModel::IDM:      return "IDM";
    }
    return "?";
}

inline bool cfModelFromName(const char* s, CfModel& out) {
    for (int i = 0; i <= (int)CfModel::IDM; ++i) {
        if (strcasecmp(s, cfModelName((CfModel)i)) == 0) { out = (CfModel)i; return true; }
    }
    return false;
}

// The five parameters the NetLogo page exposes as sliders / chooser, with its
// slider defaults.
struct CfParams {
    float speedMax     = 15.0f;  // m/s   — desired (maximum) speed
    float carSize      = 5.0f;   // m     — vehicle length
    float timeGap      = 0.8f;   // s     — desired time gap
    float reactionTime = 0.7f;   // s
    float sigma        = 0.0f;   // noise amplitude, 0 = deterministic
};

// Constants the page fixes in the script rather than exposing as sliders.
constexpr float CF_SPEED_ALIGN_TIME = 3.0f;  // FVDM, s
constexpr float CF_IDM_A            = 2.0f;  // IDM desired accel, m/s^2
constexpr float CF_IDM_B            = 4.0f;  // IDM max decel,     m/s^2
constexpr float CF_IDM_S0           = 1.0f;  // IDM minimum gap,   m

// One vehicle's view of the road ahead.
//
// `gap` is the *clear* distance to the predecessor — bumper to bumper, i.e.
// already net of carSize. That is what NetLogo's `gap` reporter returns and
// what every formula below is written against.
struct CfInput {
    float gap;        // m,   clear distance to the predecessor
    float speed;      // m/s, own speed
    float predSpeed;  // m/s, predecessor's speed
    float predGap;    // m,   predecessor's own clear gap (CF-OVM only)
};

// V(s): the piecewise-linear optimal-velocity function shared by
// Reuschel, OVM, CF-OVM and FVDM.
inline float cfOptimalVelocity(float gap, const CfParams& p) {
    if (p.timeGap <= 0.f) return p.speedMax;
    return std::max(0.f, std::min(p.speedMax, gap / p.timeGap));
}

// Smooth max (eps > 0) / smooth min (eps < 0), falling back to the hard
// operation once the exponent would overflow. Used only by ATG.
//
// Evaluated in double, like the NetLogo original. The ±700 cutoff is a double's
// exp() range; a float overflows to inf around exp(89) and flushes to zero
// around exp(-88), so the typical ATG argument (eps = 0.01 puts a time gap of
// 2s at exp(200)) would come back as inf well inside the guard.
inline float cfLogSumExp(float a, float b, float eps) {
    double ra = (double)a / eps, rb = (double)b / eps;
    if (std::fabs(ra) < 700.0 && std::fabs(rb) < 700.0)
        return (float)(eps * std::log(std::exp(ra) + std::exp(rb)));
    return eps > 0.f ? std::max(a, b) : std::min(a, b);
}

// dv_n/dt for the five second-order models. Reuschel and CF-OVM set the speed
// directly and have no acceleration — they return 0 here and are handled in
// cfStep().
inline float cfAcceleration(CfModel m, const CfInput& in, const CfParams& p) {
    const float dv = in.predSpeed - in.speed;   // Dv_n
    const float rt = std::max(p.reactionTime, 1e-3f);

    switch (m) {
        case CfModel::Pipes:
            return dv / rt;

        case CfModel::OVM:
            return (cfOptimalVelocity(in.gap, p) - in.speed) / rt;

        case CfModel::FVDM:
            return (cfOptimalVelocity(in.gap, p) - in.speed) / rt
                 + dv / CF_SPEED_ALIGN_TIME;

        case CfModel::ATG: {
            // T_n: the vehicle's current time gap, smoothly clamped to
            // [0.1s, 2s] so it can be divided by even when stopped.
            float tg = in.gap / std::max(in.speed, 1e-3f);
            float T  = cfLogSumExp(cfLogSumExp(tg, 2.f, -0.01f), 0.1f, 0.01f);
            float T0 = std::max(p.timeGap, in.gap / std::max(p.speedMax, 1e-3f));
            return (dv + (in.gap - T0 * in.speed) / 5.f) / T;
        }

        case CfModel::IDM: {
            float ss = CF_IDM_S0 + in.speed * p.timeGap
                     - in.speed * dv / (2.f * std::sqrt(CF_IDM_A * CF_IDM_B));
            float s  = ss / std::max(in.gap, 1e-3f);
            float v  = in.speed / std::max(p.speedMax, 1e-3f);
            // The deceleration floor is the page's guard against the large
            // dt driving the speed negative (see "HOW IT WORKS").
            return std::max(-3.f * in.speed,
                            CF_IDM_A * (1.f - s * s - v * v * v * v));
        }

        default:
            return 0.f;
    }
}

// One explicit-Euler step: returns the vehicle's new speed in m/s.
//
// `noise` is a unit gaussian sample supplied by the caller, so this stays a
// pure function; it is ignored unless params.sigma > 0.
inline float cfStep(CfModel m, const CfInput& in, const CfParams& p,
                    float dt, float noise = 0.f) {
    float v;
    switch (m) {
        case CfModel::Reuschel:
            v = cfOptimalVelocity(in.gap, p);
            break;

        case CfModel::CfOVM:
            v = cfOptimalVelocity(in.gap - p.reactionTime *
                    (cfOptimalVelocity(in.predGap, p) - cfOptimalVelocity(in.gap, p)), p);
            break;

        default:
            v = in.speed + cfAcceleration(m, in, p) * dt;
            break;
    }

    if (p.sigma > 0.f) {
        // A logistic in the current speed gates the noise off near standstill,
        // so a stopped vehicle is never kicked backwards. Note the NetLogo
        // source multiplies by sigma twice (the write-up says once) — kept as
        // written so runs match the reference page.
        float gate = 1.f / (1.f + std::exp(std::min(700.f, -1e3f * (in.speed - 0.1f))));
        v += p.sigma * p.sigma * std::sqrt(dt) * gate * noise;
    }
    return v;
}

// ── Cooperative buffering ────────────────────────────────────────────────────
//
// Transcribed from "Stop-and-Go Mitigation via Cooperative Buffering"
// (Korbmacher & Tordeux, Univ. Wuppertal) — the companion NetLogo page to the
// Sugiyama one above, same authors, same ring:
//
//     to move
//       ask vehicles [
//         let T (20 - B) / 19          ; N = 20, nominal time gap 1s
//         if color = cyan [set T B]    ; the buffering vehicle
//         ...
//
// One designated vehicle — the *buffering* vehicle — enlarges its desired time
// gap to B*T, and the other N-1 shrink theirs to T*(N-B)/(N-1), so the mean
// desired time gap stays T for every B. That compensation is the whole point
// on a ring: the vehicles are boxed in by one another, so enlarging one gap
// without shrinking the rest is not a strategy, it is just a lower density.
// B = 1 puts everyone back on T — the non-cooperative baseline. Larger B buys
// the buffering vehicle room to decelerate gently, and the wave dies at it.
//
// Note this is a transformation of the *desired spacing*, not of the IDM in
// particular. Every model here that has a desired spacing takes it the same
// way: Reuschel, OVM, CF-OVM and FVDM through V(s) = min(vmax, s/T), ATG
// through T0, IDM through s*. Pipes is the sole exception — dv/dt = Dv/tau
// never reads the gap at all, so there is nothing to inflate and buffering is
// a no-op there; cfModelHasDesiredGap() reports that, so a caller can say so
// rather than appear to do something.

// Whether the model's dynamics depend on a desired spacing — i.e. whether
// buffering (or the time-gap parameter at all) means anything for it.
inline bool cfModelHasDesiredGap(CfModel m) { return m != CfModel::Pipes; }

// A follower's time gap can be driven to zero by B = N, and a zero timeGap is
// not "no room wanted" but the free-flow branch of V(s) / T0 — the exact
// opposite of a fully committed buffer. Floor it just above that instead.
constexpr float CF_MIN_TIME_GAP = 0.01f;  // s

// The desired time gap for one vehicle under buffering, out of `n` on the ring.
// `b` is clamped to [1, n]: beyond n the followers' share goes negative.
inline float cfBufferedTimeGap(float timeGap, float b, int n, bool isBuffer) {
    if (n < 2) return timeGap;                    // nobody to share the gap with
    b = std::max(1.f, std::min(b, (float)n));
    float t = isBuffer ? timeGap * b
                       : timeGap * ((float)n - b) / ((float)n - 1.f);
    return std::max(CF_MIN_TIME_GAP, t);
}

// The same, as a ready-to-use parameter set for cfStep()/cfAcceleration().
inline CfParams cfBufferedParams(const CfParams& p, float b, int n, bool isBuffer) {
    CfParams q = p;
    q.timeGap  = cfBufferedTimeGap(p.timeGap, b, n, isBuffer);
    return q;
}

// The largest B worth offering for `n` vehicles: the value at which the
// followers are down to half the nominal time gap. The reference page's slider
// stops at 10 for its 20 vehicles, which is this bound to within its integer
// step — and unlike a fixed 10 it stays sane for the three or four robots that
// actually fit on the arena ring.
inline float cfMaxBuffering(int n) {
    return n < 2 ? 1.f : ((float)n + 1.f) / 2.f;
}
