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

// ── Closing the loop against a real robot ────────────────────────────────────
//
// `in.speed` is the vehicle's own *state*, not a fresh measurement. Every
// second-order model returns `in.speed + a*dt`, so handing cfStep a measured
// speed each step closes a loop with gain
//
//     (1 - dt/reactionTime) * k
//
// around the measurement, where k is whatever gain error the measurement
// carries. dt/reactionTime is small (0.025/0.7 = 3.6% at the tool's defaults),
// so any k above ~1.04 diverges and the command pins at speedMax — the model
// stops running at all. On the ring tool k is ring.radius / the robot's actual
// orbit radius, because road position is an angle: a robot orbiting 5% inside
// the ring covers road 5% faster than it is really driving.
//
// So the state is integrated, and the measurement is folded in as a slow
// first-order correction instead. With blend coefficient alpha the loop gain
// becomes
//
//     (1 - dt/reactionTime) * (1 + alpha*(k - 1))
//
// which stays below 1 for any plausible k once tau >> reactionTime — at the
// default 2s against a reaction time of 0.7s, a robot orbiting at *half* the
// ring radius (k = 2) still converges, and the settled speed is within 2% of
// V(gap) for the few-percent errors that actually occur. Re-seeding, by
// contrast, is already commanding 2.2x V(gap) at k = 1.02 and pins at speedMax
// past 1.04. Both are pinned down in tests/test_car_following.cpp.
constexpr float CF_SYNC_TAU_S = 2.0f;   // s, simulated

// EMA coefficient for that correction, from a time constant, so the blend keeps
// the same memory in simulated seconds whatever dt the caller steps with.
// tau <= 0 returns 1 — i.e. replace the state with the measurement outright,
// which is the unstable behaviour described above and is only useful in tests.
inline float cfSyncAlpha(float dt, float tauS = CF_SYNC_TAU_S) {
    return tauS <= 0.f ? 1.f : dt / (tauS + dt);
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
