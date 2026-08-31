// test_car_following.cpp
// Unit tests for the pure model functions in lib/CarFollowing/car_following.h.
// No test framework — plain asserts with a pass/fail tally, run via `make test`.
//
// Reference values come from the formulas in the NetLogo source and write-up
// embedded in tools/car-following-models/Experiment_by_Sugiyama_et_al.__2007_.html,
// worked through by hand below rather than copied from a run, so a transcription
// slip in the header shows up here.

#include "../lib/CarFollowing/car_following.h"

#include <cstdio>

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_NEAR(actual, expected, tol, msg)                                  \
    do {                                                                         \
        double _a = (actual), _e = (expected);                                   \
        if (std::fabs(_a - _e) <= (tol)) {                                       \
            g_pass++;                                                            \
        } else {                                                                 \
            g_fail++;                                                            \
            std::printf("FAIL %s: expected %g, got %g (%s)\n",                   \
                        __func__, _e, _a, msg);                                  \
        }                                                                        \
    } while (0)

#define EXPECT_TRUE(cond, msg)                                                   \
    do {                                                                         \
        if (cond) { g_pass++; }                                                  \
        else { g_fail++; std::printf("FAIL %s: %s\n", __func__, msg); }          \
    } while (0)

// The page's slider defaults.
static CfParams defaults() { return CfParams{}; }

static void test_model_names_round_trip() {
    for (int i = 0; i <= (int)CfModel::IDM; ++i) {
        CfModel m;
        EXPECT_TRUE(cfModelFromName(cfModelName((CfModel)i), m) && m == (CfModel)i,
                    "name round-trips");
    }
    CfModel m;
    // The chooser labels arrive from the page verbatim; matching must not be
    // case-sensitive since the CLI accepts lowercase too.
    EXPECT_TRUE(cfModelFromName("cf-ovm", m) && m == CfModel::CfOVM, "lowercase CF-OVM");
    EXPECT_TRUE(cfModelFromName("idm", m) && m == CfModel::IDM, "lowercase IDM");
    EXPECT_TRUE(!cfModelFromName("Krauss", m), "unknown model rejected");
}

static void test_optimal_velocity() {
    CfParams p = defaults();          // speedMax 15, timeGap 0.8
    EXPECT_NEAR(cfOptimalVelocity(0.f, p),   0.0,  1e-6, "zero gap -> stopped");
    EXPECT_NEAR(cfOptimalVelocity(-3.f, p),  0.0,  1e-6, "negative gap clamps to 0");
    EXPECT_NEAR(cfOptimalVelocity(8.f, p),  10.0,  1e-5, "8 / 0.8 = 10 m/s");
    EXPECT_NEAR(cfOptimalVelocity(80.f, p), 15.0,  1e-6, "clamps to speed-max");
}

// At the experiment's density every vehicle sits at the same spacing, so the
// gap is L/N - car-size and the equilibrium speed is V() of that.
static void test_uniform_equilibrium_is_a_fixed_point() {
    CfParams p = defaults();
    const float gap = 230.f / 22.f - p.carSize;      // 10.4545 - 5 = 5.4545 m
    const float v   = cfOptimalVelocity(gap, p);     // 5.4545 / 0.8 = 6.818 m/s
    EXPECT_NEAR(v, 6.81818, 1e-4, "equilibrium speed at the paper's density");

    // Every vehicle at that speed and spacing: nothing should accelerate.
    CfInput in{gap, v, v, gap};
    for (CfModel m : {CfModel::Reuschel, CfModel::Pipes, CfModel::OVM,
                      CfModel::CfOVM, CfModel::FVDM, CfModel::ATG}) {
        EXPECT_NEAR(cfStep(m, in, p, 0.1f), v, 1e-3,
                    "uniform flow is a fixed point");
    }
}

static void test_pipes_tracks_relative_speed() {
    CfParams p = defaults();                          // reactionTime 0.7
    CfInput in{6.f, 5.f, 8.f, 6.f};                   // 3 m/s faster ahead
    EXPECT_NEAR(cfAcceleration(CfModel::Pipes, in, p), 3.0 / 0.7, 1e-4,
                "acc = Dv / reaction-time");
    // Pipes ignores the gap entirely — that is why it can never be stable.
    CfInput far{60.f, 5.f, 8.f, 60.f};
    EXPECT_NEAR(cfAcceleration(CfModel::Pipes, far, p),
                cfAcceleration(CfModel::Pipes, in, p), 1e-6, "gap has no effect");
}

static void test_ovm_and_fvdm() {
    CfParams p = defaults();
    CfInput in{6.f, 5.f, 8.f, 6.f};
    double V = 6.0 / 0.8;                             // 7.5 m/s
    EXPECT_NEAR(cfAcceleration(CfModel::OVM, in, p), (V - 5.0) / 0.7, 1e-4,
                "acc = (V(gap) - v) / reaction-time");
    // FVDM is OVM plus a relative-speed term over the fixed 3s alignment time.
    EXPECT_NEAR(cfAcceleration(CfModel::FVDM, in, p),
                (V - 5.0) / 0.7 + 3.0 / CF_SPEED_ALIGN_TIME, 1e-4,
                "FVDM adds Dv / speed-alignment-time");
}

static void test_idm() {
    CfParams p = defaults();
    // Zero relative speed, so s* reduces to s0 + v*T.
    CfInput in{6.f, 5.f, 5.f, 6.f};
    double ss   = CF_IDM_S0 + 5.0 * 0.8;              // 5.0 m
    double want = CF_IDM_A * (1.0 - (ss / 6.0) * (ss / 6.0)
                                  - std::pow(5.0 / 15.0, 4));
    EXPECT_NEAR(cfAcceleration(CfModel::IDM, in, p), want, 1e-4, "IDM acceleration");

    // Closing fast on a nearly-closed gap: the deceleration floor must hold,
    // which is what keeps the 0.1s Euler step from producing a negative speed.
    CfInput panic{0.2f, 10.f, 0.f, 0.2f};
    float acc = cfAcceleration(CfModel::IDM, panic, p);
    EXPECT_NEAR(acc, -3.0 * 10.0, 1e-4, "decel floored at -3v");
    EXPECT_TRUE(cfStep(CfModel::IDM, panic, p, 0.1f) > 0.f, "speed stays positive");
}

static void test_atg_time_gap_is_bounded() {
    CfParams p = defaults();
    // A stopped vehicle would make the raw time gap (gap / v) blow up; the
    // logsumexp mollifier must keep the divisor finite either way.
    CfInput stopped{5.f, 0.f, 0.f, 5.f};
    float acc = cfAcceleration(CfModel::ATG, stopped, p);
    EXPECT_TRUE(std::isfinite(acc), "finite acceleration at standstill");
    EXPECT_TRUE(acc > 0.f, "a stopped car with a gap ahead pulls away");

    EXPECT_NEAR(cfLogSumExp(5.f, 2.f, -0.01f), 2.0, 0.05, "smooth min towards 2");
    EXPECT_NEAR(cfLogSumExp(0.01f, 0.1f, 0.01f), 0.1, 0.05, "smooth max towards 0.1");
}

// Reuschel and CF-OVM set the speed outright, so cfStep must not integrate.
static void test_first_order_models_set_speed_directly() {
    CfParams p = defaults();
    CfInput in{8.f, 2.f, 2.f, 8.f};
    EXPECT_NEAR(cfStep(CfModel::Reuschel, in, p, 0.1f), 10.0, 1e-4,
                "Reuschel jumps straight to V(gap)");
    EXPECT_NEAR(cfAcceleration(CfModel::Reuschel, in, p), 0.0, 1e-6,
                "no acceleration term");

    // CF-OVM shifts the gap by the predecessor's own V() difference; with both
    // gaps equal that correction vanishes and it matches Reuschel.
    EXPECT_NEAR(cfStep(CfModel::CfOVM, in, p, 0.1f), 10.0, 1e-4,
                "equal gaps -> same as Reuschel");
    CfInput opening{8.f, 2.f, 2.f, 12.f};   // predecessor pulling away
    EXPECT_TRUE(cfStep(CfModel::CfOVM, opening, p, 0.1f) < 10.f,
                "an opening gap ahead is anticipated, not chased");
}

static void test_noise_is_off_by_default() {
    CfParams p = defaults();                       // sigma = 0
    CfInput in{6.f, 5.f, 5.f, 6.f};
    EXPECT_NEAR(cfStep(CfModel::IDM, in, p, 0.1f, 99.f),
                cfStep(CfModel::IDM, in, p, 0.1f, 0.f), 1e-9,
                "noise sample ignored when sigma = 0");

    p.sigma = 1.f;
    EXPECT_TRUE(cfStep(CfModel::IDM, in, p, 0.1f, 1.f) !=
                cfStep(CfModel::IDM, in, p, 0.1f, 0.f), "sigma > 0 applies noise");

    // The logistic gate suppresses noise at a standstill so a stopped vehicle
    // is never kicked backwards.
    CfInput still{6.f, 0.f, 0.f, 6.f};
    float kicked = cfStep(CfModel::Reuschel, still, p, 0.1f, -50.f);
    EXPECT_TRUE(kicked > 0.f, "noise gated off near zero speed");
}

// The loop tools/vision/car_following.cpp closes: the model is stepped from its
// own speed state, and a measurement that carries a gain error k (on the ring,
// ring.radius / the robot's actual orbit radius) corrects that state slowly.
//
// Run to convergence and report the settled commanded speed as a fraction of
// the equilibrium V(gap). tau <= 0 selects the unstable shape — state replaced
// by the measurement outright — so the two can be compared directly.
static float settled_speed_ratio(float k, float tauS) {
    CfParams p = defaults();
    const float simDt = 0.025f;              // 0.1s model tick at --time-scale 4
    const float gap   = 5.454545f;           // the paper's density, N * 230/22
    const float veq   = cfOptimalVelocity(gap, p);
    const float alpha = cfSyncAlpha(simDt, tauS);

    float v = veq;                           // start at equilibrium
    for (int i = 0; i < 40000; ++i) {
        float measured = v * k;              // what vision reports for that command
        v += alpha * (measured - v);
        CfInput in{gap, v, v, gap};          // uniform ring: every car alike
        v = std::min(std::max(cfStep(CfModel::FVDM, in, p, simDt), 0.f), p.speedMax);
    }
    return v / veq;
}

// Re-seeding the model from the measurement every step is what made the ring
// tool pin every robot at speed-max: the gain error only has to beat the
// per-step decay (1 - dt/reactionTime), which is 3.6% at the tool's defaults.
// Divergence proper starts at k > 1/(1 - dt/reactionTime) = 1.037, but the
// approach is just as useless: speedMax / V(gap) is 2.2 at this density, and
// re-seeding is already at 2.17 by k = 1.02.
static void test_measurement_reseed_diverges() {
    EXPECT_NEAR(settled_speed_ratio(1.00f, -1.f), 1.0, 1e-3,
                "an exact measurement is a fixed point either way");
    EXPECT_TRUE(settled_speed_ratio(1.02f, -1.f) > 2.0f,
                "re-seeding more than doubles the command on a 2% error");
    EXPECT_TRUE(settled_speed_ratio(1.05f, -1.f) > 2.19f,
                "and pins at speed-max past the 1.037 divergence threshold");
    EXPECT_TRUE(settled_speed_ratio(0.90f, -1.f) < 0.30f,
                "and collapses to a third of V(gap) on a 10% error the other way");
}

// Blending it into the state instead keeps the loop contractive. The settled
// speed stays close to V(gap) across the range the ring geometry can actually
// produce, and stays bounded well away from speed-max even far outside it.
static void test_slow_sync_is_stable_under_measurement_error() {
    // +/-5% covers the residual radius error once the heading controller holds
    // the ring (it settles within ~4% of ring.radius across the speed range).
    EXPECT_TRUE(settled_speed_ratio(1.05f, CF_SYNC_TAU_S) < 1.02f,
                "within 2% of V(gap) at a 5% over-read");
    EXPECT_TRUE(settled_speed_ratio(0.95f, CF_SYNC_TAU_S) > 0.98f,
                "within 2% of V(gap) at a 5% under-read");
    for (float k : {0.9f, 0.95f, 1.05f, 1.1f}) {
        float ratio = settled_speed_ratio(k, CF_SYNC_TAU_S);
        EXPECT_TRUE(ratio > 0.96f && ratio < 1.04f,
                    "within 4% of V(gap) across +/-10% measurement error");
    }
    // Far outside that range the loop must still converge rather than run away:
    // a robot orbiting at half the ring radius reads 2x fast, and the command
    // has to stay well short of the speed-max pin that re-seeding produces.
    EXPECT_TRUE(settled_speed_ratio(2.0f, CF_SYNC_TAU_S) < 1.6f,
                "a 2x over-read stays bounded instead of pinning at speed-max");
    EXPECT_TRUE(settled_speed_ratio(0.5f, CF_SYNC_TAU_S) > 0.8f,
                "and a 2x under-read does not collapse");
    // The correction still has to bite, or a robot that cannot keep up would be
    // driven by a model that never notices: the settled state moves toward the
    // measurement, monotonically in the direction of the error.
    EXPECT_TRUE(settled_speed_ratio(0.9f, CF_SYNC_TAU_S) < 1.f &&
                settled_speed_ratio(1.1f, CF_SYNC_TAU_S) > 1.f,
                "the state tracks the measurement's direction");
}

static void test_sync_alpha() {
    EXPECT_NEAR(cfSyncAlpha(0.025f, 2.0f), 0.025 / 2.025, 1e-6, "EMA from a time constant");
    EXPECT_NEAR(cfSyncAlpha(0.025f, 0.f), 1.0, 1e-6, "tau 0 replaces the state");
    // Frame-rate independent: half the step, half the correction, near enough.
    EXPECT_NEAR(cfSyncAlpha(0.0125f, 2.0f) * 2.f, cfSyncAlpha(0.025f, 2.0f), 1e-4,
                "alpha tracks dt for a fixed tau");
}

int main() {
    test_model_names_round_trip();
    test_optimal_velocity();
    test_uniform_equilibrium_is_a_fixed_point();
    test_pipes_tracks_relative_speed();
    test_ovm_and_fvdm();
    test_idm();
    test_atg_time_gap_is_bounded();
    test_first_order_models_set_speed_directly();
    test_noise_is_off_by_default();
    test_sync_alpha();
    test_measurement_reseed_diverges();
    test_slow_sync_is_stable_under_measurement_error();

    std::printf("test_car_following: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
