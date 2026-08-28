// test_car_following_ring.cpp
// Unit tests for lib/CarFollowing/ring.h — the ring bookkeeping and run-state
// machine behind tools/vision/car_following.cpp. No test framework — plain
// asserts with a pass/fail tally, run via `make test`.
//
// Most of these are regressions for behaviour that only showed up on robots:
// a ring that could not accelerate away from standstill, gaps that doubled on
// a dropped detection, and a scale factor that moved every time a marker
// blinked. They are written against the geometry (N vehicles evenly spaced on
// a ring of radius R) rather than against recorded values, so the arithmetic
// is checked rather than pinned.

#include "../lib/CarFollowing/ring.h"

#include <cstdio>
#include <vector>

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

static constexpr float RADIUS_MM = 300.f;
static float noNoise() { return 0.f; }

// Places `n` vehicles evenly on the ring at ids 0..n-1 and settles the roster.
// `t` is advanced past CfRingConfig::settleS so simPerMm is live.
static void placeEvenly(CfRing& ring, int n, double& t, float offsetDeg = 0.f) {
    for (int frame = 0; frame < 3; ++frame) {
        t += 1.0;
        ring.beginFrame();
        for (int i = 0; i < n; ++i)
            ring.observe(i, offsetDeg + 360.f * (float)i / (float)n, t);
        ring.endFrame(t);
    }
}

// ── Angle helpers ────────────────────────────────────────────────────────────

static void test_angle_helpers() {
    EXPECT_NEAR(cfNormAngleDeg(10.f),    10.f, 1e-4, "small angle unchanged");
    EXPECT_NEAR(cfNormAngleDeg(350.f),  -10.f, 1e-4, "wraps down");
    EXPECT_NEAR(cfNormAngleDeg(-350.f),  10.f, 1e-4, "wraps up");
    EXPECT_NEAR(cfNormAngleDeg(720.f + 5.f), 5.f, 1e-4, "several turns wrap");
    EXPECT_NEAR(cfWrap360(-10.f), 350.f, 1e-4, "negative wraps into [0,360)");
    EXPECT_NEAR(cfWrap360(370.f),  10.f, 1e-4, "over a turn wraps into [0,360)");
}

// ── Ring order and gaps ──────────────────────────────────────────────────────

static void test_order_is_sorted_by_angle() {
    CfRing ring;
    double t = 0.0;
    ring.beginFrame();
    ring.observe(7, 200.f, t);
    ring.observe(2,  10.f, t);
    ring.observe(5, 100.f, t);
    ring.endFrame(t);

    const auto& order = ring.order();
    EXPECT_TRUE(order.size() == 3, "all three on the ring");
    EXPECT_TRUE(order[0] == 2 && order[1] == 5 && order[2] == 7,
                "sorted counter-clockwise by angle, not by id");
}

// N vehicles evenly spaced on a ring of L simulated metres each see a clear
// gap of L/N - carSize, wrap-around included.
static void test_even_spacing_gives_equal_gaps() {
    CfRingConfig cfg;
    cfg.simLengthM = 200.f;
    CfRing ring(cfg);
    CfParams p;
    p.carSize = 5.f;

    double t = 0.0;
    placeEvenly(ring, 4, t);
    ring.step(CfModel::Reuschel, p, 0.1f, RADIUS_MM, noNoise);

    for (int id = 0; id < 4; ++id)
        EXPECT_NEAR(ring.car(id)->gap, 200.f / 4.f - 5.f, 1e-2,
                    "even spacing, equal gaps (wrap included)");
}

static void test_gap_follows_the_direction_of_travel() {
    CfParams p;
    p.carSize = 0.f;

    // Counter-clockwise: vehicle 0 at 0 deg looks ahead to vehicle 1 at 90 deg,
    // a quarter of the ring. Clockwise the same pair is three quarters apart.
    for (float dir : {1.f, -1.f}) {
        CfRingConfig cfg;
        cfg.dirSign    = dir;
        cfg.simLengthM = 400.f;
        CfRing ring(cfg);

        double t = 0.0;
        for (int frame = 0; frame < 3; ++frame) {
            t += 1.0;
            ring.beginFrame();
            ring.observe(0,   0.f, t);
            ring.observe(1,  90.f, t);
            ring.observe(2, 180.f, t);
            ring.observe(3, 270.f, t);
            ring.endFrame(t);
        }
        ring.step(CfModel::Reuschel, p, 0.1f, RADIUS_MM, noNoise);
        EXPECT_NEAR(ring.car(0)->gap, 100.f, 1e-2,
                    dir > 0 ? "ccw: predecessor is the next one round"
                            : "cw: predecessor is the previous one round");
    }
}

static void test_single_vehicle_sees_the_whole_ring() {
    CfRingConfig cfg;
    cfg.simLengthM = 230.f;
    CfRing ring(cfg);
    CfParams p;
    p.carSize = 5.f;

    double t = 0.0;
    placeEvenly(ring, 1, t);
    ring.step(CfModel::Reuschel, p, 0.1f, RADIUS_MM, noNoise);
    EXPECT_NEAR(ring.car(0)->gap, 230.f - 5.f, 1e-2, "alone on the ring");
}

// ── The regression that mattered on hardware ─────────────────────────────────
//
// The model's speed is its own state. Before, it was re-seeded from the vision
// measurement on every tick, so the command never got more than one Euler step
// above what the robot had already done — and from standstill that step is
// milimetres per second, below the tool's own floor for commanding a motor, so
// the ring never moved at all. Here the "robots" never move (the same angles
// are observed every frame) and the models must still accelerate.

static void test_model_accelerates_from_rest_without_vision_feedback() {
    CfRingConfig cfg;
    cfg.simLengthM = 4.f * (230.f / 22.f);   // four vehicles at the paper's density
    CfRing ring(cfg);
    CfParams p;   // page defaults: speedMax 15, carSize 5, timeGap 0.8, rt 0.7

    double t = 0.0;
    placeEvenly(ring, 4, t);

    const float simDt = 0.1f / 4.f;   // --time-scale 4
    float prev = 0.f;
    for (int i = 0; i < 200; ++i) {
        // Deliberately re-observe the *same* angles: the vehicles are not
        // physically moving, so the measured speed stays zero throughout.
        t += 0.1;
        ring.beginFrame();
        for (int id = 0; id < 4; ++id) ring.observe(id, 90.f * (float)id, t);
        ring.endFrame(t);
        ring.step(CfModel::FVDM, p, simDt, RADIUS_MM, noNoise);

        EXPECT_TRUE(ring.car(0)->speed >= prev - 1e-6f, "speed never falls back");
        prev = ring.car(0)->speed;
    }

    EXPECT_NEAR(ring.car(0)->measured, 0.f, 1e-6, "measured speed is still zero");
    // V(s) for the equilibrium gap, which FVDM relaxes onto: gap/timeGap.
    float gap = ring.car(0)->gap;
    EXPECT_NEAR(ring.car(0)->speed, cfOptimalVelocity(gap, p), 0.05,
                "reaches the optimal velocity despite the robots not moving");
    EXPECT_TRUE(ring.car(0)->speed > 1.f,
                "regression: not capped at one Euler step above the measurement");
}

// The same regression against a robot that responds the way a real one does:
// its speed chases the command through a first-order lag. That lag is what
// made the old loop stall — the model read the lagging speed back as its own
// state — so this drives the whole chain (model -> command -> physical speed ->
// new positions -> new gaps) and asks whether the ring actually gets anywhere.
static void test_a_lagging_robot_still_gets_the_ring_moving() {
    const float timeScale   = 4.f;      // --time-scale 4
    const float robotTauS   = 0.25f;    // motor + PID response, real seconds
    const float realDt      = 0.01f;    // control period
    const float modelDtS    = 0.1f;     // model tick, real seconds
    const int   N           = 4;

    CfRingConfig cfg;
    CfRing   ring(cfg);                 // roster-derived length: N * 230/22 m
    CfParams p;                         // page defaults

    double t = 0.0;
    float  angle[N], travelled[N] = {}, vRealMms[N] = {};
    for (int i = 0; i < N; ++i) angle[i] = 360.f * (float)i / (float)N;
    placeEvenly(ring, N, t);

    const float spm = ring.simPerMm(RADIUS_MM);
    EXPECT_TRUE(spm > 0.f, "scale is live");

    double     nextTick   = t + modelDtS;
    float      minGap     = 1e9f;
    const int  steps      = (int)(60.0 / realDt);   // one minute of wall clock
    for (int k = 0; k < steps; ++k) {
        t += realDt;

        for (int i = 0; i < N; ++i) {
            // Command out: simulated m/s -> world mm/s, undoing the density
            // scale and then the time dilation, exactly as the tool does.
            float cmdMms = ring.car(i)->speed / spm / timeScale;
            vRealMms[i] += (cmdMms - vRealMms[i]) * (realDt / robotTauS);

            float dDeg = vRealMms[i] * realDt / RADIUS_MM * (180.f / 3.14159265f);
            angle[i]      = cfWrap360(angle[i] + dDeg);
            travelled[i] += dDeg;
        }

        ring.beginFrame();
        for (int i = 0; i < N; ++i) ring.observe(i, angle[i], t);
        ring.endFrame(t);

        if (t >= nextTick) {
            nextTick += modelDtS;
            ring.step(CfModel::FVDM, p, modelDtS / timeScale, RADIUS_MM, noNoise);
            for (int i = 0; i < N; ++i) minGap = std::min(minGap, ring.car(i)->gap);
        }
    }

    // Equilibrium for an evenly spaced ring is V(s) = gap / timeGap.
    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(ring.car(i)->speed, cfOptimalVelocity(ring.car(i)->gap, p), 0.5,
                    "settles on the optimal velocity for its gap");
        EXPECT_TRUE(travelled[i] > 360.f,
                    "regression: the ring completes a lap instead of stalling");
        EXPECT_NEAR(ring.car(i)->measured, ring.car(i)->speed, 1.0,
                    "and the robots are keeping up with what the model asked for");
    }
    EXPECT_TRUE(minGap > 0.f, "nobody drives into their predecessor");
}

// Measurement is still computed — it is what tells the operator whether the
// robots are keeping up — it just does not drive the model.
static void test_measured_speed_is_reported() {
    CfRingConfig cfg;
    cfg.simLengthM = 200.f;
    CfRing ring(cfg);
    CfParams p;

    double t = 0.0;
    placeEvenly(ring, 4, t);
    ring.step(CfModel::Reuschel, p, 0.1f, RADIUS_MM, noNoise);   // seeds prevAngle

    // Advance every vehicle by 9 deg, i.e. 1/40 of the ring, over one tick.
    t += 0.1;
    ring.beginFrame();
    for (int id = 0; id < 4; ++id) ring.observe(id, 90.f * (float)id + 9.f, t);
    ring.endFrame(t);
    ring.step(CfModel::Reuschel, p, 0.1f, RADIUS_MM, noNoise);

    EXPECT_NEAR(ring.car(0)->measured, (200.f / 40.f) / 0.1f, 1e-2,
                "9 deg of a 200 m ring in 0.1 s = 50 m/s");
}

// Every vehicle must step off the same snapshot: the result cannot depend on
// which id the sort happens to visit first.
static void test_models_step_off_one_snapshot() {
    CfParams p;
    p.sigma = 0.f;

    auto runRing = [&](const std::vector<int>& ids) {
        CfRingConfig cfg;
        cfg.simLengthM = 60.f;
        CfRing ring(cfg);
        double t = 0.0;
        // Uneven spacing, so a vehicle reading a stale vs. fresh predecessor
        // speed would diverge.
        const float angles[4] = {0.f, 30.f, 170.f, 300.f};
        for (int frame = 0; frame < 3; ++frame) {
            t += 1.0;
            ring.beginFrame();
            for (int i = 0; i < 4; ++i) ring.observe(ids[i], angles[i], t);
            ring.endFrame(t);
        }
        for (int i = 0; i < 25; ++i) ring.step(CfModel::FVDM, p, 0.025f, RADIUS_MM, noNoise);
        std::vector<float> out;
        for (int i = 0; i < 4; ++i) out.push_back(ring.car(ids[i])->speed);
        return out;
    };

    auto a = runRing({0, 1, 2, 3});
    auto b = runRing({3, 2, 1, 0});   // same positions, ids assigned backwards
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(a[i], b[i], 1e-5, "result is independent of id order");
}

// ── Dropouts ─────────────────────────────────────────────────────────────────
//
// A vehicle missing for a frame keeps its place, so its follower still brakes
// for it. Dropping it instead handed the follower the gap to the vehicle
// beyond — roughly twice the real one — and the follower accelerated into a
// robot that was still physically there.

static void test_a_dropout_keeps_its_place_on_the_ring() {
    CfRingConfig cfg;
    cfg.simLengthM = 400.f;
    cfg.holdS      = 1.0;
    CfRing ring(cfg);
    CfParams p;
    p.carSize = 0.f;

    double t = 0.0;
    placeEvenly(ring, 4, t);
    ring.step(CfModel::Reuschel, p, 0.1f, RADIUS_MM, noNoise);
    EXPECT_NEAR(ring.car(0)->gap, 100.f, 1e-2, "quarter-ring gap while all are seen");

    // Vehicle 1 — vehicle 0's predecessor — is not detected this frame.
    t += 0.1;
    ring.beginFrame();
    ring.observe(0,   0.f, t);
    ring.observe(2, 180.f, t);
    ring.observe(3, 270.f, t);
    ring.endFrame(t);
    ring.step(CfModel::Reuschel, p, 0.1f, RADIUS_MM, noNoise);

    EXPECT_TRUE(ring.order().size() == 4, "the dropout is still on the ring");
    EXPECT_NEAR(ring.car(0)->gap, 100.f, 1e-2,
                "regression: the gap does not double when a marker blinks");
    EXPECT_TRUE(ring.visibleCount() == 3, "but it is not counted as visible");
}

static void test_a_long_dropout_leaves_the_ring() {
    CfRingConfig cfg;
    cfg.holdS = 1.0;
    CfRing ring(cfg);

    double t = 0.0;
    placeEvenly(ring, 3, t);
    EXPECT_TRUE(ring.order().size() == 3, "three on the ring");

    t += 1.5;                       // past holdS with nothing observed for id 2
    ring.beginFrame();
    ring.observe(0,   0.f, t);
    ring.observe(1, 120.f, t);
    ring.endFrame(t);
    EXPECT_TRUE(ring.order().size() == 2, "expired after holdS");
    EXPECT_TRUE(!ring.has(2), "and is gone from the ring");
}

// A vehicle that is not detected is not being commanded either, so its model
// is held rather than integrated on toward a speed nothing is delivering.
static void test_an_unseen_vehicle_holds_its_model_speed() {
    CfRingConfig cfg;
    cfg.simLengthM = 100.f;
    CfRing ring(cfg);
    CfParams p;

    double t = 0.0;
    placeEvenly(ring, 3, t);
    for (int i = 0; i < 10; ++i) {
        t += 0.1;
        ring.beginFrame();
        for (int id = 0; id < 3; ++id) ring.observe(id, 120.f * (float)id, t);
        ring.endFrame(t);
        ring.step(CfModel::FVDM, p, 0.025f, RADIUS_MM, noNoise);
    }
    float held = ring.car(2)->speed;
    EXPECT_TRUE(held > 0.f, "vehicle 2 was moving");

    for (int i = 0; i < 5; ++i) {
        t += 0.1;
        ring.beginFrame();
        ring.observe(0,   0.f, t);
        ring.observe(1, 120.f, t);
        ring.endFrame(t);
        ring.step(CfModel::FVDM, p, 0.025f, RADIUS_MM, noNoise);
    }
    EXPECT_NEAR(ring.car(2)->speed, held, 1e-6, "unseen vehicle's model is held");
}

// ── Roster and scale ─────────────────────────────────────────────────────────

static void test_roster_settles_before_it_rescales_the_ring() {
    CfRingConfig cfg;
    cfg.settleS = 1.0;
    CfRing ring(cfg);

    double t = 0.0;
    placeEvenly(ring, 4, t);
    EXPECT_TRUE(ring.rosterCount() == 4, "roster settled at four");
    EXPECT_TRUE(ring.takeRosterChange(), "settling is announced");
    float scale = ring.simPerMm(RADIUS_MM);
    EXPECT_TRUE(scale > 0.f, "scale is live");

    // One marker blinks out for a frame. The virtual ring must not resize.
    t += 0.02;
    ring.beginFrame();
    for (int id = 0; id < 3; ++id) ring.observe(id, 90.f * (float)id, t);
    ring.endFrame(t);
    EXPECT_TRUE(ring.rosterCount() == 4, "regression: a blink does not resize the ring");
    EXPECT_NEAR(ring.simPerMm(RADIUS_MM), scale, 1e-9, "so the scale factor holds");

    // A robot genuinely removed and left off does eventually commit — but only
    // after both waits: holdS for it to leave the ring at all, then settleS for
    // the new count to hold.
    for (int frame = 0; frame < 2; ++frame) {   // ~1.2 s: past holdS, inside settleS
        t += 0.6;
        ring.beginFrame();
        for (int id = 0; id < 3; ++id) ring.observe(id, 90.f * (float)id, t);
        ring.endFrame(t);
    }
    EXPECT_TRUE(ring.rosterCount() == 4, "the new count has not settled yet");

    for (int frame = 0; frame < 2; ++frame) {
        t += 0.6;
        ring.beginFrame();
        for (int id = 0; id < 3; ++id) ring.observe(id, 90.f * (float)id, t);
        ring.endFrame(t);
    }
    EXPECT_TRUE(ring.rosterCount() == 3, "a real change does commit after settleS");
    EXPECT_TRUE(ring.takeRosterChange(), "and is announced once");
    EXPECT_TRUE(!ring.takeRosterChange(), "only once");
}

static void test_pinned_count_and_length_override_the_roster() {
    CfRingConfig cfg;
    cfg.pinnedCount = 6;
    CfRing ring(cfg);

    double t = 0.0;
    placeEvenly(ring, 2, t);
    EXPECT_TRUE(ring.rosterCount() == 6, "--count pins the vehicle count");
    EXPECT_NEAR(ring.simLengthM(), 6.f * (230.f / 22.f), 1e-3,
                "and therefore the ring length");

    CfRingConfig cfg2;
    cfg2.simLengthM = 77.f;
    CfRing ring2(cfg2);
    double t2 = 0.0;
    placeEvenly(ring2, 3, t2);
    EXPECT_NEAR(ring2.simLengthM(), 77.f, 1e-3, "--sim-length pins the length outright");
}

static void test_scale_is_not_ready_before_the_roster_settles() {
    CfRing ring;
    double t = 0.0;
    EXPECT_TRUE(!ring.scaleReady(RADIUS_MM), "nothing detected yet");

    t += 0.1;
    ring.beginFrame();
    ring.observe(0, 0.f, t);
    ring.endFrame(t);
    EXPECT_TRUE(!ring.scaleReady(RADIUS_MM), "one frame is not a settled roster");

    placeEvenly(ring, 1, t);
    EXPECT_TRUE(ring.scaleReady(RADIUS_MM), "settled");
    EXPECT_TRUE(!ring.scaleReady(0.f), "and still needs a radius to divide by");
}

static void test_rest_returns_the_ring_to_standstill() {
    CfRing ring;
    CfParams p;
    double t = 0.0;
    placeEvenly(ring, 3, t);
    for (int i = 0; i < 20; ++i) ring.step(CfModel::FVDM, p, 0.025f, RADIUS_MM, noNoise);
    EXPECT_TRUE(ring.car(0)->speed > 0.f, "moving");

    ring.rest();
    for (int id = 0; id < 3; ++id) {
        EXPECT_NEAR(ring.car(id)->speed,    0.f, 1e-6, "rest zeroes the model speed");
        EXPECT_NEAR(ring.car(id)->measured, 0.f, 1e-6, "and the measurement");
    }
    EXPECT_TRUE(ring.order().size() == 3, "but the roster survives a re-arm");
}

// ── Alignment ────────────────────────────────────────────────────────────────

static void test_align_targets_are_already_correct_when_evenly_spaced() {
    CfRingConfig cfg;
    CfRing ring(cfg);
    double t = 0.0;
    // Rotated by 37 deg, so the test cannot pass by accident if the rotation
    // that minimizes travel were hard-coded to zero.
    placeEvenly(ring, 4, t, 37.f);
    for (int id = 0; id < 4; ++id)
        EXPECT_NEAR(ring.car(id)->alignErrorDeg, 0.f, 1e-2,
                    "already on an evenly spaced slot needs no travel");
    EXPECT_TRUE(ring.allAligned(1.f), "and allAligned agrees");
}

// The rotation computeAlignTargets() picks is the one that minimizes total
// travel, not whichever vehicle the angle sort happens to put first. Checked
// against a concrete alternative (pin slot 0 to vehicle 0) rather than a
// pinned value, same instinct as the rest of this file.
static void test_align_targets_minimize_total_travel() {
    CfRingConfig cfg;
    CfRing ring(cfg);
    double t = 0.0;
    const float angles[4] = {0.f, 5.f, 10.f, 15.f};   // bunched together
    for (int frame = 0; frame < 3; ++frame) {
        t += 1.0;
        ring.beginFrame();
        for (int i = 0; i < 4; ++i) ring.observe(i, angles[i], t);
        ring.endFrame(t);
    }

    double sumSq = 0.0;
    for (int id = 0; id < 4; ++id) {
        double e = ring.car(id)->alignErrorDeg;
        sumSq += e * e;
    }

    double naiveSumSq = 0.0;
    for (int i = 0; i < 4; ++i) {
        float target = cfWrap360(angles[0] + 90.f * (float)i);
        double e = cfNormAngleDeg(target - angles[i]);
        naiveSumSq += e * e;
    }
    EXPECT_TRUE(sumSq <= naiveSumSq + 1e-6,
                "circular-mean rotation travels no more than pinning slot 0 to vehicle 0");
}

// A vehicle that blinks out keeps its place on the ring (CfRingConfig::holdS)
// and so still shapes the target rotation, but allAligned() only requires the
// vehicles currently visible to be in their slots — the same asymmetry
// step()/gap already has for dropouts.
//
// The shove has to stay inside vehicle 0's own slot (< half of 360/N) or it
// would cross another vehicle's angle and the angle sort would hand out
// slots in a different order entirely — order_ is always the current angular
// sort, not an identity pinned to who held which slot a moment ago.
static void test_all_aligned_skips_a_vehicle_that_is_not_currently_visible() {
    CfRingConfig cfg;
    CfRing ring(cfg);
    double t = 0.0;
    const int N = 8;   // slots are 45 deg apart
    placeEvenly(ring, N, t);
    EXPECT_TRUE(ring.allAligned(1.f), "starts evenly spaced");

    // Vehicle 0 gets shoved 20 deg out of its slot, then blinks out.
    t += 0.1;
    ring.beginFrame();
    ring.observe(0, 20.f, t);
    for (int i = 1; i < N; ++i) ring.observe(i, 360.f * (float)i / (float)N, t);
    ring.endFrame(t);
    EXPECT_TRUE(std::fabs(ring.car(0)->alignErrorDeg) > 10.f,
                "the shoved vehicle is well out of its slot");

    t += 0.1;
    ring.beginFrame();
    for (int i = 1; i < N; ++i) ring.observe(i, 360.f * (float)i / (float)N, t);   // 0 not observed
    ring.endFrame(t);

    EXPECT_TRUE(ring.allAligned(5.f),
                "the rest are close enough even though the missing vehicle is not");
}

// ── Run state ────────────────────────────────────────────────────────────────

static void test_run_state_starts_in_setup() {
    CfRunState run;
    EXPECT_TRUE(run.phase() == CfPhase::Setup, "comes up in setup");
    EXPECT_TRUE(!run.running(), "not running");
    for (int i = 0; i < 5; ++i)
        EXPECT_TRUE(run.update(true) == CfRunEvent::None && !run.running(),
                    "regression: a ready rig does not start itself");
}

static void test_a_start_cue_is_latched_until_ready() {
    CfRunState run;
    run.requestStart("page");
    EXPECT_TRUE(run.pending(), "cue latched");
    EXPECT_TRUE(run.update(false) == CfRunEvent::Waiting, "says what it is waiting for");
    EXPECT_TRUE(run.update(false) == CfRunEvent::None, "but only once");
    EXPECT_TRUE(!run.running(), "still not running");

    EXPECT_TRUE(run.update(true) == CfRunEvent::Started, "starts as soon as it can");
    EXPECT_TRUE(run.running(), "running");
    EXPECT_TRUE(run.update(true) == CfRunEvent::None, "and stays there quietly");
}

static void test_stop_returns_to_setup_once() {
    CfRunState run;
    run.requestStart("key");
    run.update(true);
    EXPECT_TRUE(run.running(), "running");

    run.requestStop("key");
    EXPECT_TRUE(run.update(true) == CfRunEvent::Stopped, "one stop event");
    EXPECT_TRUE(!run.running() && run.phase() == CfPhase::Setup, "back in setup");
    EXPECT_TRUE(run.update(true) == CfRunEvent::None,
                "regression: a stopped run does not restart on the next frame");
}

static void test_toggle_cancels_a_pending_cue() {
    CfRunState run;
    run.toggle("key");
    EXPECT_TRUE(run.pending(), "first toggle cues a start");
    run.update(false);            // not ready yet — still only cued
    run.toggle("key");
    EXPECT_TRUE(!run.pending(), "second toggle withdraws it");
    EXPECT_TRUE(run.update(true) == CfRunEvent::None && !run.running(),
                "so a rig that becomes ready stays put");
}

static void test_stop_while_never_started_is_quiet() {
    CfRunState run;
    run.requestStop("stdin");
    EXPECT_TRUE(run.update(true) == CfRunEvent::None,
                "stopping something that was never running says nothing");
}

// ── Run state: align cue ────────────────────────────────────────────────────

static void test_an_align_cue_is_latched_until_ready() {
    CfRunState run;
    run.requestAlign("page");
    EXPECT_TRUE(run.phase() == CfPhase::Setup, "not aligning yet");
    EXPECT_TRUE(run.update(false, false) == CfRunEvent::AlignWaiting,
                "says what it is waiting for");
    EXPECT_TRUE(run.update(false, false) == CfRunEvent::None, "but only once");

    EXPECT_TRUE(run.update(true, false) == CfRunEvent::AlignStarted,
                "starts as soon as it can");
    EXPECT_TRUE(run.aligning(), "aligning");
    EXPECT_TRUE(run.update(true, false) == CfRunEvent::None,
                "stays there quietly until the caller reports it aligned");

    EXPECT_TRUE(run.update(true, true) == CfRunEvent::Aligned,
                "finishes on its own once the rig reports every robot in its slot");
    EXPECT_TRUE(!run.aligning() && run.phase() == CfPhase::Setup, "back in setup");
}

static void test_align_cue_rests_a_running_ring_first() {
    CfRunState run;
    run.requestStart("key");
    run.update(true, false);
    EXPECT_TRUE(run.running(), "running");

    run.requestAlign("page");
    EXPECT_TRUE(run.update(true, false) == CfRunEvent::Stopped,
                "align interrupts a run with a stop first, like requestStop");
    EXPECT_TRUE(run.update(true, false) == CfRunEvent::AlignStarted,
                "then starts aligning on the next frame");
}

static void test_a_start_cue_cancels_a_pending_align() {
    CfRunState run;
    run.requestAlign("page");
    EXPECT_TRUE(run.update(false, false) == CfRunEvent::AlignWaiting, "align cued");

    run.requestStart("key");
    EXPECT_TRUE(run.update(true, false) == CfRunEvent::Started,
                "regression: a run cue supersedes a pending align rather than queuing behind it");
}

static void test_stop_cancels_alignment_in_progress() {
    CfRunState run;
    run.requestAlign("page");
    run.update(true, false);
    EXPECT_TRUE(run.aligning(), "aligning");

    run.requestStop("key");
    EXPECT_TRUE(run.update(true, false) == CfRunEvent::Stopped,
                "stop interrupts an alignment in progress");
    EXPECT_TRUE(!run.aligning() && run.phase() == CfPhase::Setup, "back in setup");
}

int main() {
    test_angle_helpers();
    test_order_is_sorted_by_angle();
    test_even_spacing_gives_equal_gaps();
    test_gap_follows_the_direction_of_travel();
    test_single_vehicle_sees_the_whole_ring();
    test_model_accelerates_from_rest_without_vision_feedback();
    test_a_lagging_robot_still_gets_the_ring_moving();
    test_measured_speed_is_reported();
    test_models_step_off_one_snapshot();
    test_a_dropout_keeps_its_place_on_the_ring();
    test_a_long_dropout_leaves_the_ring();
    test_an_unseen_vehicle_holds_its_model_speed();
    test_roster_settles_before_it_rescales_the_ring();
    test_pinned_count_and_length_override_the_roster();
    test_scale_is_not_ready_before_the_roster_settles();
    test_rest_returns_the_ring_to_standstill();
    test_align_targets_are_already_correct_when_evenly_spaced();
    test_align_targets_minimize_total_travel();
    test_all_aligned_skips_a_vehicle_that_is_not_currently_visible();
    test_run_state_starts_in_setup();
    test_a_start_cue_is_latched_until_ready();
    test_stop_returns_to_setup_once();
    test_toggle_cancels_a_pending_cue();
    test_stop_while_never_started_is_quiet();
    test_an_align_cue_is_latched_until_ready();
    test_align_cue_rests_a_running_ring_first();
    test_a_start_cue_cancels_a_pending_align();
    test_stop_cancels_alignment_in_progress();

    std::printf("test_car_following_ring: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
