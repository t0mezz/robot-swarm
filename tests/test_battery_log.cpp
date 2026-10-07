// test_battery_log.cpp
// Unit tests for lib/BatteryLog/battery_log.h — the camera-free bookkeeping
// behind tools/vision/battery_log.cpp. No test framework — plain asserts with
// a pass/fail tally, run via `make test`.
//
// The speed tests drive synthetic poses round a ring of known radius, so the
// expected mm/s is geometry (omega * R), not a recorded value.

#include "../lib/BatteryLog/battery_log.h"

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

static constexpr float  R_MM  = 300.f;
static constexpr float  CX    = 500.f, CY = 400.f;
static constexpr double FPS   = 100.0;

// Feeds `seconds` of frames of a robot orbiting at `mms` in direction dirSign
// (+1 = ccw), starting at angle a0 (radians), with an optional radial wobble.
static double orbit(BlRowAccumulator& acc, double t, double seconds, float mms,
                    float dirSign, double& a, float wobbleMm = 0.f) {
    const double dt = 1.0 / FPS;
    const double omega = dirSign * mms / R_MM;
    int n = (int)(seconds * FPS + 0.5);
    for (int i = 0; i < n; ++i) {
        t += dt;
        a += omega * dt;
        float r = R_MM + ((i % 2) ? wobbleMm : -wobbleMm);
        acc.frame(true, CX + r * (float)std::cos(a), CY + r * (float)std::sin(a),
                  CX, CY, R_MM, dirSign, 5.f, t);
    }
    return t;
}

// ── Schedule ─────────────────────────────────────────────────────────────────

static void test_schedule_cycles() {
    BlSchedule s{120.0, 12.0};
    EXPECT_TRUE(s.next(BlPhase::Orbit, 119.0) == BlPhase::Orbit, "orbit before the rest is due");
    EXPECT_TRUE(s.next(BlPhase::Orbit, 120.0) == BlPhase::Rest,  "rest once due");
    EXPECT_TRUE(s.next(BlPhase::Rest, 11.0)   == BlPhase::Rest,  "rest holds for restForS");
    EXPECT_TRUE(s.next(BlPhase::Rest, 12.0)   == BlPhase::Seek,  "a rest ends in a reseek, not straight back to orbit");
    EXPECT_TRUE(s.next(BlPhase::Seek, 500.0)  == BlPhase::Seek,  "seek -> orbit is geometric, not scheduled");
}

static void test_schedule_disabled() {
    BlSchedule s{0.0, 12.0};
    EXPECT_TRUE(s.next(BlPhase::Orbit, 1e6) == BlPhase::Orbit, "restEveryS <= 0 never rests");
}

// ── Row accumulator ──────────────────────────────────────────────────────────

static void test_speed_ccw_and_cw() {
    for (float dir : {1.f, -1.f}) {
        BlRowAccumulator acc; acc.begin(0.0);
        double a = 0.3;
        double t = orbit(acc, 0.0, 1.0, 80.f, dir, a);
        BlRow r = acc.take(t);
        EXPECT_NEAR(r.speedMms, 80.0, 0.5, dir > 0 ? "ccw orbit reads its speed" : "cw orbit reads its speed");
        EXPECT_NEAR(r.radialErrMm, 0.0, 0.01, "on the ring");
        EXPECT_NEAR(r.absTurn, 5.0, 1e-4, "mean |turn|");
        EXPECT_TRUE(r.nFrames == 100, "one frame per tick");
    }
}

static void test_speed_is_signed_along_travel() {
    // Configured ccw but actually going cw: the speed must come out negative,
    // not as a magnitude — a robot orbiting the wrong way is a fault to see.
    BlRowAccumulator acc2; acc2.begin(0.0);
    double a = 0.0, t = 0.0;
    const double dt = 1.0 / FPS;
    for (int i = 0; i < 100; ++i) {
        t += dt; a -= 60.0 / R_MM * dt;
        acc2.frame(true, CX + R_MM * (float)std::cos(a), CY + R_MM * (float)std::sin(a),
                   CX, CY, R_MM, +1.f, 0.f, t);
    }
    EXPECT_NEAR(acc2.take(t).speedMms, -60.0, 0.5, "travel against dirSign reads negative");
}

static void test_radial_wobble_does_not_inflate_speed() {
    // hypot() of per-frame deltas would read a 2mm-per-frame zig-zag as
    // ~400 mm/s on top of the orbit; the tangential projection ignores it.
    BlRowAccumulator acc; acc.begin(0.0);
    double a = 0.0;
    double t = orbit(acc, 0.0, 1.0, 50.f, 1.f, a, 2.f);
    EXPECT_NEAR(acc.take(t).speedMms, 50.0, 1.0, "radial noise is not speed");
}

static void test_dropout_breaks_the_chain() {
    // Half a second of orbit, a 0.3s dropout (during which the robot keeps
    // moving), then half a second more. Differencing across the gap would
    // still be right here, but counting the gap's time without its travel —
    // or its travel without its time — would not.
    BlRowAccumulator acc; acc.begin(0.0);
    double a = 0.0;
    double t = orbit(acc, 0.0, 0.5, 100.f, 1.f, a);
    for (int i = 0; i < 30; ++i) {
        t += 1.0 / FPS;
        a += 100.0 / R_MM / FPS;
        acc.frame(false, 0, 0, CX, CY, R_MM, 1.f, 0.f, t);
    }
    t = orbit(acc, t, 0.5, 100.f, 1.f, a);
    BlRow r = acc.take(t);
    EXPECT_NEAR(r.speedMms, 100.0, 0.5, "dropout neither inflates nor deflates");
    EXPECT_TRUE(r.nFrames == 100, "missing frames are not counted");
    EXPECT_TRUE(r.nPairs == 98, "each side of the gap is its own chain (49 + 49)");
}

static void test_empty_row_is_nan() {
    BlRowAccumulator acc; acc.begin(0.0);
    acc.frame(false, 0, 0, CX, CY, R_MM, 1.f, 0.f, 0.5);
    BlRow r = acc.take(1.0);
    EXPECT_TRUE(std::isnan(r.speedMms), "no pose -> no speed");
    EXPECT_TRUE(!r.visible(), "no pose -> not visible");
    EXPECT_NEAR(r.dtS, 1.0, 1e-9, "row still spans its time");
}

static void test_take_carries_the_last_pose() {
    // Two back-to-back rows: the step straddling the boundary is counted in
    // the second, so the rows' pair counts add up to frames - 1.
    BlRowAccumulator acc; acc.begin(0.0);
    double a = 0.0;
    double t = orbit(acc, 0.0, 1.0, 70.f, 1.f, a);
    BlRow r1 = acc.take(t);
    t = orbit(acc, t, 1.0, 70.f, 1.f, a);
    BlRow r2 = acc.take(t);
    EXPECT_TRUE(r1.nPairs == 99 && r2.nPairs == 100, "boundary step lands in the second row");
    EXPECT_NEAR(r2.speedMms, 70.0, 0.5, "second row speed");
    EXPECT_NEAR(r2.dtS, 1.0, 1e-6, "second row starts where the first ended");
}

// ── Stop criteria ────────────────────────────────────────────────────────────

static BlRow row(float mms) { BlRow r; r.dtS = 1.0; r.nFrames = 100; r.nPairs = 99; r.speedMms = mms; return r; }

static void test_low_voltage_is_debounced() {
    BlStop s;   // 4000 mV, 6 s
    s.battery(0.0, 3990, true, false);
    s.battery(5.0, 3990, true, false);
    EXPECT_TRUE(!s.stopped(), "not yet held for lowHoldS");
    s.battery(5.5, 4040, true, false);   // recovers — sag under a load step
    s.battery(9.0, 3990, true, false);
    s.battery(14.0, 3990, true, false);
    EXPECT_TRUE(!s.stopped(), "a recovery restarts the hold");
    s.battery(15.0, 3990, true, false);
    EXPECT_TRUE(s.reason() == BlStopReason::LowVoltage, "held below for lowHoldS");
}

static void test_invalid_battery_is_ignored() {
    BlStop s;
    s.battery(0.0, 0, false, false);
    s.battery(100.0, 0, false, false);
    EXPECT_TRUE(!s.stopped(), "0 with BAT_VALID unset is 'never measured', not empty");
}

static void test_low_flag_stops() {
    BlStop s;
    s.battery(0.0, 4800, true, true);
    EXPECT_TRUE(s.reason() == BlStopReason::LowBatteryFlag, "flag stops at once");
}

static void test_stall_needs_a_baseline_and_a_hold() {
    BlStopConfig c; c.baselineS = 10.0; c.stallS = 5.0;
    BlStop s(c);
    double t = 0.0, orbitS = 0.0;
    for (int i = 0; i < 10; ++i) { t += 1; orbitS += 1; s.row(t, BlPhase::Orbit, row(100.f), orbitS); }
    EXPECT_NEAR(s.baseline(), 100.0, 1e-4, "baseline is the first baselineS of orbit");

    for (int i = 0; i < 4; ++i) { t += 1; orbitS += 1; s.row(t, BlPhase::Orbit, row(10.f), orbitS); }
    EXPECT_TRUE(!s.stopped(), "4 s slow is not a stall");
    t += 1; orbitS += 1; s.row(t, BlPhase::Orbit, row(90.f), orbitS);
    for (int i = 0; i < 4; ++i) { t += 1; orbitS += 1; s.row(t, BlPhase::Orbit, row(10.f), orbitS); }
    EXPECT_TRUE(!s.stopped(), "a normal row resets the hold");
    t += 1; orbitS += 1; s.row(t, BlPhase::Orbit, row(10.f), orbitS);
    EXPECT_TRUE(s.reason() == BlStopReason::Stall, "5 s under 20% of baseline");
}

static void test_rest_rows_do_not_stall() {
    BlStopConfig c; c.baselineS = 2.0; c.stallS = 3.0;
    BlStop s(c);
    double t = 0.0, orbitS = 0.0;
    for (int i = 0; i < 2; ++i) { t += 1; orbitS += 1; s.row(t, BlPhase::Orbit, row(100.f), orbitS); }
    for (int i = 0; i < 20; ++i) { t += 1; s.row(t, BlPhase::Rest, row(0.f), orbitS); }
    EXPECT_TRUE(!s.stopped(), "a rest is supposed to be slow");
}

static void test_seek_radial_floor() {
    // The case from the 2026-10-06 run: 21.2 mm inside the ring, P alone = 6.4.
    float v = blSeekRadial(-21.2f, 0.30f, 50.f, 18.f, 20.f);
    EXPECT_TRUE(v == 18.f, "stalled-band error is driven outward at the floor");
    EXPECT_TRUE(blSeekRadial(21.2f, 0.30f, 50.f, 18.f, 20.f) == -18.f, "and inward on the other side");
    EXPECT_TRUE(blSeekRadial(-19.f, 0.30f, 50.f, 18.f, 20.f) < 18.f, "inside the band the floor is off");
    EXPECT_TRUE(blSeekRadial(-300.f, 0.30f, 50.f, 18.f, 20.f) == 50.f, "far away is capped at max");
    EXPECT_NEAR(blSeekRadial(-100.f, 0.30f, 50.f, 18.f, 20.f), 30.0, 1e-4, "P term untouched above the floor");
}

static void test_seek_timeout_and_lost() {
    BlStop s;
    s.tick(BlPhase::Seek, 299.0, 0.0);
    EXPECT_TRUE(!s.stopped(), "seek within timeout");
    s.tick(BlPhase::Seek, 300.0, 0.0);
    EXPECT_TRUE(s.reason() == BlStopReason::SeekTimeout, "seek timed out");

    BlStop s2;
    s2.tick(BlPhase::Setup, 0.0, 1e6);
    EXPECT_TRUE(!s2.stopped(), "unseen during setup is just waiting");
    s2.tick(BlPhase::Orbit, 0.0, 60.0);
    EXPECT_TRUE(s2.reason() == BlStopReason::RobotLost, "gone for lostS");
}

static void test_first_reason_wins() {
    BlStop s;
    s.battery(0.0, 4800, true, true);
    s.tick(BlPhase::Seek, 1e6, 1e6);
    EXPECT_TRUE(s.reason() == BlStopReason::LowBatteryFlag, "reason is not overwritten");
}

// ── Rolling median ───────────────────────────────────────────────────────────

static void test_rolling_median() {
    std::vector<double> t = {0, 1, 2, 3, 4, 5};
    std::vector<float>  v = {10, 12, 100, 11, NAN, 13};
    auto m = blRollingMedian(t, v, 3.0);   // window (t-3, t]
    EXPECT_NEAR(m[0], 10.0, 1e-4, "single sample");
    EXPECT_NEAR(m[1], 11.0, 1e-4, "even count averages the middle two");
    EXPECT_NEAR(m[2], 12.0, 1e-4, "an outlier does not move the median");
    EXPECT_NEAR(m[3], 12.0, 1e-4, "t=0 has left the window");
    EXPECT_NEAR(m[4], 55.5, 1e-4, "NaN is skipped (100, 11)");
    EXPECT_NEAR(m[5], 12.0, 1e-4, "window {11, NaN, 13}");
}

static void test_rolling_median_all_nan() {
    auto m = blRollingMedian({0, 1}, {NAN, NAN}, 5.0);
    EXPECT_TRUE(std::isnan(m[0]) && std::isnan(m[1]), "nothing to take a median of");
}

int main() {
    test_seek_radial_floor();
    test_schedule_cycles();
    test_schedule_disabled();
    test_speed_ccw_and_cw();
    test_speed_is_signed_along_travel();
    test_radial_wobble_does_not_inflate_speed();
    test_dropout_breaks_the_chain();
    test_empty_row_is_nan();
    test_take_carries_the_last_pose();
    test_low_voltage_is_debounced();
    test_invalid_battery_is_ignored();
    test_low_flag_stops();
    test_stall_needs_a_baseline_and_a_hold();
    test_rest_rows_do_not_stall();
    test_seek_timeout_and_lost();
    test_first_reason_wins();
    test_rolling_median();
    test_rolling_median_all_nan();

    std::printf("test_battery_log: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
