// test_goto_controller.cpp
// Unit tests for lib/SwarmControl/goto_controller.h and avoidance.h.
//
// The primary job of this file is an EQUIVALENCE CHECK. The controller was
// extracted from control loops that were copy-pasted inline across
// drag_drop_demo / shape_demo / circle_demo, each tuned against real hardware.
// A refactor that quietly changes a gain, a clamp order, or the brake ramp
// would not fail to compile — it would just make the robots behave differently
// on the floor, where it is expensive to notice.
//
// So `legacy*()` below are verbatim transcriptions of the original inline math
// (see git history for tools/vision/drag_drop_demo.cpp:479-503 and
// shape_demo.cpp:626-647), and the tests sweep both implementations over a
// randomized input grid asserting byte-identical int8 motor output.
//
// If you intend to change the controller's behavior, these tests SHOULD fail —
// update the legacy transcriptions in the same commit, and say so in the
// message, so the behavior change is a deliberate reviewable act.

#include "../lib/SwarmControl/goto_controller.h"
#include "../lib/SwarmControl/avoidance.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <unordered_map>

using namespace swarmctl;

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_EQ(actual, expected, msg) \
    do { \
        auto _a = (actual); auto _e = (expected); \
        if (_a == _e) { g_pass++; } \
        else { g_fail++; std::printf("FAIL %s: expected %lld, got %lld (%s)\n", \
               __func__, (long long)_e, (long long)_a, msg); } \
    } while (0)

#define EXPECT_TRUE(cond, msg) EXPECT_EQ((bool)(cond), true, msg)

#define EXPECT_NEAR(actual, expected, tol, msg) \
    do { \
        double _a = (actual), _e = (expected); \
        if (std::fabs(_a - _e) <= (tol)) { g_pass++; } \
        else { g_fail++; std::printf("FAIL %s: expected %f±%f, got %f (%s)\n", \
               __func__, _e, (double)(tol), _a, msg); } \
    } while (0)

// ─── Legacy transcriptions ────────────────────────────────────────────────────
// Verbatim from the pre-extraction demos. Do not "clean these up" — their value
// is that they are an independent copy of the original arithmetic.

static float legacyNormAngle(float a) {
    while (a >  180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}
static float legacyClampf(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

// drag_drop_demo.cpp — with brake term.
struct LegacyOut { int8_t l, r; };

static LegacyOut legacyDragDrop(float dx, float dy, float yaw, float maxSpd,
                                float dt, float* prevAngleErr, bool hasPrev,
                                float K_DIST, float K_ANGLE, float K_YAW_D,
                                float MAX_TURN, float ARRIVAL_MM)
{
    float dist      = sqrtf(dx*dx + dy*dy);
    float tgtAngle  = atan2f(dy, dx) * 180.f / (float)M_PI;
    float angleErr  = legacyNormAngle(tgtAngle - yaw);
    float headingN  = legacyClampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
    float headingSc = 1.f - headingN * headingN;
    float brakeSc   = legacyClampf((dist - ARRIVAL_MM) / ARRIVAL_MM, 0.f, 1.f);

    float dAngleErr = 0.f;
    if (hasPrev)
        dAngleErr = legacyClampf(legacyNormAngle(angleErr - *prevAngleErr) / dt,
                                 -300.f, 300.f);
    *prevAngleErr = angleErr;

    float forward = legacyClampf(K_DIST * dist, 0.f, maxSpd) * headingSc * brakeSc;
    float turn    = legacyClampf(K_ANGLE * angleErr + K_YAW_D * dAngleErr,
                                 -MAX_TURN, MAX_TURN);
    return { (int8_t)legacyClampf(forward + turn, -100, 100),
             (int8_t)legacyClampf(forward - turn, -100, 100) };
}

// shape_demo.cpp — no brake term (looping path has no terminal stop).
static LegacyOut legacyShape(float dx, float dy, float yaw, float maxSpd,
                             float dt, float* prevAngleErr, bool hasPrev,
                             float K_DIST, float K_ANGLE, float K_YAW_D,
                             float MAX_TURN)
{
    float dist      = sqrtf(dx*dx + dy*dy);
    float tgtAngle  = atan2f(dy, dx) * 180.f / (float)M_PI;
    float angleErr  = legacyNormAngle(tgtAngle - yaw);
    float headingN  = legacyClampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
    float headingSc = 1.f - headingN * headingN;

    float dAngleErr = 0.f;
    if (hasPrev)
        dAngleErr = legacyClampf(legacyNormAngle(angleErr - *prevAngleErr) / dt,
                                 -300.f, 300.f);
    *prevAngleErr = angleErr;

    float forward = legacyClampf(K_DIST * dist, 0.f, maxSpd) * headingSc;
    float turn    = legacyClampf(K_ANGLE * angleErr + K_YAW_D * dAngleErr,
                                 -MAX_TURN, MAX_TURN);
    return { (int8_t)legacyClampf(forward + turn, -100, 100),
             (int8_t)legacyClampf(forward - turn, -100, 100) };
}

// ─── Equivalence sweeps ───────────────────────────────────────────────────────

static void test_equivalence_drag_drop() {
    GotoParams p;
    p.kDist = 0.40f; p.kAngle = 0.50f; p.kYawD = 0.08f;
    p.maxTurn = 16.0f; p.arrivalMm = 20.0f; p.brake = true;

    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> pos(-2000.f, 2000.f);
    std::uniform_real_distribution<float> ang(-360.f, 360.f);
    std::uniform_real_distribution<float> dtd(0.01f, 0.20f);

    int mismatches = 0;
    // Multi-step trajectories, so the stateful D term is exercised across
    // frames rather than only from a cold start.
    for (int trial = 0; trial < 2000; trial++) {
        GotoState st;
        float legacyPrev = 0.f; bool legacyHasPrev = false;
        float yaw = ang(rng);

        for (int step = 0; step < 6; step++) {
            float dx = pos(rng), dy = pos(rng), dt = dtd(rng);
            float maxSpd = 60.0f;

            MotorCmd got = computeGoto(dx, dy, yaw, maxSpd, p, st, dt);
            LegacyOut want = legacyDragDrop(dx, dy, yaw, maxSpd, dt,
                                            &legacyPrev, legacyHasPrev,
                                            0.40f, 0.50f, 0.08f, 16.0f, 20.0f);
            legacyHasPrev = true;
            if (got.left != want.l || got.right != want.r) mismatches++;
            yaw = ang(rng);   // simulate the robot turning between frames
        }
    }
    EXPECT_EQ(mismatches, 0, "drag_drop_demo control law diverged after extraction");
}

static void test_equivalence_shape() {
    GotoParams p;
    p.kDist = 0.40f; p.kAngle = 0.50f; p.kYawD = 0.08f;
    p.maxTurn = 16.0f; p.arrivalMm = 40.0f; p.brake = false;

    std::mt19937 rng(67890);
    std::uniform_real_distribution<float> pos(-2000.f, 2000.f);
    std::uniform_real_distribution<float> ang(-360.f, 360.f);
    std::uniform_real_distribution<float> dtd(0.01f, 0.20f);

    int mismatches = 0;
    for (int trial = 0; trial < 2000; trial++) {
        GotoState st;
        float legacyPrev = 0.f; bool legacyHasPrev = false;
        float yaw = ang(rng);

        for (int step = 0; step < 6; step++) {
            float dx = pos(rng), dy = pos(rng), dt = dtd(rng);
            float maxSpd = 51.7f;

            MotorCmd got = computeGoto(dx, dy, yaw, maxSpd, p, st, dt);
            LegacyOut want = legacyShape(dx, dy, yaw, maxSpd, dt,
                                         &legacyPrev, legacyHasPrev,
                                         0.40f, 0.50f, 0.08f, 16.0f);
            legacyHasPrev = true;
            if (got.left != want.l || got.right != want.r) mismatches++;
            yaw = ang(rng);
        }
    }
    EXPECT_EQ(mismatches, 0, "shape_demo control law diverged after extraction");
}

// ─── Behavioral properties ────────────────────────────────────────────────────

static void test_heading_scale() {
    GotoParams p; p.brake = false; p.kYawD = 0.f; p.kAngle = 0.f;
    GotoState st;
    // Aimed straight at a far goal → forward should saturate at maxSpd.
    MotorCmd aimed = computeGoto(1000.f, 0.f, 0.f, 50.f, p, st, 0.05f);
    EXPECT_EQ((int)aimed.left, 50, "aimed at goal should run at maxSpd");

    // 90° off → headingSc == 0 → no forward component at all.
    GotoState st2;
    MotorCmd sideways = computeGoto(0.f, 1000.f, 0.f, 50.f, p, st2, 0.05f);
    EXPECT_EQ((int)sideways.left, 0, "90 deg off should produce zero forward");
}

static void test_brake_ramp() {
    GotoParams p; p.brake = true; p.arrivalMm = 20.f; p.kAngle = 0.f; p.kYawD = 0.f;
    GotoState st;
    // Exactly at the arrival radius the brake term zeroes the output.
    MotorCmd at = computeGoto(20.f, 0.f, 0.f, 60.f, p, st, 0.05f);
    EXPECT_EQ((int)at.left, 0, "brake should zero output at arrival radius");

    // One full ramp-span out, the brake is fully released.
    GotoState st2;
    MotorCmd out = computeGoto(40.f, 0.f, 0.f, 60.f, p, st2, 0.05f);
    EXPECT_EQ((int)out.left, (int)(0.40f * 40.f), "brake fully released one span out");
}

static void test_turn_slew_limit() {
    GotoParams p; p.brake = false; p.kAngle = 1.0f; p.kYawD = 0.f;
    p.maxTurn = 100.f; p.maxTurnRate = 120.f;   // 120 units/s
    GotoState st;
    // Demand a hard left from rest with dt = 0.05 → at most 6 units of turn.
    MotorCmd cmd = computeGoto(0.f, 1000.f, 0.f, 50.f, p, st, 0.05f);
    // headingSc is 0 at 90°, so forward is 0 and left == +turn.
    EXPECT_NEAR(st.lastTurn, 6.0, 1e-4, "turn slew should cap at maxTurnRate*dt");
    EXPECT_EQ((int)cmd.left, 6, "slewed turn drives the motors");
}

static void test_norm_angle_seam() {
    EXPECT_NEAR(normAngle(190.f),  -170.f, 1e-4, "wrap above +180");
    EXPECT_NEAR(normAngle(-190.f),  170.f, 1e-4, "wrap below -180");
    EXPECT_NEAR(normAngle(180.f),   180.f, 1e-4, "+180 is its own representative");
}

// ─── Yaw smoother ─────────────────────────────────────────────────────────────

static void test_yaw_smoother_seam() {
    YawSmoother sm(0.50f);
    // First sample is adopted as-is.
    EXPECT_NEAR(sm.update(0, 179.f, 0.05f), 179.f, 1e-4, "first sample seeds filter");
    // Crossing the ±180 seam must take the SHORT way (179 → -179 is +2°,
    // not -358°). A naive EMA on raw degrees would swing toward zero here.
    float y = sm.update(0, -179.f, 0.05f);
    EXPECT_TRUE(y > 179.f || y < -179.f, "seam crossing must not swing the long way");
}

static void test_yaw_smoother_rate_independence() {
    // The same elapsed time at different loop rates must land in the same
    // place (to within discretization) — that is the whole point of specifying
    // the filter as a time-constant rather than a fixed per-frame alpha.
    YawSmoother fast(0.50f), slow(0.50f);
    for (int i = 0; i < 100; i++) fast.update(0, 90.f, 0.005f);  // 200 Hz, 0.5 s
    for (int i = 0; i < 10;  i++) slow.update(0, 90.f, 0.05f);   // 20 Hz,  0.5 s
    float f = fast.update(0, 90.f, 0.005f);
    float s = slow.update(0, 90.f, 0.05f);
    EXPECT_NEAR(f, s, 2.0, "filter must hold its time-constant across loop rates");
}

// ─── Avoidance ────────────────────────────────────────────────────────────────

struct TestPose { float x, y, yaw; };

static const float DT = 0.02f;   // 50Hz, matches the demos' control rate

static void test_avoidance_far_apart_is_noop() {
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f, 0.f, 0.f}}, {1, {1000.f, 0.f, 180.f}},
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    EXPECT_TRUE(!av[0].dodging && !av[1].dodging, "robots beyond safeMm must not dodge");
    EXPECT_TRUE(av[0].minDist > 1e5f, "no pair recorded beyond safeMm");
}

static void test_avoidance_face_to_face_yields_by_id() {
    // Head-on at 150mm (inside dangerMm 200), both moving → higher ID dodges,
    // lower ID is marked priority and keeps full speed.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {150.f, 0.f, 180.f}},
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    EXPECT_TRUE(av[1].dodging,  "higher ID dodges in a face-to-face");
    EXPECT_TRUE(!av[0].dodging, "lower ID holds its line");
    EXPECT_TRUE(av[0].priority, "lower ID is the protected robot");
    EXPECT_NEAR(av[0].minDist, 150.0, 1e-3, "minDist recorded for both");
}

static void test_priority_robot_is_never_slowed() {
    // The whole point of the rework: the protected robot runs at its full cap
    // even with a neighbour well inside the danger disc. Distance is chosen
    // inside dangerMm (500) but outside emergencyMm (160) — the normal regime. The old proximity ramp
    // scaled BOTH robots, so an approaching pair stalled nose to nose.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {300.f, 0.f, 180.f}},
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    auto act = applyAvoidance(av[0], 60.f, eng.params());
    EXPECT_TRUE(!act.hardStop, "priority robot must not be stopped");
    EXPECT_NEAR(act.maxSpd, 60.0, 1e-3, "priority robot must keep its full speed cap");
}

static void test_avoidance_moving_yields_to_stationary() {
    // Robot 1 (higher ID) is parked; robot 0 drives at it. ID priority must NOT
    // apply — a parked robot cannot dodge, and since the mover is never slowed,
    // nominating the parked one would drive robot 0 straight into it.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {150.f, 0.f, 0.f}},
    };
    auto av = eng.update(poses, [](int id){ return id == 0; }, DT);
    EXPECT_TRUE(av[0].dodging,  "the moving robot dodges around a parked one");
    EXPECT_TRUE(!av[1].dodging, "a parked robot is never asked to dodge");
}

static void test_avoidance_arc_is_unit_length() {
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {150.f, 0.f, 180.f}},
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    const auto& a = av[1];
    EXPECT_NEAR(std::sqrt(a.arcDx*a.arcDx + a.arcDy*a.arcDy), 1.0, 1e-4,
                "arc direction must be normalized");
}

static void test_avoidance_ignores_non_closing_pair() {
    // Both inside the danger disc but driving apart — no dodge should trigger.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 180.f}},   // heading -X, away from robot 1
        {1, {150.f, 0.f, 0.f}},     // heading +X, away from robot 0
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    EXPECT_TRUE(!av[0].dodging && !av[1].dodging, "diverging robots need no dodge");
}

static void test_dodge_latches_across_frames() {
    // Once triggered the dodge must hold even after the dodger has turned far
    // enough that the closing test would no longer fire. A stateless version
    // cancels here, the robot turns back, and it chatters in and out.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {150.f, 0.f, 180.f}},
    };
    auto first = eng.update(poses, [](int){ return true; }, DT);
    EXPECT_TRUE(first[1].dodging, "dodge triggers on the closing frame");
    const float ax = first[1].arcDx, ay = first[1].arcDy;

    // Robot 1 has swung 90° away — no longer closing, still in range.
    poses[1].yaw = 90.f;
    auto later = eng.update(poses, [](int){ return true; }, DT);
    EXPECT_TRUE(later[1].dodging, "latched dodge must survive the dodger turning away");
    EXPECT_NEAR(later[1].arcDx, ax, 1e-4, "latched dodge must not switch sides");
    EXPECT_NEAR(later[1].arcDy, ay, 1e-4, "latched dodge must not switch sides");
}

static void test_dodge_releases_past_safe_distance() {
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {150.f, 0.f, 180.f}},
    };
    EXPECT_TRUE(eng.update(poses, [](int){ return true; }, DT)[1].dodging, "dodge triggers");

    // Still inside safeMm (1000): the latch holds even though the pair is now
    // well outside the 500mm trigger — that gap is the hysteresis.
    poses[1].x = 700.f;
    EXPECT_TRUE(eng.update(poses, [](int){ return true; }, DT)[1].dodging,
                "latch holds between dangerMm and safeMm");

    poses[1].x = 1100.f;
    EXPECT_TRUE(!eng.update(poses, [](int){ return true; }, DT)[1].dodging,
                "latch releases past safeMm");
}

static void test_latch_renominates_when_the_dodger_parks() {
    // A latched dodge is only valid while its dodger can still act on it. If the
    // nominated dodger reaches its goal and parks, the latch must be dropped and
    // the dodge handed to whoever is still moving — otherwise the priority
    // robot, which is never slowed, drives into a robot that will never move.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {300.f, 0.f, 180.f}},
    };
    bool hiMoving = true;
    auto isMoving = [&](int id) { return id == 0 ? true : hiMoving; };

    auto av = eng.update(poses, isMoving, DT);
    EXPECT_TRUE(av[1].dodging, "higher ID dodges while both are moving");

    // Robot 1 arrives and parks, still inside the danger disc. The stale latch
    // is dropped on this frame and re-nomination happens on the next one — the
    // release path cannot also re-trigger in the same pass. One frame at 50Hz is
    // ~5mm of travel, versus the ~50mm of closing that timeout-only recovery
    // allowed, so the gap is left in rather than restructuring the scan.
    hiMoving = false;
    av = eng.update(poses, isMoving, DT);
    EXPECT_TRUE(!av[1].dodging, "a parked robot must not stay nominated as dodger");

    av = eng.update(poses, isMoving, DT);
    EXPECT_TRUE(av[0].dodging,  "the dodge passes to the robot still moving");
    EXPECT_TRUE(!av[1].dodging, "the parked robot stays un-nominated");
}

static void test_latch_survives_the_priority_robot_parking() {
    // The mirror case must NOT re-nominate: when the protected robot parks, the
    // dodger is still the one moving and still the one that has to get around.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {300.f, 0.f, 180.f}},
    };
    bool loMoving = true;
    auto isMoving = [&](int id) { return id == 0 ? loMoving : true; };

    EXPECT_TRUE(eng.update(poses, isMoving, DT)[1].dodging, "higher ID dodges");
    loMoving = false;
    auto av = eng.update(poses, isMoving, DT);
    EXPECT_TRUE(av[1].dodging,  "dodger keeps dodging around a robot that parked");
    EXPECT_TRUE(!av[0].dodging, "a parked robot is never asked to dodge");
}

static void test_dodge_times_out() {
    // A dodge around a permanently parked obstacle must eventually give up,
    // otherwise a robot whose goal sits behind that obstacle orbits forever.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {150.f, 0.f, 0.f}},
    };
    auto isMoving = [](int id){ return id == 0; };
    EXPECT_TRUE(eng.update(poses, isMoving, DT)[0].dodging, "dodge triggers");

    bool stillDodging = true;
    for (int i = 0; i < 1000 && stillDodging; i++)   // 20s at 50Hz
        stillDodging = eng.update(poses, isMoving, DT)[0].dodging;
    EXPECT_TRUE(!stillDodging, "a latched dodge must time out");
}

static void test_emergency_stops_the_charging_robot_not_the_dodger() {
    // Head-on inside emergencyMm: the dodge has failed. The DODGER must keep
    // maneuvering — it is the only one with a plan and therefore the only one
    // that can open the gap — while the charging robot is halted. Stopping the
    // dodger instead lets the other drive through it; stopping both is an
    // inescapable freeze, since then nothing can change the distance.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {140.f, 0.f, 180.f}},   // inside emergencyMm (160), both closing
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    auto dodger   = applyAvoidance(av[1], 60.f, eng.params());
    auto charging = applyAvoidance(av[0], 60.f, eng.params());
    EXPECT_TRUE(charging.hardStop, "the charging robot is stopped");
    EXPECT_NEAR(charging.maxSpd, 0.0, 1e-4, "hardStop must not leave a live speed cap");
    EXPECT_TRUE(!dodger.hardStop, "the dodger must keep maneuvering — it is the escape");
    EXPECT_TRUE(dodger.maxSpd > 0.f, "a stopped dodger cannot open the gap");
}

static void test_emergency_spares_a_fleeing_robot() {
    // Overtake inside emergencyMm: robot 1 is in front driving away, robot 0
    // behind is the dodger. Stopping robot 1 would delete the separation it is
    // creating, so a robot that is already leaving is never halted.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},     // behind, closing
        {1, {140.f, 0.f, 0.f}},     // in front, same direction — fleeing
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    EXPECT_TRUE(av[0].dodging, "the robot behind is the one that must dodge");
    auto fleeing = applyAvoidance(av[1], 60.f, eng.params());
    EXPECT_TRUE(!fleeing.hardStop, "a robot driving away is never emergency-stopped");
}

static void test_overtake_nominates_the_robot_behind() {
    // Both moving, but only the one behind is closing. ID priority must not
    // nominate the robot in front (higher ID) — it would decline to dodge, and
    // nobody would.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {0.f,   0.f, 0.f}},
        {1, {300.f, 0.f, 0.f}},
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    EXPECT_TRUE(av[0].dodging,  "the closing robot behind dodges");
    EXPECT_TRUE(!av[1].dodging, "the robot in front is not asked to dodge");
}

static void test_apply_avoidance_clear_robot() {
    AvoidState clear;   // minDist stays 1e6, not dodging
    auto a = applyAvoidance(clear, 60.f);
    EXPECT_TRUE(!a.hardStop, "clear robot does not stop");
    EXPECT_NEAR(a.maxSpd, 60.0, 1e-3, "clear robot keeps full speed");
    EXPECT_NEAR(a.arcBlend, 0.0, 1e-4, "clear robot gets no detour");
}

static void test_opposing_threats_do_not_cancel() {
    // Robot 2 is pinned between robots 0 and 1, one on each side. The two
    // detours point in opposite directions and sum to ~zero; without the
    // degeneracy guard that normalizes into garbage, and there is no longer a
    // speed ramp behind it to catch the mistake.
    AvoidanceEngine eng;
    std::unordered_map<int, TestPose> poses = {
        {0, {-150.f, 0.f, 0.f}},
        {1, { 150.f, 0.f, 180.f}},
        {2, {   0.f, 0.f, 0.f}},
    };
    auto av = eng.update(poses, [](int){ return true; }, DT);
    if (av[2].dodging) {
        const float len = std::sqrt(av[2].arcDx*av[2].arcDx + av[2].arcDy*av[2].arcDy);
        EXPECT_NEAR(len, 1.0, 1e-3, "a dodging robot must always get a usable detour");
    } else {
        g_pass++;   // not nominated in this geometry; nothing to assert
    }
}

static void test_latches_do_not_leak() {
    // Pairs that stop existing must not accumulate in the engine's latch map
    // over a long run.
    AvoidanceEngine eng;
    for (int i = 0; i < 200; i++) {
        std::unordered_map<int, TestPose> poses = {
            {i * 2,     {0.f,   0.f, 0.f}},
            {i * 2 + 1, {150.f, 0.f, 180.f}},
        };
        eng.update(poses, [](int){ return true; }, DT);
    }
    // Only the final pair may remain; an empty pose set must clear everything.
    std::unordered_map<int, TestPose> none;
    auto av = eng.update(none, [](int){ return true; }, DT);
    EXPECT_TRUE(av.empty(), "no state for robots that are not present");
}

// ─── Runner ───────────────────────────────────────────────────────────────────

int main() {
    test_equivalence_drag_drop();
    test_equivalence_shape();
    test_heading_scale();
    test_brake_ramp();
    test_turn_slew_limit();
    test_norm_angle_seam();
    test_yaw_smoother_seam();
    test_yaw_smoother_rate_independence();
    test_avoidance_far_apart_is_noop();
    test_avoidance_face_to_face_yields_by_id();
    test_avoidance_moving_yields_to_stationary();
    test_avoidance_arc_is_unit_length();
    test_priority_robot_is_never_slowed();
    test_avoidance_ignores_non_closing_pair();
    test_dodge_latches_across_frames();
    test_dodge_releases_past_safe_distance();
    test_latch_renominates_when_the_dodger_parks();
    test_latch_survives_the_priority_robot_parking();
    test_dodge_times_out();
    test_emergency_stops_the_charging_robot_not_the_dodger();
    test_emergency_spares_a_fleeing_robot();
    test_overtake_nominates_the_robot_behind();
    test_apply_avoidance_clear_robot();
    test_opposing_threats_do_not_cancel();
    test_latches_do_not_leak();

    std::printf("\ntest_goto_controller: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
