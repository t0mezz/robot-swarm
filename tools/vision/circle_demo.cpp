// circle_demo.cpp — Circle orbit formation controller
// Usage: ./circle_demo [--serial SN] [--ip IP] [--calibrate]
//                      [--radius MM] [--min-gap MM] [--orbit-speed DEG_S]
//                      [--speed PCT] [--log-perf] [--log-score]
// Controls: left-click = set circle centre, s = stop, c = calibrate,
//           0-9 = select robot (again to deselect),
//           +/- = radius ±25mm  (or robot speed ±10% when robot selected),
//           t = toggle orbit tracking,  [ / ] = orbit speed ±5 deg/s,
//           q/Esc = quit

#include "aruco_tracker.h"
#include "SwarmClient.h"
#include "DebugHud.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <algorithm>

// ── Controller tunables ───────────────────────────────────────────────────────

static constexpr float DEFAULT_RADIUS_MM      = 300.0f;
static constexpr float DEFAULT_MIN_GAP_MM     = 0.0f;   // min arc-chord distance between neighbours
static constexpr float DEFAULT_ORBIT_SPEED    = 30.0f;  // deg/s, default orbit speed in tracking mode

static constexpr float K_DIST    = 0.40f;
static constexpr float K_ANGLE   = 0.45f;  // heading P-gain — corrects residual error only; orbit feedforward carries the steady turn
static constexpr float K_YAW_D   = 0.15f;  // heading D-gain: damps transient overshoot
static constexpr float K_FF_YAW  = 1.00f;   // orbit-mode yaw feedforward, turn-units per deg/s of required yaw rate — tune empirically (see orbit control below)
static constexpr float K_RAD     = 0.30f;   // radial correction gain (mm/s per mm of error)
static constexpr float MAX_SPEED = 100.0f;  // full motor range for high-speed testing
static constexpr float MAX_TURN  = 20.0f;
// Slew-rate limit on the commanded turn differential, applied at the final
// output stage. This is the actuator-side counterpart to the D-term
// windowing below: whatever measurement noise survives upstream, this caps
// d(turn)/dt directly — by construction the controller score's `jerk` term
// can never exceed MAX_TURN_RATE. 200 units/s = full -40..+40 swing in 0.4s;
// tune up if corrections feel sluggish, down if still buzzing.
static constexpr float MAX_TURN_RATE = 120.0f;  // turn-units/s
static constexpr float ARRIVAL_MM       = 40.0f;   // stop when within this distance of slot
static constexpr float SEND_INTERVAL_S  = 0.01f; // Send intervall in s

// Yaw EMA: smooths the raw per-frame ArUco corner-angle before it reaches any
// controller. At 115fps (~8.7ms/frame) raw yaw jitters by ~1deg frame-to-frame
// from corner-detection noise alone; left unfiltered, that noise is what the
// D-term (divided by a ~10ms dt) blows up into ~30 turn-units of pure buzz per
// frame. alpha=0.08 gives ~12-frame memory (~105ms), matching the smoothing
// the old alpha=0.25 gave at 30fps (~4-frame memory, ~133ms).
static constexpr float YAW_ALPHA = 0.08f;

// D-term rate window: angleErr/yaw rate-of-change is estimated via
// sample-and-hold over this period rather than per-frame, since
// differencing consecutive ~8.7ms frames just measures measurement noise —
// SEND_INTERVAL_S is the actuation rate, so there's no benefit to a faster
// derivative anyway.
static constexpr float D_TERM_WINDOW_S = SEND_INTERVAL_S;

// Controller score: three EMA-filtered terms isolating the PD controller's
// failure modes (see ControlScore below). SCORE_ALPHA sets the averaging
// time-constant (~1/SCORE_ALPHA frames, so ~33s at 30fps for 0.03).
static constexpr float SCORE_ALPHA = 0.03f;
static constexpr float W_BIAS     = 1.0f;   // weight on persistent heading error  (deg)
static constexpr float W_OSC      = 1.0f;   // weight on oscillation RMS           (deg)
static constexpr float W_JERK     = 0.05f;  // weight on output-jerk RMS           (turn-units/s)
static constexpr float W_POS_BIAS = 0.1f;   // weight on persistent radial (off-track) error (per mm)
static constexpr float W_POS_OSC  = 0.1f;   // weight on radial oscillation RMS              (per mm)

// ── Swarm hub ────────────────────────────────────────────────────────────────

static constexpr int MAX_ROBOTS = SC_MAX_ROBOTS;

static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

// ── Helpers ───────────────────────────────────────────────────────────────────

static float normAngle(float a) {
    while (a >  180) a -= 360;
    while (a < -180) a += 360;
    return a;
}
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

// Sample-and-hold rate-of-change estimator for an angle (deg/s). Holds the
// previous rate until D_TERM_WINDOW_S has elapsed since the last sample, so
// the result reflects change over a real time window instead of frame-to-frame
// measurement noise (see D_TERM_WINDOW_S above).
struct RateEstimator {
    bool   init      = false;
    float  baseAngle = 0.f;
    float  rate      = 0.f;
    std::chrono::steady_clock::time_point baseTime;
};

static float updateRate(RateEstimator& r, float angle,
                         std::chrono::steady_clock::time_point now,
                         float windowS) {
    if (!r.init) {
        r.baseAngle = angle;
        r.baseTime  = now;
        r.init      = true;
        return 0.f;
    }
    float elapsed = std::chrono::duration<float>(now - r.baseTime).count();
    if (elapsed >= windowS) {
        r.rate      = normAngle(angle - r.baseAngle) / elapsed;
        r.baseAngle = angle;
        r.baseTime  = now;
    }
    return r.rate;
}

// ── Controller performance score ─────────────────────────────────────────────
//
// Four failure modes of the PD controller, each isolated by a different filter
// on the (angleErr, turn, radialErr) signal triple:
//
//   1. General error    — persistent offset from the desired heading (gain too
//                          low, feedforward miscalibrated, ...). Captured as a
//                          slow EMA of angleErr itself: a controller that's just
//                          lagging settles to a nonzero mean error.
//   2. Oscillating error — oversteering / ringing. Captured as the RMS of the
//                          high-frequency residual (angleErr minus its slow EMA)
//                          — overshoot makes angleErr swing around the mean even
//                          when the mean itself is ~0.
//   3. Snappy output     — too little smoothing. Captured as the RMS of
//                          d(turn)/dt: a controller riding raw, noisy yaw turns
//                          measurement noise directly into rapid motor-command
//                          changes.
//   4. Off-track position — the heading can be perfectly tracked (angleErr ~ 0)
//                          while the robot itself sits off the circle, e.g.
//                          orbiting at the wrong radius. Captured the same way
//                          as (1)/(2) but on radialErr = dist(robot, centre) -
//                          circle.radius (mm): a slow EMA for a persistent
//                          radius offset, and the RMS of its residual for
//                          radial "breathing" oscillation.
//
// All four are reported in native units plus a single weighted composite
// (lower = better) — a natural objective for an outer optimizer (e.g. a
// Gaussian-process search over K_ANGLE / K_YAW_D / K_FF_YAW / K_RAD).
struct ControlScore {
    bool  init       = false;
    float biasEma    = 0.f;   // slow EMA of angleErr              (deg)
    float oscVar     = 0.f;   // EMA of (angleErr - biasEma)^2     (deg^2)
    float jerkVar    = 0.f;   // EMA of (d turn/dt)^2              ((turn-units/s)^2)
    float posBiasEma = 0.f;   // slow EMA of radialErr             (mm)
    float posVar     = 0.f;   // EMA of (radialErr - posBiasEma)^2 (mm^2)
    float prevTurn   = 0.f;
};

static void updateScore(ControlScore& sc, float angleErr, float turn, float radialErr, float dt) {
    if (!sc.init) {
        sc.biasEma    = angleErr;
        sc.posBiasEma = radialErr;
        sc.prevTurn   = turn;
        sc.init       = true;
        return;
    }
    sc.biasEma += SCORE_ALPHA * (angleErr - sc.biasEma);
    float osc   = angleErr - sc.biasEma;
    sc.oscVar  += SCORE_ALPHA * (osc * osc - sc.oscVar);
    float jerk  = (turn - sc.prevTurn) / dt;
    sc.jerkVar += SCORE_ALPHA * (jerk * jerk - sc.jerkVar);
    sc.prevTurn = turn;

    sc.posBiasEma += SCORE_ALPHA * (radialErr - sc.posBiasEma);
    float posOsc   = radialErr - sc.posBiasEma;
    sc.posVar      += SCORE_ALPHA * (posOsc * posOsc - sc.posVar);
}

static float totalScore(const ControlScore& sc) {
    return W_BIAS     * fabsf(sc.biasEma)
         + W_OSC      * sqrtf(std::max(sc.oscVar,  0.f))
         + W_JERK     * sqrtf(std::max(sc.jerkVar, 0.f))
         + W_POS_BIAS * fabsf(sc.posBiasEma)
         + W_POS_OSC  * sqrtf(std::max(sc.posVar,  0.f));
}

// ── Calibration ───────────────────────────────────────────────────────────────

static cv::Mat  g_H;
static bool     g_hasH = false;
static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";
static const char* CIRCLE_FILE     = "/tmp/circle_demo.yml";

static cv::Point2f pixelToWorld(cv::Point2f px) {
    if (!g_hasH) return px;
    std::vector<cv::Point2f> src = {px}, dst;
    cv::perspectiveTransform(src, dst, g_H);
    return dst[0];
}
static cv::Point2f worldToPixel(cv::Point2f w) {
    if (!g_hasH) return w;
    cv::Mat Hinv = g_H.inv();
    std::vector<cv::Point2f> src = {w}, dst;
    cv::perspectiveTransform(src, dst, Hinv);
    return dst[0];
}

struct CalibState { std::vector<cv::Point2f> pixPts; bool done = false; };
static void onCalibMouse(int event, int x, int y, int, void* ud) {
    auto* s = (CalibState*)ud;
    if (event == cv::EVENT_LBUTTONDOWN && s->pixPts.size() < 4) {
        s->pixPts.push_back({(float)x, (float)y});
        printf("  corner %d: (%d, %d)\n", (int)s->pixPts.size(), x, y);
        if (s->pixPts.size() == 4) s->done = true;
    }
}

static bool runCalibration(ArucoTracker& tracker) {
    printf("\nCalibration: click 4 arena corners (TL TR BR BL)\n");
    // Wait until the capture thread delivers at least one valid frame.
    {
        int attempts = 0;
        while (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (++attempts > 100) {
                fprintf(stderr, "[calib] Timed out waiting for first frame.\n");
                return false;
            }
        }
    }
    cv::Mat frame = tracker.debugFrame().clone();

    cv::namedWindow("Calibration", cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow("Calibration", frame.cols, frame.rows);
    CalibState cs;
    cv::setMouseCallback("Calibration", onCalibMouse, &cs);

    while (!cs.done) {
        cv::Mat disp = frame.clone();
        for (auto& pt : cs.pixPts) cv::circle(disp, pt, 8, {0, 0, 255}, -1);
        tracker.drawText(disp,
            "Click corners TL TR BR BL  " + std::to_string(cs.pixPts.size()) + "/4",
            {10, 40}, 20, {0, 255, 0});
        cv::imshow("Calibration", disp);
        if (cv::waitKey(30) == 27) { cv::destroyWindow("Calibration"); return false; }
    }
    cv::destroyWindow("Calibration");

    printf("Arena width height mm (e.g. 800 600): ");
    float W = 0, H = 0;
    if (scanf("%f %f", &W, &H) != 2 || W <= 0 || H <= 0) { printf("Invalid.\n"); return false; }

    std::vector<cv::Point2f> worldPts = {{0,0},{W,0},{W,H},{0,H}};
    g_H    = cv::findHomography(cs.pixPts, worldPts);
    g_hasH = !g_H.empty();
    if (g_hasH) {
        tracker.setHomography(cs.pixPts, worldPts);
        tracker.saveHomography(HOMOGRAPHY_FILE);
        printf("Saved to %s\n", HOMOGRAPHY_FILE);
    }
    return g_hasH;
}

// ── Circle formation ──────────────────────────────────────────────────────────
//
// Slot assignment:
//   N robots → N evenly-spaced slots around the circle.
//   Each frame we solve the assignment that minimises total angular travel
//   (greedy nearest-slot, which is optimal for ≤~8 robots in practice).
//   Robots drive to their slot; once all are within ARRIVAL_MM the formation
//   is "locked" and each robot just holds its slot.
//
// Minimum-gap enforcement:
//   After slot assignment we check every adjacent pair.  If their current
//   arc-chord distance < minGapMm we nudge their target angles apart.

struct CircleState {
    cv::Point2f centre;         // world coords (mm) or pixels
    float       radius;         // mm or pixels, matching coord space
    float       minGapMm;
    bool        centreSet = false;

    // Orbit tracking
    bool        tracking   = false;
    float       orbitSpeed = DEFAULT_ORBIT_SPEED;   // deg/s (negative = CW)
};

static void saveCircle(const CircleState& c) {
    cv::FileStorage fs(CIRCLE_FILE, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { fprintf(stderr, "[circle] Could not save to %s\n", CIRCLE_FILE); return; }
    fs << "cx"          << c.centre.x
       << "cy"          << c.centre.y
       << "centre_set"  << (int)c.centreSet
       << "radius"      << c.radius
       << "min_gap"     << c.minGapMm
       << "orbit_spd"   << c.orbitSpeed;
    printf("[circle] Saved: centre=(%.0f,%.0f) radius=%.0fmm gap=%.0fmm\n",
           c.centre.x, c.centre.y, c.radius, c.minGapMm);
}

static bool loadCircle(CircleState& c) {
    cv::FileStorage fs(CIRCLE_FILE, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    // Read into temporaries and only apply valid values, so stale/partial
    // files (missing keys read back as 0) don't overwrite good defaults.
    float cx = 0, cy = 0, r = 0, gap = -1.f, spd = 0.f;
    int   centreSet = 0;
    fs["cx"]         >> cx;
    fs["cy"]         >> cy;
    fs["centre_set"] >> centreSet;
    fs["radius"]     >> r;
    fs["min_gap"]    >> gap;
    fs["orbit_spd"]  >> spd;
    if (centreSet) { c.centre = {cx, cy}; c.centreSet = true; }
    if (r   >  0.f) c.radius     = r;
    if (gap >= 0.f) c.minGapMm   = gap;
    if (spd != 0.f) c.orbitSpeed = spd;
    printf("[circle] Loaded: centre=(%.0f,%.0f) radius=%.0fmm gap=%.0fmm orbit=%.0fdeg/s\n",
           c.centre.x, c.centre.y, c.radius, c.minGapMm, c.orbitSpeed);
    return true;
}

// How long a robot must be undetected before its slot is freed.
static constexpr float EVICT_TIMEOUT_S = 5.0f;

// Nudge slot angles so no two adjacent slots are closer than minGapMm arc-chord.
// arc-chord distance ≈ 2*R*sin(Δθ/2).
static void enforceMinGap(std::unordered_map<int,float>& slots,
                          float radius, float minGapMm)
{
    if (minGapMm <= 0.f || slots.empty()) return;

    // Minimum angle separation in degrees for the given chord distance.
    // chord = 2R sin(Δ/2) → Δ = 2 asin(chord / (2R))
    float minAngDeg = (radius > 0)
        ? (float)(2.0 * std::asin(std::min(1.0, (double)minGapMm / (2.0 * radius))) * 180.0 / M_PI)
        : 0.f;

    if (minAngDeg <= 0.f) return;

    // Sort by current slot angle.
    std::vector<std::pair<int,float>> vec(slots.begin(), slots.end());
    std::sort(vec.begin(), vec.end(), [](auto& a, auto& b){ return a.second < b.second; });

    int N = (int)vec.size();
    // Two-pass forward nudge (wrap-around handled by a second pass).
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = 1; i < N; ++i) {
            float gap = normAngle(vec[i].second - vec[i-1].second);
            if (gap < 0) gap += 360.f;
            if (gap < minAngDeg) {
                vec[i].second = fmodf(vec[i-1].second + minAngDeg + 360.f, 360.f);
            }
        }
        // Check wrap-around gap between last and first.
        float wrapGap = normAngle(vec[0].second + 360.f - vec[N-1].second);
        if (wrapGap < 0) wrapGap += 360.f;
        if (wrapGap < minAngDeg)
            vec[0].second = fmodf(vec[N-1].second + minAngDeg, 360.f);
    }

    for (auto& [id, a] : vec) slots[id] = a;
}

// ── Mouse ─────────────────────────────────────────────────────────────────────

static bool        g_leftClick = false;
static cv::Point   g_clickPt;

static void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) { g_leftClick = true; g_clickPt = {x, y}; }
}

// ── Speed ─────────────────────────────────────────────────────────────────────

static float g_defaultSpeedMult = 0.40f;                   // set by --speed; lower default since MAX_SPEED is now 100

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    std::string serial, ip;
    float radiusMm    = -1.f;
    float minGapMm    = -1.f;
    float orbitSpeed  = -1.f;
    float speedPct    = 70.f;
    bool  doCalibrate = false;
    bool  logPerf     = false;
    bool  logScore    = false;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--serial")       && i+1<argc) serial     = argv[++i];
        if (!strcmp(argv[i], "--ip")           && i+1<argc) ip         = argv[++i];
        if (!strcmp(argv[i], "--radius")       && i+1<argc) radiusMm   = atof(argv[++i]);
        if (!strcmp(argv[i], "--min-gap")      && i+1<argc) minGapMm   = atof(argv[++i]);
        if (!strcmp(argv[i], "--orbit-speed")  && i+1<argc) orbitSpeed = atof(argv[++i]);
        if (!strcmp(argv[i], "--speed")        && i+1<argc) speedPct   = atof(argv[++i]);
        if (!strcmp(argv[i], "--calibrate"))                doCalibrate = true;
        if (!strcmp(argv[i], "--log-perf"))                 logPerf     = true;
        if (!strcmp(argv[i], "--log-score"))                logScore    = true;
    }
    g_defaultSpeedMult = clampf(speedPct / 100.f, 0.05f, 2.0f);

    SwarmClient swarm;
    if (swarm.connect()) printf("[hub] Connected.\n");
    else                 printf("[hub] Not available — will retry.\n");

    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    cfg.debugOverlay = true;
    ArucoTracker tracker(cfg);
    if (!tracker.open()) { fprintf(stderr, "Could not open Basler camera.\n"); return 1; }
    //auto undist = std::make_unique<FisheyeUndistortPreprocessor>();
    //if (undist->load("fisheye_calib.yaml", tracker.frameSize())) tracker.prependPreprocessor(std::move(undist));
    printf("Camera open at %dx%d.\n", tracker.frameSize().width, tracker.frameSize().height);

    if (!doCalibrate && tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
        printf("Loaded homography from %s\n", HOMOGRAPHY_FILE);
    }
    if (doCalibrate && !runCalibration(tracker)) printf("Calibration skipped.\n");

    const char* WIN = "Circle Demo";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
    cv::setMouseCallback(WIN, onMouse, nullptr);

    CircleState circle;
    // Defaults, then try to restore from saved file, then apply any CLI overrides.
    circle.radius      = DEFAULT_RADIUS_MM;
    circle.minGapMm    = DEFAULT_MIN_GAP_MM;
    circle.orbitSpeed  = DEFAULT_ORBIT_SPEED;
    circle.centre      = {tracker.frameSize().width / 2.f, tracker.frameSize().height / 2.f};

    loadCircle(circle);  // no-op if file absent

    if (radiusMm   >= 0.f) circle.radius     = radiusMm;
    if (minGapMm   >= 0.f) circle.minGapMm   = minGapMm;
    if (orbitSpeed >= 0.f) circle.orbitSpeed = orbitSpeed;

    int8_t motors[MAX_ROBOTS][2]     = {};
    int8_t lastMotors[MAX_ROBOTS][2] = {};

    auto sendMotors = [&]() {
        for (int id = 0; id < MAX_ROBOTS; ++id) swarm.setSpeed((uint8_t)id, motors[id][0], motors[id][1]);
        swarm.flush();
    };

    auto lastSend     = std::chrono::steady_clock::now();
    auto lastFpsT     = lastSend;
    auto lastControlT = lastSend;
    auto lastHubRetry = lastSend - std::chrono::seconds(10);
    int  frameCount   = 0;
    float fps         = 0;

    // ── Phase-1 perf instrumentation (see TODO.md "Performance: loop_fps vs
    // cam_fps") ──────────────────────────────────────────────────────────
    // Coarse per-section timings, printed once per second alongside loop_fps,
    // to locate where the loop falls behind cam_fps before optimizing.
    auto   prevIterT  = lastSend;
    double accTotal   = 0, accControl = 0, accDraw = 0, accImshow = 0, accWaitKey = 0;

    std::unordered_map<int, std::chrono::steady_clock::time_point> robotLastSeen;
    std::unordered_map<int, std::chrono::steady_clock::time_point> robotLostSince;

    // Stable slot registry.
    // Each robot is assigned a slot index on first detection; the slot angle is
    // slotIndex * (360 / registeredCount).  Slots only change when a genuinely
    // new robot joins or when one has been absent for > EVICT_TIMEOUT_S.
    // Temporary detection drops (50 % detection rate, etc.) leave the count and
    // angles untouched, so no spurious reassignment occurs.
    std::unordered_map<int, int>   robotSlotIndex;    // id → slot index
    int                             registeredCount = 0;
    std::unordered_map<int, float> persistentSlots;   // id → slot angle (deg)

    // Per-robot EMA-smoothed yaw.  Seeded on first detection; updated only when
    // the robot is visible.  Handles angle wrap via normAngle delta.
    std::unordered_map<int, float> smoothedYaw;
    // D-term rate estimators (see RateEstimator above): position mode tracks
    // d(angleErr)/dt, orbit mode tracks d(yaw)/dt — both sample-and-hold over
    // D_TERM_WINDOW_S.
    std::unordered_map<int, RateEstimator> angleErrRate;
    std::unordered_map<int, RateEstimator> yawRateEst;
    // Last slew-limited turn output per robot (see MAX_TURN_RATE above).
    std::unordered_map<int, float> prevTurnOut;
    // Controller performance score per robot (see ControlScore above).
    std::unordered_map<int, ControlScore> robotScore;

    printf("\nLeft-click = centre  +/- = radius\nt = orbit  [ / ] = orbit speed  s = stop  c = calibrate  q = quit\n\n");

    while (g_running) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        accTotal += std::chrono::duration<double>(now - prevIterT).count();
        prevIterT = now;
        ++frameCount;
        float controlDt = clampf(std::chrono::duration<float>(now - lastControlT).count(), 0.01f, 0.2f);
        lastControlT = now;

        if (!swarm.isConnected() && std::chrono::duration<float>(now - lastHubRetry).count() >= 2.0f) {
            lastHubRetry = now;
            if (swarm.connect()) printf("[hub] Connected.\n");
        }
        swarm.poll();

        float dt = std::chrono::duration<float>(now - lastFpsT).count();
        if (dt >= 1.0f) {
            fps = frameCount / dt;
            if (frameCount > 0 && logPerf) {
                double n = (double)frameCount;
                printf("[perf] loop_fps=%.0f  total=%.2fms  control=%.2fms  draw=%.2fms  imshow=%.2fms  waitKey=%.2fms  other=%.2fms\n",
                       fps, accTotal/n*1000.0, accControl/n*1000.0, accDraw/n*1000.0,
                       accImshow/n*1000.0, accWaitKey/n*1000.0,
                       (accTotal - accControl - accDraw - accImshow - accWaitKey)/n*1000.0);
            }
            frameCount = 0; lastFpsT = now;
            accTotal = accControl = accDraw = accImshow = accWaitKey = 0;

            if (logScore) {
                for (auto& [id, sc] : robotScore) {
                    if (!sc.init) continue;
                    printf("[score] robot %d  bias=%.2fdeg  osc=%.2fdeg  jerk=%.2f  posBias=%+.1fmm  posOsc=%.1fmm  total=%.2f\n",
                           id, fabsf(sc.biasEma), sqrtf(std::max(sc.oscVar, 0.f)),
                           sqrtf(std::max(sc.jerkVar, 0.f)),
                           sc.posBiasEma, sqrtf(std::max(sc.posVar, 0.f)),
                           totalScore(sc));
                }
            }
        }

        // ── Handle click: set new circle centre ───────────────────────────────
        if (g_leftClick) {
            cv::Point2f w = pixelToWorld({(float)g_clickPt.x, (float)g_clickPt.y});
            circle.centre    = w;
            circle.centreSet = true;
            printf("Circle centre: (%.0f, %.0f)  radius: %.0f mm\n", w.x, w.y, circle.radius);
            saveCircle(circle);
            g_leftClick = false;
        }

        // ── Build pose map ────────────────────────────────────────────────────
        std::unordered_map<int, RobotPose> poseById;
        for (auto& r : tracker.robots()) {
            if (r.id < 0 || r.id >= MAX_ROBOTS) continue;
            poseById[r.id] = r;
            robotLastSeen[r.id] = now;
        }

        // Apply EMA to yaw before it reaches any controller.
        // emplace seeds the value on first sight; subsequent frames blend in
        // the raw yaw via normAngle so wrap-around is handled correctly.
        for (auto& [id, pose] : poseById) {
            auto [it, fresh] = smoothedYaw.emplace(id, pose.yaw);
            if (!fresh) {
                float delta = normAngle(pose.yaw - it->second);
                it->second  = normAngle(it->second + YAW_ALPHA * delta);
            }
            pose.yaw = it->second;
        }

        // ── Registration / eviction ───────────────────────────────────────────
        // Newly seen robots get a slot index equal to the current registeredCount,
        // then the count is bumped.  Robots that briefly drop out of detection
        // keep their slot.  Only after EVICT_TIMEOUT_S of absence is the slot
        // freed and the remaining indices compacted.
        {
            bool countChanged = false;

            // Register new robots; clear their lost timer.
            for (auto& [id, _] : poseById) {
                robotLostSince.erase(id);
                if (!robotSlotIndex.count(id)) {
                    swarm.registerRobot((uint8_t)id);
                    robotSlotIndex[id] = registeredCount++;
                    printf("[circle] Robot %d registered → slot %d / %d\n",
                           id, robotSlotIndex[id], registeredCount);
                    countChanged = true;
                }
            }

            // Start / maintain lost timer for registered robots not seen this frame.
            for (auto& [id, _] : robotSlotIndex) {
                if (!poseById.count(id) && !robotLostSince.count(id))
                    robotLostSince[id] = now;
            }

            // Evict robots that have exceeded the timeout.
            {
                std::vector<int> toEvict;
                for (auto& [id, t] : robotLostSince) {
                    if (robotSlotIndex.count(id) &&
                        std::chrono::duration<float>(now - t).count() > EVICT_TIMEOUT_S)
                        toEvict.push_back(id);
                }
                for (int id : toEvict) {
                    int evicted = robotSlotIndex[id];
                    robotSlotIndex.erase(id);
                    robotLostSince.erase(id);
                    persistentSlots.erase(id);
                    smoothedYaw.erase(id);
                    angleErrRate.erase(id);
                    yawRateEst.erase(id);
                    prevTurnOut.erase(id);
                    robotScore.erase(id);
                    for (auto& [rid, sidx] : robotSlotIndex)
                        if (sidx > evicted) sidx--;
                    registeredCount--;
                    countChanged = true;
                    printf("[circle] Robot %d evicted (lost >%.0fs)  remaining: %d\n",
                           id, EVICT_TIMEOUT_S, registeredCount);
                }
            }

            // Recompute all slot angles whenever the count changes.
            if (countChanged && registeredCount > 0) {
                persistentSlots.clear();
                for (auto& [id, slotIdx] : robotSlotIndex)
                    persistentSlots[id] = slotIdx * 360.f / registeredCount;
                enforceMinGap(persistentSlots, circle.radius, circle.minGapMm);
            }
        }

        // slotAngles is the view used by the control loop this frame
        // (includes all registered robots, not just currently visible ones).
        std::unordered_map<int, float> slotAngles = persistentSlots;

        // ── Orbit: current angle and adjacency ────────────────────────────────
        // Robots sorted by current angle CCW; used by both orbit control and HUD.
        // byAngle[i] = {current_angle_deg, robot_id}
        std::vector<std::pair<float,int>> byAngle;
        if (circle.centreSet) {
            for (auto& [id, pose] : poseById) {
                float a = atan2f(pose.y - circle.centre.y, pose.x - circle.centre.x) * 180.f / (float)M_PI;
                if (a < 0) a += 360.f;
                byAngle.push_back({a, id});
            }
            std::sort(byAngle.begin(), byAngle.end());
        }

        // Precompute chord distance to each robot's leading neighbour in orbit
        // direction (positive orbitSpeed = CCW = increasing angle index).
        // chordToLeader[id] = chord distance to the next robot ahead.
        std::unordered_map<int,float> chordToLeader;
        {
            int M = (int)byAngle.size();
            if (M >= 2) {
                for (int i = 0; i < M; ++i) {
                    // Leading neighbour: next index CCW for positive speed, prev for negative.
                    int jLead = (circle.orbitSpeed >= 0.f)
                        ? (i + 1) % M
                        : (i - 1 + M) % M;
                    int idA = byAngle[i].second;
                    int idB = byAngle[jLead].second;
                    auto& pA = poseById[idA];
                    auto& pB = poseById[idB];
                    float dx = pB.x - pA.x, dy = pB.y - pA.y;
                    chordToLeader[idA] = sqrtf(dx*dx + dy*dy);
                }
            }
        }

        memcpy(motors, lastMotors, sizeof(motors));

        for (auto& [id, pose] : poseById) {
            if (!circle.centreSet) {
                motors[id][0] = motors[id][1] = 0;
                continue;
            }

            if (circle.tracking) {
                // ── Orbit mode: velocity-based controller ─────────────────────
                //
                // Desired world-space velocity = tangential + radial correction.
                //   tangential: keeps the robot moving around the circle at orbitSpeed.
                //   radial: P-correction that pulls the robot back onto the circle.
                //
                // Min-gap: if the robot is closer than minGapMm to the robot ahead,
                // scale the tangential component down linearly to zero.

                float dx_c = pose.x - circle.centre.x;
                float dy_c = pose.y - circle.centre.y;
                float distC = sqrtf(dx_c*dx_c + dy_c*dy_c);
                if (distC < 1.f) { motors[id][0] = motors[id][1] = 0; continue; }

                // Radial unit vector (outward).
                float rx = dx_c / distC, ry = dy_c / distC;
                // Tangential unit vector: CCW for positive orbitSpeed.
                float tx = (circle.orbitSpeed >= 0.f) ? -ry :  ry;
                float ty = (circle.orbitSpeed >= 0.f) ?  rx : -rx;

                // Tangential speed: angular rate (deg/s → rad/s) × radius, capped, then scaled.
                // g_defaultSpeedMult multiplies the cap (not the raw velocity) so it has a
                // linear effect across its full range regardless of how fast the orbit
                // geometry wants to run — consistent with position mode's
                // "maxSpd = MAX_SPEED * g_defaultSpeedMult".
                float omegaRad = fabsf(circle.orbitSpeed) * (float)M_PI / 180.f;
                float vTan = std::min(omegaRad * circle.radius, MAX_SPEED) * g_defaultSpeedMult;

                // Scale tangential speed down when too close to the leading robot.
                if (circle.minGapMm > 0.f && chordToLeader.count(id)) {
                    float chord = chordToLeader[id];
                    // Full speed above 2× min gap, zero at min gap.
                    float gapFactor = clampf((chord - circle.minGapMm) / circle.minGapMm, 0.f, 1.f);
                    vTan *= gapFactor;
                }

                // Radial correction: negative error (inside circle) → push outward.
                float vRad = -K_RAD * (distC - circle.radius);
                vRad = clampf(vRad, -MAX_SPEED * 0.5f, MAX_SPEED * 0.5f);

                // Desired velocity vector in world frame.
                float vx = vTan * tx + vRad * rx;
                float vy = vTan * ty + vRad * ry;
                float vMag = sqrtf(vx*vx + vy*vy);

                if (vMag < 0.5f) { motors[id][0] = motors[id][1] = 0; continue; }

                float desHeading = atan2f(vy, vx) * 180.f / (float)M_PI;
                float angleErr   = normAngle(desHeading - pose.yaw);
                float headingN   = clampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
                float headingSc  = 1.f - headingN * headingN;

                // ── Yaw feedforward + feedback ─────────────────────────────────
                //
                // Why pure-feedback PD oscillates here: tracking a circle means
                // desHeading itself continuously rotates — it's the tangent
                // direction, which advances at exactly the rate the robot is
                // actually moving around the circle. A P-term can only produce
                // a *sustained* turning output from a *sustained* error, so the
                // faster the orbit, the larger the steady heading lag has to be
                // to drive enough turn. Raise K_ANGLE to shrink that lag and the
                // loop overshoots/rings instead; back it off and the robot just
                // trails the path — exactly the "oscillate or lag" dead end.
                //
                // The fix: stop asking feedback to manufacture the entire steady
                // turning effort — feed the required yaw rate forward directly
                // so feedback only has to correct the residual (noise, slip,
                // model mismatch). Required rate = v / R. We use the *actual*
                // commanded tangential speed vTan (after MAX_SPEED capping, the
                // per-robot multiplier, and min-gap throttling) rather than the
                // nominal orbitSpeed, so the feedforward stays honest about what
                // the robot can really do — the nominal product (omegaRad ×
                // radius) routinely exceeds MAX_SPEED, and feeding *that*
                // forward would overdrive the turn and reintroduce oscillation.
                float dirSign    = (circle.orbitSpeed >= 0.f) ? 1.f : -1.f;
                float ffOmegaDeg = dirSign * (vTan / circle.radius) * (180.f / (float)M_PI);
                float turnFF     = K_FF_YAW * ffOmegaDeg;

                // D-term: (ffOmegaDeg − yawRate) is the analytic d(angleErr)/dt
                // — the same quantity the old code numerically differenced out
                // of the atan2-derived (and therefore noisy) angleErr, but here
                // built from one clean measurement (yawRate) instead of
                // finite-differencing two noisy signals. yawRate itself is
                // sample-and-held over D_TERM_WINDOW_S (see RateEstimator) so
                // it reflects real angular velocity, not per-frame yaw jitter.
                float yawRate    = updateRate(yawRateEst[id], pose.yaw, now, D_TERM_WINDOW_S);
                float dAngleErr  = clampf(ffOmegaDeg - yawRate, -300.f, 300.f);

                // Gate the feedforward by headingSc too: turnFF assumes the robot
                // is cruising tangentially at vTan, which only holds once it's
                // roughly aligned. While |angleErr| >= 90° (headingSc == 0, pure
                // spin-to-align), an unconditional turnFF can cancel the feedback
                // term to ~0 turn — with forward already 0 from headingSc, both
                // motors land at ~0 and the robot freezes in a self-consistent
                // deadlock (nothing changes next frame, so it never recovers).
                // Dropping turnFF here leaves pure feedback, which for
                // |angleErr| >= 90° always produces |turn| >= K_ANGLE*90 — large
                // enough to clamp to MAX_TURN, so the deadlock can't form.
                float forward = clampf(vMag, 0.f, MAX_SPEED) * headingSc;
                float turnTgt = clampf(turnFF * headingSc + K_ANGLE * angleErr + K_YAW_D * dAngleErr,
                                       -MAX_TURN, MAX_TURN);
                float maxStep = MAX_TURN_RATE * controlDt;
                float turn    = clampf(turnTgt, prevTurnOut[id] - maxStep, prevTurnOut[id] + maxStep);
                prevTurnOut[id] = turn;

                motors[id][0] = (int8_t)clampf(forward + turn, -100, 100);
                motors[id][1] = (int8_t)clampf(forward - turn, -100, 100);

                updateScore(robotScore[id], angleErr, turn, distC - circle.radius, controlDt);

            } else {
                // ── Position mode: drive to assigned slot and stop ────────────
                if (!slotAngles.count(id)) {
                    motors[id][0] = motors[id][1] = 0;
                    continue;
                }

                float slotAngleRad = slotAngles[id] * (float)M_PI / 180.f;
                float tgtX = circle.centre.x + circle.radius * cosf(slotAngleRad);
                float tgtY = circle.centre.y + circle.radius * sinf(slotAngleRad);

                float dx   = tgtX - pose.x;
                float dy   = tgtY - pose.y;
                float dist = sqrtf(dx*dx + dy*dy);

                if (dist < ARRIVAL_MM) { motors[id][0] = motors[id][1] = 0; continue; }

                float tgtAngle  = atan2f(dy, dx) * 180.f / (float)M_PI;
                float angleErr  = normAngle(tgtAngle - pose.yaw);
                float headingN  = clampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
                float headingSc = 1.f - headingN * headingN;
                float brakeSc   = clampf((dist - ARRIVAL_MM) / ARRIVAL_MM, 0.f, 1.f);
                float maxSpd    = MAX_SPEED * g_defaultSpeedMult;

                // Sample-and-held over D_TERM_WINDOW_S — see RateEstimator.
                float dAngleErr = clampf(updateRate(angleErrRate[id], angleErr, now, D_TERM_WINDOW_S),
                                         -300.f, 300.f);

                float forward = clampf(K_DIST * dist, 0.f, maxSpd) * headingSc * brakeSc;
                float turnTgt = clampf(K_ANGLE * angleErr + K_YAW_D * dAngleErr,
                                       -MAX_TURN, MAX_TURN);
                float maxStep = MAX_TURN_RATE * controlDt;
                float turn    = clampf(turnTgt, prevTurnOut[id] - maxStep, prevTurnOut[id] + maxStep);
                prevTurnOut[id] = turn;

                motors[id][0] = (int8_t)clampf(forward + turn, -100, 100);
                motors[id][1] = (int8_t)clampf(forward - turn, -100, 100);

                float distCentre = sqrtf((pose.x - circle.centre.x)*(pose.x - circle.centre.x)
                                        + (pose.y - circle.centre.y)*(pose.y - circle.centre.y));
                updateScore(robotScore[id], angleErr, turn, distCentre - circle.radius, controlDt);
            }
        }

        // ── Watchdog: stop invisible robots after 1 s ─────────────────────────
        for (int id = 0; id < MAX_ROBOTS; ++id) {
            if (poseById.count(id)) continue;
            auto it = robotLastSeen.find(id);
            if (it == robotLastSeen.end()) continue;
            auto gapMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count();
            if (gapMs > 1000) {
                motors[id][0] = motors[id][1] = 0;
                lastMotors[id][0] = lastMotors[id][1] = 0;
            }
        }

        if (std::chrono::duration<float>(now - lastSend).count() >= SEND_INTERVAL_S) {
            sendMotors();
            lastSend = now;
            memcpy(lastMotors, motors, sizeof(motors));
        }

        auto tControlEnd = std::chrono::steady_clock::now();
        accControl += std::chrono::duration<double>(tControlEnd - now).count();

        // ── Draw HUD ──────────────────────────────────────────────────────────
        cv::Mat disp = tracker.debugFrame().clone();

        // Circle overlay (always, even before centre is clicked).
        cv::Point2f centrePx = worldToPixel(circle.centre);
        // Radius in pixels: use a known reference point offset in world space.
        float radiusPx = circle.radius;
        if (g_hasH) {
            cv::Point2f edgePx = worldToPixel({circle.centre.x + circle.radius, circle.centre.y});
            radiusPx = (float)cv::norm(edgePx - centrePx);
        }
        cv::Scalar circleCol = circle.centreSet ? cv::Scalar(0, 200, 255) : cv::Scalar(80, 80, 80);
        cv::circle(disp, centrePx, (int)radiusPx, circleCol, 2, cv::LINE_AA);
        cv::drawMarker(disp, centrePx, circleCol, cv::MARKER_CROSS, 20, 2, cv::LINE_AA);

        if (!circle.tracking) {
            // ── Position mode: slot markers and arrows ────────────────────────
            for (auto& [id, pose] : poseById) {
                if (!slotAngles.count(id)) continue;

                float slotAngleRad = slotAngles[id] * (float)M_PI / 180.f;
                float tx = circle.centre.x + circle.radius * cosf(slotAngleRad);
                float ty = circle.centre.y + circle.radius * sinf(slotAngleRad);
                cv::Point2f slotPx = worldToPixel({tx, ty});

                cv::circle(disp, slotPx, 7, {0, 255, 128}, -1, cv::LINE_AA);

                float dist = sqrtf((tx-pose.x)*(tx-pose.x) + (ty-pose.y)*(ty-pose.y));
                if (dist > ARRIVAL_MM)
                    cv::arrowedLine(disp, {(int)pose.px,(int)pose.py}, slotPx,
                                    {255,128,0}, 2, cv::LINE_AA, 0, 0.15);

                tracker.drawText(disp, std::to_string(id),
                    slotPx + cv::Point2f(8,-8), 18, {0,255,128});
            }
        } else {
            // ── Orbit mode: velocity arrows ───────────────────────────────────
            for (auto& [id, pose] : poseById) {
                float dx_c = pose.x - circle.centre.x;
                float dy_c = pose.y - circle.centre.y;
                float distC = sqrtf(dx_c*dx_c + dy_c*dy_c);
                if (distC < 1.f) continue;

                float rx = dx_c/distC, ry = dy_c/distC;
                float vtx = (circle.orbitSpeed >= 0.f) ? -ry :  ry;
                float vty = (circle.orbitSpeed >= 0.f) ?  rx : -rx;

                float omegaRad = fabsf(circle.orbitSpeed) * (float)M_PI / 180.f;
                float vTan = std::min(omegaRad * circle.radius, MAX_SPEED) * g_defaultSpeedMult;
                if (circle.minGapMm > 0.f && chordToLeader.count(id))
                    vTan *= clampf((chordToLeader[id] - circle.minGapMm) / circle.minGapMm, 0.f, 1.f);

                float vRad = clampf(-K_RAD * (distC - circle.radius), -MAX_SPEED*0.5f, MAX_SPEED*0.5f);

                float vx = vTan*vtx + vRad*rx;
                float vy = vTan*vty + vRad*ry;

                // Scale arrow to 60px at MAX_SPEED for readability.
                float vMag = sqrtf(vx*vx + vy*vy);
                if (vMag < 0.5f) continue;
                float arrowScale = 60.f / MAX_SPEED;
                cv::Point2f pRobot(pose.px, pose.py);
                cv::Point2f pTip = worldToPixel({pose.x + vx*arrowScale, pose.y + vy*arrowScale});
                cv::arrowedLine(disp, pRobot, pTip, {0, 220, 255}, 2, cv::LINE_AA, 0, 0.2);
            }
        }

        // ── Neighbour distances ───────────────────────────────────────────────
        // Uses byAngle (current positions) so it works in both modes.
        {
            int M = (int)byAngle.size();
            for (int i = 0; i < M; ++i) {
                int idA = byAngle[i].second;
                int idB = byAngle[(i + 1) % M].second;
                if (idA == idB) continue;

                auto& pA = poseById[idA];
                auto& pB = poseById[idB];
                float dx = pB.x - pA.x, dy = pB.y - pA.y;
                float chordDist = sqrtf(dx*dx + dy*dy);

                cv::Point2f pxA(pA.px, pA.py);
                cv::Point2f pxB(pB.px, pB.py);
                cv::Point2f mid = (pxA + pxB) * 0.5f;

                bool tooClose = (circle.minGapMm > 0 && chordDist < circle.minGapMm);
                cv::Scalar lineCol = tooClose ? cv::Scalar(0,0,255) : cv::Scalar(200,200,200);
                cv::line(disp, pxA, pxB, lineCol, 1, cv::LINE_AA);

                char label[32];
                snprintf(label, sizeof(label), "%.0fmm", chordDist);
                tracker.drawText(disp, label, mid + cv::Point2f(4,-4), 18, lineCol);
            }
        }

        // ── Status bar ────────────────────────────────────────────────────────
        DebugHud hud;
        hud.title(DebugHud::fmt(
            "loop_fps:%.0f  Robots:%d/%d  R:%.0fmm  Gap:%.0fmm  %s  HUB:%s  %s",
            fps, (int)poseById.size(), registeredCount,
            circle.radius, circle.minGapMm,
            g_hasH ? "world" : "pixels",
            swarm.isConnected() ? "OK" : "INACTIVE",
            circle.tracking
                ? cv::format("ORBIT %.0fdeg/s", circle.orbitSpeed).c_str()
                : "POSITION"),
            swarm.isConnected() ? DebugHud::COL_OK : DebugHud::COL_BAD);
        hud.draw(disp, {10, disp.rows - 36});

        auto tDrawEnd = std::chrono::steady_clock::now();
        accDraw += std::chrono::duration<double>(tDrawEnd - tControlEnd).count();

        cv::imshow(WIN, disp);

        auto tImshowEnd = std::chrono::steady_clock::now();
        accImshow += std::chrono::duration<double>(tImshowEnd - tDrawEnd).count();

        int key = cv::waitKey(1) & 0xFF;

        accWaitKey += std::chrono::duration<double>(std::chrono::steady_clock::now() - tImshowEnd).count();
        if (key == 'q' || key == 27) break;
        if (key == 's') {
            memset(motors, 0, sizeof(motors));
            memset(lastMotors, 0, sizeof(lastMotors));
            sendMotors();
            printf("Stopped.\n");
        }
        if (key == 'c') runCalibration(tracker);
        if (key == '+' || key == '=') {
            circle.radius += 25.f;
            printf("Radius: %.0f mm\n", circle.radius);
            saveCircle(circle);
        }
        if (key == '-') {
            circle.radius = std::max(50.f, circle.radius - 25.f);
            printf("Radius: %.0f mm\n", circle.radius);
            saveCircle(circle);
        }
        if (key == 't') {
            circle.tracking = !circle.tracking;
            printf("Tracking: %s\n", circle.tracking ? "ON" : "OFF");
        }
        if (key == ']') {
            circle.orbitSpeed += 5.f;
            printf("Orbit speed: %.0f deg/s\n", circle.orbitSpeed);
            saveCircle(circle);
        }
        if (key == '[') {
            circle.orbitSpeed -= 5.f;
            printf("Orbit speed: %.0f deg/s\n", circle.orbitSpeed);
            saveCircle(circle);
        }
    }

    g_running = false;
    memset(motors, 0, sizeof(motors));
    sendMotors();
    cv::destroyAllWindows();
    printf("Stopped.\n");
    return 0;
}
