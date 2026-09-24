// circle_speed_test.cpp — Motor-command -> real-world mm/s calibration tool,
// on the circular path, matching circle_demo.cpp's orbit-mode controller.
//
// Straight-line commands and circular-orbit commands don't reach the same
// real speed for the same motor value — orbiting spends part of the
// differential-drive budget on turning — so this is the companion to
// max_speed_test.cpp rather than a replacement: run both if a controller
// (like car_following.cpp's ring orbit) drives robots in circles in practice.
//
// The controller here is circle_demo.cpp's orbit-mode block (same gains,
// same feedforward+PD heading law, same worldToPixel/pixelToWorld drawing),
// trimmed to a single selected robot and parameterised by a directly
// commanded tangential speed (1-100 motor units, exactly like circle_demo's
// own vTan — see its "velocity field is in motor units, not mm/s" note)
// instead of deriving vTan from an orbit-speed-in-deg/s setting. It shares
// circle_demo's saved ring fixture (/tmp/circle_demo.yml), so a circle set
// up in circle_demo can be reused here and vice versa.
//
// Usage: ./circle_speed_test [--serial SN] [--ip IP] [--robot ID]
//                             [--speed N] [--run-time S] [--dir cw|ccw]
//                             [--calibrate] [--out-dir DIR]
//                             [--plot] [--plot-min N] [--plot-max N] [--plot-step N]
//
// Keys:
//   0-9        build a target speed (1-100) digit by digit
//   Enter      commit the typed speed
//   Backspace  remove the last typed digit
//   left-click set the circle centre (needs a homography)
//   +/-        circle radius ±25mm
//   d          flip orbit direction (cw/ccw)
//   space, g   run: first seeks the robot onto the ring (radial-only, no
//              orbiting), then orbits it at the committed speed for --run-time
//              once it's within SEEK_ARRIVAL_MM — see the SEEKING/RUNNING
//              phase note below
//   s          stop immediately
//   n/p        select next/previous visible robot
//   P          run the full sweep (--plot-min..--plot-max step --plot-step) now
//   w          save the session log (CSV + PNG plot) to --out-dir now
//   l          clear the session log
//   c          (re)calibrate the homography
//   q, Esc     quit (also stops the robot)
//
// A run needs a loaded homography and a circle centre (click one, or reuse
// circle_demo's saved one) — RunResult.avgMms is only real mm/s once both are
// set, exactly as max_speed_test needs a homography.

#include "aruco_tracker.h"
#include "SwarmClient.h"
#include "DemoHud.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <ctime>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <algorithm>
#include <chrono>
#include <thread>
#include <fstream>
#include <filesystem>

// ── Controller tunables — copied from circle_demo.cpp's orbit-mode block so
// this tool exercises the exact same control law it's calibrating for ───────

static constexpr float DEFAULT_RADIUS_MM = 300.0f;
static constexpr float K_ANGLE           = 0.45f;
static constexpr float K_YAW_D           = 0.15f;
static constexpr float K_FF_YAW          = 1.00f;
static constexpr float K_RAD             = 0.30f;
static constexpr float MAX_SPEED         = 100.0f;  // motor units — circle_demo's vTan is in this scale, not mm/s
static constexpr float MAX_TURN          = 20.0f;
static constexpr float MAX_TURN_RATE     = 120.0f;   // turn-units/s
static constexpr float CONTROL_INTERVAL_S = 0.02f;
static constexpr float D_TERM_WINDOW_S    = CONTROL_INTERVAL_S;

static constexpr float SPEED_TAU_S  = 0.15f;  // low-pass on the instantaneous mm/s reading
static constexpr float WARMUP_FRAC  = 0.35f;  // fraction of run-time excluded (orbit settle: radius + heading)
static constexpr int   MAX_ROBOTS   = SC_MAX_ROBOTS;
static constexpr int   LOG_MAX_ROWS = 8;

// A run starts wherever the robot happens to be, which — without this — would
// put the "get onto the ring" transient inside the timed window and rely
// entirely on WARMUP_FRAC to hide it. Instead a run has an explicit SEEKING
// phase first (radial-only: vTan = 0, so it moves onto the ring without
// orbiting), and only switches to RUNNING (and starts the clock) once it's
// within SEEK_ARRIVAL_MM of the ring — the same explicit-lifecycle shape
// car_following.cpp's CfRunState uses for its own start cue.
static constexpr float SEEK_ARRIVAL_MM = 20.f;
static constexpr float SEEK_TIMEOUT_S  = 8.f;   // abort if it never reaches the ring (bad radius, stuck robot, ...)

// Exe-relative, not /tmp — matches circle_demo.cpp/car_following.cpp
// (arucoVisionDataPath() in aruco_tracker.h) so this tool reads/writes the
// exact same fixture files circle_demo does, and they survive a reboot
// instead of living on tmpfs.
static const std::string HOMOGRAPHY_FILE_S = arucoVisionDataPath("aruco_homography.yml");
static const std::string CIRCLE_FILE_S     = arucoVisionDataPath("circle_demo.yml");
static const char* HOMOGRAPHY_FILE = HOMOGRAPHY_FILE_S.c_str();
static const char* CIRCLE_FILE     = CIRCLE_FILE_S.c_str();

// ── Globals ───────────────────────────────────────────────────────────────────

static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

static float normAngle(float a) { while (a > 180.f) a -= 360.f; while (a < -180.f) a += 360.f; return a; }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static cv::Mat g_H;
static bool    g_hasH = false;
static void setH(const cv::Mat& H) { g_H = H; g_hasH = !g_H.empty(); }
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

// ── Circle fixture — same file/keys as circle_demo.cpp ──────────────────────

struct CircleState {
    cv::Point2f centre;
    float       radius     = DEFAULT_RADIUS_MM;
    float       minGapMm   = 0.f;
    bool        centreSet  = false;
    float       orbitSpeed = 30.f;  // only its sign is used here (initial direction)
};

static void saveCircle(const CircleState& c) {
    cv::FileStorage fs(CIRCLE_FILE, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { fprintf(stderr, "[circle] Could not save to %s\n", CIRCLE_FILE); return; }
    fs << "cx" << c.centre.x << "cy" << c.centre.y << "centre_set" << (int)c.centreSet
       << "radius" << c.radius << "min_gap" << c.minGapMm << "orbit_spd" << c.orbitSpeed;
    printf("[circle] Saved: centre=(%.0f,%.0f) radius=%.0fmm\n", c.centre.x, c.centre.y, c.radius);
}
static bool loadCircle(CircleState& c) {
    cv::FileStorage fs(CIRCLE_FILE, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    float cx = 0, cy = 0, r = 0, gap = -1.f, spd = 0.f;
    int centreSet = 0;
    fs["cx"] >> cx; fs["cy"] >> cy; fs["centre_set"] >> centreSet;
    fs["radius"] >> r; fs["min_gap"] >> gap; fs["orbit_spd"] >> spd;
    if (centreSet) { c.centre = {cx, cy}; c.centreSet = true; }
    if (r > 0.f) c.radius = r;
    if (gap >= 0.f) c.minGapMm = gap;
    if (spd != 0.f) c.orbitSpeed = spd;
    printf("[circle] Loaded: centre=(%.0f,%.0f) radius=%.0fmm\n", c.centre.x, c.centre.y, c.radius);
    return true;
}

static bool g_leftClick = false;
static cv::Point g_clickPt;
static void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) { g_leftClick = true; g_clickPt = {x, y}; }
}

// ── Homography calibration (click 4 corners, enter arena size) ──────────────

struct CalibState { std::vector<cv::Point2f> pixPts; bool done = false; };
static void onCalibMouse(int event, int x, int y, int, void* ud) {
    auto* s = (CalibState*)ud;
    if (event == cv::EVENT_LBUTTONDOWN && s->pixPts.size() < 4) {
        s->pixPts.push_back({(float)x, (float)y});
        printf("  corner %d: (%d, %d)\n", (int)s->pixPts.size(), x, y);
        if (s->pixPts.size() == 4) s->done = true;
    }
}
static bool runCalibration(ArucoTracker& tracker, const char* win) {
    printf("\nCalibration: click 4 corners TL TR BR BL\n");
    for (int attempts = 0; !tracker.update(); ) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (++attempts > 100) { fprintf(stderr, "[calib] timeout\n"); return false; }
    }
    cv::Mat frame = tracker.debugFrame().clone();
    CalibState cs;
    cv::setMouseCallback(win, onCalibMouse, &cs);
    while (!cs.done) {
        cv::Mat disp = frame.clone();
        for (auto& p : cs.pixPts) cv::circle(disp, p, 8, {0,0,255}, -1);
        ArucoTracker::drawText(disp,
            "Click corners TL TR BR BL  " + std::to_string(cs.pixPts.size()) + "/4",
            {10, 40}, 20, {0, 255, 0});
        cv::imshow(win, disp);
        if (cv::waitKey(30) == 27) { cv::setMouseCallback(win, onMouse, nullptr); return false; }
    }
    cv::setMouseCallback(win, onMouse, nullptr);

    printf("Arena width height mm (e.g. 800 600): ");
    float W = 0, H = 0;
    if (scanf("%f %f", &W, &H) != 2 || W <= 0 || H <= 0) return false;

    std::vector<cv::Point2f> worldPts = {{0,0},{W,0},{W,H},{0,H}};
    cv::Mat H_ = cv::findHomography(cs.pixPts, worldPts);
    if (H_.empty()) return false;
    tracker.setHomography(cs.pixPts, worldPts);
    tracker.saveHomography(HOMOGRAPHY_FILE);
    setH(H_);
    printf("Saved homography.\n");
    return true;
}

// ── Heading-rate estimator — same sample-and-hold shape as circle_demo.cpp ──

struct RateEstimator {
    bool init = false;
    float baseAngle = 0.f, rate = 0.f;
    std::chrono::steady_clock::time_point baseTime;
};
static float updateRate(RateEstimator& r, float angle, std::chrono::steady_clock::time_point now, float windowS) {
    if (!r.init) { r.init = true; r.baseAngle = angle; r.baseTime = now; return 0.f; }
    float elapsed = std::chrono::duration<float>(now - r.baseTime).count();
    if (elapsed >= windowS) {
        float d = normAngle(angle - r.baseAngle);
        r.rate = d / std::max(elapsed, 1e-3f);
        r.baseAngle = angle;
        r.baseTime = now;
    }
    return r.rate;
}

// ── Session log + plot/CSV output ────────────────────────────────────────────

struct RunResult {
    int   speedCmd;
    float avgMms;
    float maxMms;
    int   nSamples;
};

enum class Phase { Idle, Seeking, Running };

struct RunState {
    double startS   = 0.0;
    float  sumMms   = 0.f;
    int    nSamples = 0;
    float  maxMms   = 0.f;
};

static std::string timestampStr() {
    std::time_t t = std::time(nullptr);
    std::tm tmv{};
    localtime_r(&t, &tmv);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tmv);
    return buf;
}

// Simple axes + points + connecting line — no external plotting dependency,
// consistent with the rest of the vision tools already linking OpenCV.
static cv::Mat renderPlot(const std::vector<std::pair<int,float>>& pts) {
    const int W = 900, H = 600, mL = 70, mR = 30, mT = 40, mB = 60;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(250, 250, 250));

    float yMax = 10.f;
    for (auto& p : pts) yMax = std::max(yMax, p.second);
    yMax = std::ceil(yMax / 50.f) * 50.f;
    if (yMax <= 0.f) yMax = 50.f;

    auto X = [&](float v) { return mL + (int)(v / 100.f * (W - mL - mR)); };
    auto Y = [&](float v) { return H - mB - (int)(v / yMax * (H - mT - mB)); };

    for (int v = 0; v <= 100; v += 10) {
        int x = X((float)v);
        cv::line(img, {x, mT}, {x, H - mB}, {228,228,228}, 1, cv::LINE_AA);
        cv::line(img, {x, H - mB}, {x, H - mB + 6}, {60,60,60}, 1, cv::LINE_AA);
        cv::putText(img, std::to_string(v), {x - 10, H - mB + 22}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {60,60,60}, 1, cv::LINE_AA);
    }
    for (int i = 0; i <= 5; i++) {
        float v = yMax * i / 5.f;
        int y = Y(v);
        cv::line(img, {mL, y}, {W - mR, y}, {228,228,228}, 1, cv::LINE_AA);
        cv::line(img, {mL - 6, y}, {mL, y}, {60,60,60}, 1, cv::LINE_AA);
        cv::putText(img, DemoHud::fmt("%.0f", v), {8, y + 4}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {60,60,60}, 1, cv::LINE_AA);
    }
    cv::line(img, {mL, mT}, {mL, H - mB}, {60,60,60}, 2, cv::LINE_AA);
    cv::line(img, {mL, H - mB}, {W - mR, H - mB}, {60,60,60}, 2, cv::LINE_AA);
    cv::putText(img, "commanded speed (motor units)", {mL, H - 14}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {20,20,20}, 1, cv::LINE_AA);
    cv::putText(img, "measured mm/s (orbit)", {mL, mT - 14}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {20,20,20}, 1, cv::LINE_AA);

    cv::Point prev; bool havePrev = false;
    for (auto& p : pts) {
        cv::Point pt(X((float)p.first), Y(p.second));
        if (havePrev) cv::line(img, prev, pt, {200,120,0}, 2, cv::LINE_AA);
        cv::circle(img, pt, 4, {0,90,255}, -1, cv::LINE_AA);
        prev = pt; havePrev = true;
    }
    return img;
}

static void saveResults(const std::vector<RunResult>& log, const std::string& outDir) {
    if (log.empty()) { printf("[save] log is empty — run a test first.\n"); return; }
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
    if (ec) { fprintf(stderr, "[save] could not create '%s': %s\n", outDir.c_str(), ec.message().c_str()); return; }

    std::string ts  = timestampStr();
    std::string csvPath = outDir + "/circle_speed_test_" + ts + ".csv";
    std::string pngPath = outDir + "/circle_speed_test_" + ts + ".png";

    std::ofstream csv(csvPath);
    if (!csv) { fprintf(stderr, "[save] could not open '%s'\n", csvPath.c_str()); return; }
    csv << "speed_cmd,avg_mms,max_mms,samples\n";
    std::map<int, std::vector<float>> bySpeed;
    for (auto& r : log) {
        csv << r.speedCmd << "," << r.avgMms << "," << r.maxMms << "," << r.nSamples << "\n";
        bySpeed[r.speedCmd].push_back(r.avgMms);
    }
    csv.close();

    std::vector<std::pair<int,float>> pts;
    for (auto& [spd, vals] : bySpeed) {
        float sum = 0.f;
        for (float v : vals) sum += v;
        pts.push_back({spd, sum / vals.size()});
    }
    cv::imwrite(pngPath, renderPlot(pts));

    printf("[save] wrote %s and %s (%zu runs, %zu speed points)\n",
           csvPath.c_str(), pngPath.c_str(), log.size(), pts.size());
}

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    std::string serial, ip, outDir = "speed_test_results";
    int   robotArg     = -1;
    int   initialSpeed = 50;
    float runTimeS     = 4.0f;
    bool  doCalib      = false;
    bool  dirCw        = false;
    bool  plotAtStart  = false;
    int   plotMin = 10, plotMax = 100, plotStep = 10;

    for (int i = 1; i < argc; i++) {
        auto arg = [&](const char* n) { return strcmp(argv[i], n) == 0 && i + 1 < argc; };
        if      (arg("--serial"))    serial       = argv[++i];
        else if (arg("--ip"))        ip           = argv[++i];
        else if (arg("--robot"))     robotArg     = atoi(argv[++i]);
        else if (arg("--speed"))     initialSpeed = atoi(argv[++i]);
        else if (arg("--run-time"))  runTimeS     = (float)atof(argv[++i]);
        else if (arg("--out-dir"))   outDir       = argv[++i];
        else if (arg("--plot-min"))  plotMin      = atoi(argv[++i]);
        else if (arg("--plot-max"))  plotMax      = atoi(argv[++i]);
        else if (arg("--plot-step")) plotStep     = atoi(argv[++i]);
        else if (arg("--dir")) {
            const char* d = argv[++i];
            if      (!strcmp(d, "cw"))  dirCw = true;
            else if (!strcmp(d, "ccw")) dirCw = false;
            else { fprintf(stderr, "--dir must be cw or ccw, got: %s\n", d); return 2; }
        }
        else if (!strcmp(argv[i], "--calibrate")) doCalib = true;
        else if (!strcmp(argv[i], "--plot"))       plotAtStart = true;
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("usage: %s [--serial SN] [--ip IP] [--robot ID] [--speed N]\n"
                   "       [--run-time S] [--dir cw|ccw] [--calibrate] [--out-dir DIR]\n"
                   "       [--plot] [--plot-min N] [--plot-max N] [--plot-step N]\n", argv[0]);
            return 0;
        } else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 2; }
    }

    int   targetSpeed = (int)clampf((float)initialSpeed, 1.f, 100.f);
    int   typedSpeed  = -1;
    int   selectedId  = robotArg;
    float dirSign     = dirCw ? -1.f : 1.f;

    SwarmClient swarm;
    if (swarm.connect()) printf("[hub] Connected.\n");
    else                  printf("[hub] Not available — will retry.\n");

    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    cfg.debugOverlay = true;
    ArucoTracker tracker(cfg);
    if (!tracker.open()) { fprintf(stderr, "Could not open camera.\n"); return 1; }
    printf("Camera: %dx%d\n", tracker.frameSize().width, tracker.frameSize().height);

    const char* WIN = "Circle Speed Test";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
    cv::setMouseCallback(WIN, onMouse, nullptr);

    bool hasH = doCalib ? runCalibration(tracker, WIN) : tracker.loadHomography(HOMOGRAPHY_FILE);
    if (hasH) printf("Loaded homography — measuring in real mm/s.\n");
    else      printf("No homography loaded — run with --calibrate first; runs are disabled until then.\n");
    g_hasH = hasH;
    if (hasH) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        cv::Mat H;
        if (fs.isOpened()) fs["H"] >> H;
        if (!H.empty()) setH(H);
    }

    CircleState circle;
    loadCircle(circle);  // no-op if absent — starts with centreSet=false
    if (circle.orbitSpeed != 0.f) dirSign = circle.orbitSpeed >= 0.f ? 1.f : -1.f;

    std::vector<RunResult> log;
    std::deque<int>        sweepQueue;
    bool sweeping = false;

    Phase     phase     = Phase::Idle;
    double    seekStartS = 0.0;
    RunState  run;
    RateEstimator yawRateEst;
    float prevTurn = 0.f;

    bool havePrevPose = false;
    cv::Point2f prevPos; double prevPoseS = 0.0;
    float filteredMms = 0.f;

    int8_t motors[MAX_ROBOTS][2] = {};

    auto t0 = std::chrono::steady_clock::now();
    auto lastSend = t0, lastRetry = t0 - std::chrono::seconds(10), lastFrame = t0, lastCtrl = t0;
    auto elapsedS = [&](std::chrono::steady_clock::time_point t) { return std::chrono::duration<float>(t - t0).count(); };

    printf("\nLeft-click = circle centre   +/- = radius   d = flip direction\n"
           "0-9 + Enter = speed   space/g = run   s = stop   n/p = robot\n"
           "P = run full sweep   w = save log now   l = clear log   c = calib   q = quit\n\n");

    bool plotArmed = plotAtStart;  // fires once, on the first frame the circle+robot are ready

    while (g_running) {
        if (!tracker.update()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
        auto now = std::chrono::steady_clock::now();
        float frameDt = std::chrono::duration<float>(now - lastFrame).count();
        lastFrame = now;
        float controlDt = clampf(std::chrono::duration<float>(now - lastCtrl).count(), 0.001f, 0.2f);
        lastCtrl = now;
        double nowS = elapsedS(now);

        if (!swarm.isConnected() && std::chrono::duration<float>(now - lastRetry).count() >= 2.f) {
            lastRetry = now;
            if (swarm.connect()) printf("[hub] Connected.\n");
        }
        swarm.poll();

        if (g_leftClick && g_hasH) {
            g_leftClick = false;
            circle.centre = pixelToWorld(cv::Point2f((float)g_clickPt.x, (float)g_clickPt.y));
            circle.centreSet = true;
            printf("Circle centre: (%.0f, %.0f)  radius: %.0f mm\n", circle.centre.x, circle.centre.y, circle.radius);
            saveCircle(circle);
        }

        std::vector<int> visible;
        for (auto& r : tracker.robots()) visible.push_back(r.id);
        std::sort(visible.begin(), visible.end());
        if (selectedId < 0 && !visible.empty()) selectedId = visible.front();

        const RobotPose* pose = nullptr;
        for (auto& r : tracker.robots()) if (r.id == selectedId) { pose = &r; break; }

        // Kick off an armed --plot sweep as soon as we have what it needs.
        if (plotArmed && hasH && circle.centreSet && pose && phase == Phase::Idle && sweepQueue.empty()) {
            plotArmed = false;
            sweepQueue.clear();
            for (int s = plotMin; s <= plotMax; s += std::max(1, plotStep)) sweepQueue.push_back(clampf((float)s,1.f,100.f));
            if (!sweepQueue.empty()) {
                sweeping = true;
                targetSpeed = sweepQueue.front(); sweepQueue.pop_front();
                phase = Phase::Seeking; seekStartS = nowS;
                printf("[sweep] starting: %d points from %d to %d step %d (seeking onto ring first)\n",
                       (int)sweepQueue.size() + 1, plotMin, plotMax, plotStep);
            }
        }

        // ── Speed measurement (position delta, EMA-filtered) ────────────────
        if (pose && hasH) {
            cv::Point2f cur{pose->x, pose->y};
            if (havePrevPose) {
                float ddt = (float)(nowS - prevPoseS);
                if (ddt > 1e-3f) {
                    float d = std::hypot(cur.x - prevPos.x, cur.y - prevPos.y);
                    float inst = d / ddt;
                    float alpha = frameDt / (SPEED_TAU_S + frameDt);
                    filteredMms += alpha * (inst - filteredMms);
                    if (phase == Phase::Running && (nowS - run.startS) >= WARMUP_FRAC * runTimeS) {
                        run.sumMms += filteredMms;
                        run.nSamples++;
                        run.maxMms = std::max(run.maxMms, filteredMms);
                    }
                }
            } else havePrevPose = true;
            prevPos = cur; prevPoseS = nowS;
        } else { havePrevPose = false; filteredMms = 0.f; }

        // Seeking has no orbit measurement clock running, so it needs its own
        // watchdog: abort back to idle if the robot never reaches the ring
        // (bad radius, stuck robot, camera dropout) rather than seeking forever.
        if (phase == Phase::Seeking) {
            bool timedOut  = (nowS - seekStartS) >= SEEK_TIMEOUT_S;
            bool lostRobot = (pose == nullptr);
            if (timedOut || lostRobot) {
                printf("[run] seek aborted (%s) — robot never reached the ring.\n",
                       lostRobot ? "robot lost" : "timed out");
                phase = Phase::Idle;
                sweeping = false;
                sweepQueue.clear();
            }
        }

        // ── Orbit control for the selected robot ─────────────────────────────
        // Seeking drives with vTan = 0 (radial-only: onto the ring, no orbiting)
        // until within SEEK_ARRIVAL_MM, then Running takes over with the real
        // target speed — see SEEK_ARRIVAL_MM above.
        memset(motors, 0, sizeof(motors));
        if (phase != Phase::Idle && pose && circle.centreSet && selectedId >= 0 && selectedId < MAX_ROBOTS) {
            float dx_c = pose->x - circle.centre.x, dy_c = pose->y - circle.centre.y;
            float distC = std::hypot(dx_c, dy_c);
            if (distC >= 1.f) {
                float rx = dx_c / distC, ry = dy_c / distC;
                float tx = (dirSign >= 0.f) ? -ry :  ry;
                float ty = (dirSign >= 0.f) ?  rx : -rx;

                if (phase == Phase::Seeking && fabsf(distC - circle.radius) <= SEEK_ARRIVAL_MM) {
                    phase = Phase::Running;
                    run = RunState{};
                    run.startS = nowS;
                    printf("[run] on ring — starting cmd=%d on robot %d\n", targetSpeed, selectedId);
                }

                float vTan = (phase == Phase::Running) ? clampf((float)targetSpeed, 0.f, MAX_SPEED) : 0.f;
                float vRad = clampf(-K_RAD * (distC - circle.radius), -MAX_SPEED * 0.5f, MAX_SPEED * 0.5f);
                float vx = vTan * tx + vRad * rx, vy = vTan * ty + vRad * ry;
                float vMag = std::hypot(vx, vy);

                if (vMag >= 0.5f) {
                    float desHeading = atan2f(vy, vx) * 180.f / (float)M_PI;
                    float angleErr   = normAngle(desHeading - pose->yaw);
                    float headingN   = clampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
                    float headingSc  = 1.f - headingN * headingN;

                    float ffOmegaDeg = dirSign * (vTan / circle.radius) * (180.f / (float)M_PI);
                    float turnFF     = K_FF_YAW * ffOmegaDeg;
                    float yawRate    = updateRate(yawRateEst, pose->yaw, now, D_TERM_WINDOW_S);
                    float dAngleErr  = clampf(ffOmegaDeg - yawRate, -300.f, 300.f);

                    float forward  = clampf(vMag, 0.f, MAX_SPEED) * headingSc;
                    float turnTgt  = clampf(turnFF * headingSc + K_ANGLE * angleErr + K_YAW_D * dAngleErr, -MAX_TURN, MAX_TURN);
                    float maxStep  = MAX_TURN_RATE * controlDt;
                    float turn     = clampf(turnTgt, prevTurn - maxStep, prevTurn + maxStep);
                    prevTurn = turn;

                    motors[selectedId][0] = (int8_t)clampf(forward + turn, -100, 100);
                    motors[selectedId][1] = (int8_t)clampf(forward - turn, -100, 100);
                }
            }
        }

        // ── End a run on timeout or the robot going out of view ─────────────
        if (phase == Phase::Running) {
            bool timedOut  = (nowS - run.startS) >= runTimeS;
            bool lostRobot = (pose == nullptr);
            if (timedOut || lostRobot) {
                prevTurn = 0.f;
                yawRateEst = RateEstimator{};
                if (run.nSamples > 0) {
                    float avg = run.sumMms / run.nSamples;
                    RunResult r{targetSpeed, avg, run.maxMms, run.nSamples};
                    log.push_back(r);
                    printf("[run] cmd=%3d  avg=%.1f mm/s  max=%.1f mm/s%s\n",
                           r.speedCmd, r.avgMms, r.maxMms, lostRobot ? "  (robot lost — cut short)" : "");
                } else {
                    printf("[run] cmd=%3d  no samples captured%s\n", targetSpeed,
                           lostRobot ? " (robot never visible)" : "");
                }

                if (sweeping && !sweepQueue.empty()) {
                    // Already orbiting on the ring — go straight to the next
                    // speed rather than re-seeking between sweep points.
                    targetSpeed = sweepQueue.front(); sweepQueue.pop_front();
                    run = RunState{}; run.startS = nowS;
                    printf("[sweep] next cmd=%d (%zu remaining)\n", targetSpeed, sweepQueue.size());
                } else {
                    phase = Phase::Idle;
                    if (sweeping) { sweeping = false; saveResults(log, outDir); }
                }
            }
        }

        if (std::chrono::duration<float>(now - lastSend).count() >= CONTROL_INTERVAL_S) {
            for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed((uint8_t)id, motors[id][0], motors[id][1]);
            swarm.flush();
            lastSend = now;
        }

        // ── Draw ─────────────────────────────────────────────────────────────
        cv::Mat disp = tracker.debugFrame().clone();

        if (g_hasH) {
            cv::Point2f centrePx = worldToPixel(circle.centre);
            float radiusPx = circle.radius;
            cv::Point2f edgePx = worldToPixel({circle.centre.x + circle.radius, circle.centre.y});
            radiusPx = (float)cv::norm(edgePx - centrePx);
            cv::Scalar circleCol = circle.centreSet ? cv::Scalar(0, 200, 255) : cv::Scalar(80, 80, 80);
            cv::circle(disp, centrePx, (int)radiusPx, circleCol, 2, cv::LINE_AA);
            cv::drawMarker(disp, centrePx, circleCol, cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
        }
        if (pose) {
            cv::Scalar col = phase == Phase::Running ? cv::Scalar(0,140,255)
                            : phase == Phase::Seeking ? cv::Scalar(0,220,0)
                                                       : cv::Scalar(0,255,255);
            cv::circle(disp, {(int)pose->px, (int)pose->py}, 22, col, 2, cv::LINE_AA);
        }

        ArucoTracker::drawText(disp, DemoHud::fmt("%.0f mm/s", filteredMms), {20, 60}, 40,
            phase == Phase::Running ? cv::Scalar(0,140,255)
          : phase == Phase::Seeking ? cv::Scalar(0,220,0)
                                     : cv::Scalar(0,255,180));

        DemoHud hud;
        std::string typing = typedSpeed >= 0 ? DemoHud::fmt(" (typing: %d)", typedSpeed) : "";
        hud.title(DemoHud::fmt("CIRCLE SPEED TEST  robot:%s  H:%s  circle:%s  HUB:%s",
                                selectedId >= 0 ? std::to_string(selectedId).c_str() : "-",
                                hasH ? "ok" : "MISSING",
                                circle.centreSet ? "set" : "unset",
                                swarm.isConnected() ? "OK" : "OFFLINE"),
                  (hasH && circle.centreSet) ? DemoHud::COL_TEXT : DemoHud::COL_BAD);
        hud.row("Target speed", DemoHud::fmt("%d%s", targetSpeed, typing.c_str()),
                phase == Phase::Running ? DemoHud::COL_WARN : DemoHud::COL_TEXT);
        hud.row("Circle", DemoHud::fmt("r=%.0fmm  dir=%s", circle.radius, dirSign >= 0.f ? "ccw" : "cw"));
        std::string stateStr;
        if (phase == Phase::Running)
            stateStr = DemoHud::fmt("RUNNING  %.1fs / %.1fs%s", nowS - run.startS, runTimeS,
                sweeping ? DemoHud::fmt("  [sweep, %zu left]", sweepQueue.size()).c_str() : "");
        else if (phase == Phase::Seeking)
            stateStr = DemoHud::fmt("SEEKING onto ring  %.1fs / %.1fs", nowS - seekStartS, SEEK_TIMEOUT_S);
        else
            stateStr = "idle";
        hud.row("State", stateStr, phase == Phase::Seeking ? DemoHud::COL_WARN : DemoHud::COL_TEXT);
        hud.row("Keys", "0-9+Enter=speed  space/g=run  s=stop  n/p=robot  P=sweep  w=save  c=calib  q=quit");
        if (!log.empty()) {
            hud.header({"cmd", "avg mm/s", "max mm/s", "n"});
            int start = std::max(0, (int)log.size() - LOG_MAX_ROWS);
            for (int i = start; i < (int)log.size(); i++) {
                auto& r = log[i];
                hud.row({DemoHud::fmt("%d", r.speedCmd), DemoHud::fmt("%.1f", r.avgMms),
                          DemoHud::fmt("%.1f", r.maxMms), DemoHud::fmt("%d", r.nSamples)});
            }
        }
        hud.drawTopRight(disp);

        cv::imshow(WIN, disp);

        // ── Keys ─────────────────────────────────────────────────────────────
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) g_running = false;
        else if (key >= '0' && key <= '9') {
            typedSpeed = (typedSpeed < 0 ? 0 : typedSpeed) * 10 + (key - '0');
            if (typedSpeed > 999) typedSpeed = 999;
        }
        else if (key == 8 || key == 127) {
            if (typedSpeed >= 0) { typedSpeed /= 10; if (typedSpeed == 0) typedSpeed = -1; }
        }
        else if (key == 13 || key == 10) {
            if (typedSpeed >= 0) targetSpeed = (int)clampf((float)typedSpeed, 1.f, 100.f);
            typedSpeed = -1;
        }
        else if (key == '+' || key == '=') { circle.radius += 25.f; saveCircle(circle); }
        else if (key == '-')               { circle.radius = std::max(50.f, circle.radius - 25.f); saveCircle(circle); }
        else if (key == 'd') { dirSign = -dirSign; printf("Direction: %s\n", dirSign >= 0.f ? "ccw" : "cw"); }
        else if (key == 'n' && !visible.empty()) {
            auto it = std::upper_bound(visible.begin(), visible.end(), selectedId);
            selectedId = (it == visible.end()) ? visible.front() : *it;
        }
        else if (key == 'p' && !visible.empty()) {
            auto it = std::lower_bound(visible.begin(), visible.end(), selectedId);
            selectedId = (it == visible.begin()) ? visible.back() : *std::prev(it);
        }
        else if ((key == ' ' || key == 'g') && phase == Phase::Idle) {
            if (!hasH)               printf("[run] refused: no homography loaded — run with --calibrate first.\n");
            else if (!circle.centreSet) printf("[run] refused: no circle centre set — left-click to set one.\n");
            else if (selectedId < 0 || !pose) printf("[run] refused: no robot in view.\n");
            else {
                phase = Phase::Seeking; seekStartS = nowS;
                printf("[run] seeking onto ring, then cmd=%d on robot %d\n", targetSpeed, selectedId);
            }
        }
        else if (key == 's') {
            phase = Phase::Idle;
            sweeping = false;
            sweepQueue.clear();
            memset(motors, 0, sizeof(motors));
            for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed((uint8_t)id, 0, 0);
            swarm.flush();
        }
        else if (key == 'P' && phase == Phase::Idle) {
            if (!hasH)                  printf("[sweep] refused: no homography loaded.\n");
            else if (!circle.centreSet) printf("[sweep] refused: no circle centre set.\n");
            else if (selectedId < 0 || !pose) printf("[sweep] refused: no robot in view.\n");
            else {
                sweepQueue.clear();
                for (int s = plotMin; s <= plotMax; s += std::max(1, plotStep)) sweepQueue.push_back(clampf((float)s,1.f,100.f));
                if (!sweepQueue.empty()) {
                    sweeping = true;
                    targetSpeed = sweepQueue.front(); sweepQueue.pop_front();
                    phase = Phase::Seeking; seekStartS = nowS;
                    printf("[sweep] starting: %d points from %d to %d step %d (seeking onto ring first)\n",
                           (int)sweepQueue.size() + 1, plotMin, plotMax, plotStep);
                }
            }
        }
        else if (key == 'w') saveResults(log, outDir);
        else if (key == 'l') { log.clear(); printf("[log] cleared.\n"); }
        else if (key == 'c') { hasH = runCalibration(tracker, WIN); g_hasH = hasH; }
    }

    for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed((uint8_t)id, 0, 0);
    swarm.flush();
    if (!log.empty()) printf("[exit] %zu unsaved run(s) in the log — press 'w' next time before quitting to save.\n", log.size());
    return 0;
}
