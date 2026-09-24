// max_speed_test.cpp — Motor-command -> real-world-speed calibration tool.
//
// Commands one robot straight ahead at a chosen motor value (1-100) and
// reads its actual speed live from vision (world mm, via the homography),
// so the motor-command -> mm/s mapping used by car_following.cpp
// (--robot-max-speed) and any other speed-dependent controller can be
// measured on real hardware instead of assumed. See CLAUDE.md's
// "robotMaxMms" calibration note.
//
// Usage: ./max_speed_test [--serial SN] [--ip IP] [--robot ID] [--speed N]
//                          [--run-time S] [--max-distance MM] [--calibrate]
//
// Keys:
//   0-9        build a target speed (1-100) digit by digit
//   Enter      commit the typed speed
//   Backspace  remove the last typed digit
//   +/-        nudge the committed speed by 1      [ / ]  by 10
//   space, g   run: drive the selected robot straight at the committed speed
//   s          stop immediately
//   n/p        select next/previous visible robot
//   l          clear the session log of measured runs
//   c          (re)calibrate the homography (click 4 corners)
//   q, Esc     quit (also stops the robot)
//
// A run needs a loaded/valid homography — RobotPose.x/y (and therefore the
// measured speed) are only in real mm once one is set (see aruco_tracker.h).
// Without one this tool still shows the camera feed but refuses to run.

#include "aruco_tracker.h"
#include "SwarmClient.h"
#include "DemoHud.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <chrono>
#include <thread>

// ── Tunables ──────────────────────────────────────────────────────────────────

static constexpr float SPEED_TAU_S      = 0.15f;  // low-pass on the instantaneous mm/s reading
static constexpr float WARMUP_FRAC      = 0.25f;  // fraction of run-time excluded from the steady-state average (accel phase)
static constexpr int   MAX_ROBOTS       = 32;
static constexpr int   LOG_MAX_ROWS     = 8;

static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";

// ── Globals (signal handling only) ───────────────────────────────────────────

static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

// ── Calibration (click 4 corners, enter arena size) — same flow as
// drag_drop_demo.cpp's runCalibration ────────────────────────────────────────

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
        if (cv::waitKey(30) == 27) { cv::setMouseCallback(win, nullptr, nullptr); return false; }
    }
    cv::setMouseCallback(win, nullptr, nullptr);

    printf("Arena width height mm (e.g. 800 600): ");
    float W = 0, H = 0;
    if (scanf("%f %f", &W, &H) != 2 || W <= 0 || H <= 0) return false;

    std::vector<cv::Point2f> worldPts = {{0,0},{W,0},{W,H},{0,H}};
    cv::Mat H_ = cv::findHomography(cs.pixPts, worldPts);
    if (H_.empty()) return false;
    tracker.setHomography(cs.pixPts, worldPts);
    tracker.saveHomography(HOMOGRAPHY_FILE);
    printf("Saved homography.\n");
    return true;
}

// ── Run state ─────────────────────────────────────────────────────────────────

struct RunResult {
    int   speedCmd;
    float avgMms;
    float maxMms;
    float impliedMaxMms;  // avgMms extrapolated to command 100, assuming linearity
};

struct RunState {
    bool   active   = false;
    double startS   = 0.0;
    float  sumMms   = 0.f;
    int    nSamples = 0;
    float  maxMms   = 0.f;
    float  distMm   = 0.f;
};

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    std::string serial, ip;
    int   robotArg     = -1;
    int   initialSpeed = 50;
    float runTimeS     = 5.0f;
    float maxDistMm    = 500.f;
    bool  doCalib      = false;

    for (int i = 1; i < argc; i++) {
        auto arg = [&](const char* n) { return strcmp(argv[i], n) == 0 && i + 1 < argc; };
        if      (arg("--serial"))       serial       = argv[++i];
        else if (arg("--ip"))           ip           = argv[++i];
        else if (arg("--robot"))        robotArg     = atoi(argv[++i]);
        else if (arg("--speed"))        initialSpeed = atoi(argv[++i]);
        else if (arg("--run-time"))     runTimeS     = (float)atof(argv[++i]);
        else if (arg("--max-distance")) maxDistMm    = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--calibrate")) doCalib = true;
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("usage: %s [--serial SN] [--ip IP] [--robot ID] [--speed N]\n"
                   "       [--run-time S] [--max-distance MM] [--calibrate]\n", argv[0]);
            return 0;
        } else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 2; }
    }

    int targetSpeed = (int)clampf((float)initialSpeed, 1.f, 100.f);
    int typedSpeed  = -1;  // -1 = not currently typing
    int selectedId  = robotArg;

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

    const char* WIN = "Max Speed Test";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);

    bool hasH = doCalib ? runCalibration(tracker, WIN) : tracker.loadHomography(HOMOGRAPHY_FILE);
    if (hasH) printf("Loaded homography — measuring in real mm/s.\n");
    else      printf("No homography loaded — run with --calibrate first; runs are disabled until then.\n");

    std::deque<RunResult> log;
    RunState run;
    bool   havePrevPose = false;
    cv::Point2f prevPos;
    double prevPoseS = 0.0;
    float  filteredMms = 0.f;

    int8_t motors[MAX_ROBOTS][2] = {};

    auto t0 = std::chrono::steady_clock::now();
    auto lastSend = t0, lastRetry = t0 - std::chrono::seconds(10), lastFrame = t0;
    auto elapsedS = [&](std::chrono::steady_clock::time_point t) {
        return std::chrono::duration<float>(t - t0).count();
    };

    printf("\nType digits then Enter to set speed (1-100), space/g to run, s to stop, "
           "n/p to select robot, q to quit.\n\n");

    while (g_running) {
        if (!tracker.update()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastFrame).count();
        lastFrame = now;
        double nowS = elapsedS(now);

        if (!swarm.isConnected() &&
            std::chrono::duration<float>(now - lastRetry).count() >= 2.f) {
            lastRetry = now;
            if (swarm.connect()) printf("[hub] Connected.\n");
        }
        swarm.poll();

        // Visible robots, sorted, for n/p selection.
        std::vector<int> visible;
        for (auto& r : tracker.robots()) visible.push_back(r.id);
        std::sort(visible.begin(), visible.end());
        if (selectedId < 0 && !visible.empty()) selectedId = visible.front();

        const RobotPose* pose = nullptr;
        for (auto& r : tracker.robots()) if (r.id == selectedId) { pose = &r; break; }

        // Speed measurement — only while a pose is tracked continuously frame
        // to frame; a re-acquire after a dropout skips one delta so a gap in
        // detection doesn't read back as a speed spike.
        if (pose && hasH) {
            cv::Point2f cur{pose->x, pose->y};
            if (havePrevPose) {
                float ddt = (float)(nowS - prevPoseS);
                if (ddt > 1e-3f) {
                    float d = std::hypot(cur.x - prevPos.x, cur.y - prevPos.y);
                    float inst = d / ddt;
                    float alpha = dt / (SPEED_TAU_S + dt);
                    filteredMms += alpha * (inst - filteredMms);
                    if (run.active) {
                        run.distMm += d;
                        if (nowS - run.startS >= WARMUP_FRAC * runTimeS) {
                            run.sumMms += filteredMms;
                            run.nSamples++;
                            run.maxMms = std::max(run.maxMms, filteredMms);
                        }
                    }
                }
            } else {
                havePrevPose = true;
            }
            prevPos = cur;
            prevPoseS = nowS;
        } else {
            havePrevPose = false;
            filteredMms = 0.f;
        }

        // End a run on timeout, distance cap, or the robot going out of view.
        if (run.active) {
            bool timedOut  = (nowS - run.startS) >= runTimeS;
            bool tooFar    = run.distMm >= maxDistMm;
            bool lostRobot = (pose == nullptr);
            if (timedOut || tooFar || lostRobot) {
                run.active = false;
                if (run.nSamples > 0) {
                    float avg = run.sumMms / run.nSamples;
                    RunResult r{targetSpeed, avg, run.maxMms, avg / (targetSpeed / 100.f)};
                    log.push_back(r);
                    while ((int)log.size() > LOG_MAX_ROWS) log.pop_front();
                    printf("[run] cmd=%3d  avg=%.1f mm/s  max=%.1f mm/s  implied_max@100=%.1f mm/s%s\n",
                           r.speedCmd, r.avgMms, r.maxMms, r.impliedMaxMms,
                           lostRobot ? "  (robot lost — cut short)" : "");
                } else {
                    printf("[run] cmd=%3d  no samples captured%s\n", targetSpeed,
                           lostRobot ? " (robot never visible)" : "");
                }
            }
        }

        // Motor output.
        memset(motors, 0, sizeof(motors));
        if (run.active && selectedId >= 0 && selectedId < MAX_ROBOTS)
            motors[selectedId][0] = motors[selectedId][1] = (int8_t)targetSpeed;

        if (std::chrono::duration<float>(now - lastSend).count() >= 0.05f) {
            for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed((uint8_t)id, motors[id][0], motors[id][1]);
            swarm.flush();
            lastSend = now;
        }

        // ── Draw ─────────────────────────────────────────────────────────────
        cv::Mat disp = tracker.debugFrame().clone();

        if (pose) {
            cv::Scalar col = run.active ? cv::Scalar(0,140,255) : cv::Scalar(0,255,255);
            cv::circle(disp, {(int)pose->px, (int)pose->py}, 22, col, 2, cv::LINE_AA);
        }

        char big[64];
        snprintf(big, sizeof(big), "%.0f mm/s", filteredMms);
        ArucoTracker::drawText(disp, big, {20, 60}, 40,
            run.active ? cv::Scalar(0,140,255) : cv::Scalar(0,255,180));

        DemoHud hud;
        std::string typing = typedSpeed >= 0 ? DemoHud::fmt(" (typing: %d)", typedSpeed) : "";
        hud.title(DemoHud::fmt("MAX SPEED TEST  robot:%s  H:%s  HUB:%s",
                                selectedId >= 0 ? std::to_string(selectedId).c_str() : "-",
                                hasH ? "ok" : "MISSING",
                                swarm.isConnected() ? "OK" : "OFFLINE"),
                  hasH ? DemoHud::COL_TEXT : DemoHud::COL_BAD);
        hud.row("Target speed", DemoHud::fmt("%d%s", targetSpeed, typing.c_str()),
                run.active ? DemoHud::COL_WARN : DemoHud::COL_TEXT);
        hud.row("State", run.active ? DemoHud::fmt("RUNNING  %.1fs / %.1fs  %.0f/%.0f mm",
                                                     nowS - run.startS, runTimeS, run.distMm, maxDistMm)
                                     : "idle");
        hud.row("Keys", "0-9+Enter=speed  space/g=run  s=stop  n/p=robot  c=calib  l=clear log  q=quit");
        if (!log.empty()) {
            hud.header({"cmd", "avg mm/s", "max mm/s", "implied@100"});
            for (auto& r : log)
                hud.row({DemoHud::fmt("%d", r.speedCmd), DemoHud::fmt("%.1f", r.avgMms),
                          DemoHud::fmt("%.1f", r.maxMms), DemoHud::fmt("%.1f", r.impliedMaxMms)});
        }
        hud.drawTopRight(disp);

        cv::imshow(WIN, disp);

        // ── Keys ─────────────────────────────────────────────────────────────
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) g_running = false;
        else if (key >= '0' && key <= '9') {
            typedSpeed = (typedSpeed < 0 ? 0 : typedSpeed) * 10 + (key - '0');
            if (typedSpeed > 999) typedSpeed = 999;  // avoid runaway before Enter clamps it
        }
        else if (key == 8 || key == 127) {  // backspace
            if (typedSpeed >= 0) { typedSpeed /= 10; if (typedSpeed == 0) typedSpeed = -1; }
        }
        else if (key == 13 || key == 10) {  // enter
            if (typedSpeed >= 0) targetSpeed = (int)clampf((float)typedSpeed, 1.f, 100.f);
            typedSpeed = -1;
        }
        else if (key == '+' || key == '=') targetSpeed = (int)clampf((float)targetSpeed + 1, 1.f, 100.f);
        else if (key == '-')               targetSpeed = (int)clampf((float)targetSpeed - 1, 1.f, 100.f);
        else if (key == ']')               targetSpeed = (int)clampf((float)targetSpeed + 10, 1.f, 100.f);
        else if (key == '[')               targetSpeed = (int)clampf((float)targetSpeed - 10, 1.f, 100.f);
        else if (key == 'n' && !visible.empty()) {
            auto it = std::upper_bound(visible.begin(), visible.end(), selectedId);
            selectedId = (it == visible.end()) ? visible.front() : *it;
        }
        else if (key == 'p' && !visible.empty()) {
            auto it = std::lower_bound(visible.begin(), visible.end(), selectedId);
            selectedId = (it == visible.begin()) ? visible.back() : *std::prev(it);
        }
        else if ((key == ' ' || key == 'g') && !run.active) {
            if (!hasH) {
                printf("[run] refused: no homography loaded — run with --calibrate first.\n");
            } else if (selectedId < 0 || !pose) {
                printf("[run] refused: no robot in view.\n");
            } else {
                run = RunState{};
                run.active = true;
                run.startS = nowS;
                printf("[run] starting cmd=%d on robot %d\n", targetSpeed, selectedId);
            }
        }
        else if (key == 's') {
            run.active = false;
            memset(motors, 0, sizeof(motors));
            for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed((uint8_t)id, 0, 0);
            swarm.flush();
        }
        else if (key == 'l') { log.clear(); printf("[log] cleared.\n"); }
        else if (key == 'c') { hasH = runCalibration(tracker, WIN); }
    }

    for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed((uint8_t)id, 0, 0);
    swarm.flush();
    return 0;
}
