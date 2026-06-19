// vision_controller.cpp — Vision-based swarm controller
// Usage: ./vision_controller [--serial SN] [--ip IP] [--calibrate]
// Controls: left-click = goal all, right-click = goal selected, 0-9 select,
//           s stop (unlock leader), c calibrate, +/- speed, WASD drive leader,
//           q/Esc quit

#ifdef __APPLE__
#include <CoreGraphics/CoreGraphics.h>
#else
#include "evdev_keys.h"
#endif

#include "aruco_tracker.h"
#include "SwarmClient.h"

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
#include <atomic>
#include <mutex>

#ifdef __APPLE__
using KeyHandle = CGKeyCode;
static constexpr KeyHandle kKey_A = 0x00;
static constexpr KeyHandle kKey_S = 0x01;
static constexpr KeyHandle kKey_D = 0x02;
static constexpr KeyHandle kKey_W = 0x0D;
static inline bool keyDown(KeyHandle code) {
    return CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, code);
}
#else
using KeyHandle = int;
static constexpr KeyHandle kKey_A = KEY_A;
static constexpr KeyHandle kKey_S = KEY_S;
static constexpr KeyHandle kKey_D = KEY_D;
static constexpr KeyHandle kKey_W = KEY_W;
static EvdevKeyboard g_keyboard;
static inline bool keyDown(KeyHandle code) {
    return g_keyboard.down(code);
}
#endif

static int8_t g_wasdL = 0;
static int8_t g_wasdR = 0;
static bool    g_wasdActive = false;

// WASD thread state (thread writes, main loop reads)
static std::atomic<int>    g_leaderIdAtomic{-1};
static std::atomic<bool>   g_wasdActive_a{false};
static std::atomic<int8_t> g_wasdL_a{0};
static std::atomic<int8_t> g_wasdR_a{0};
static std::atomic<bool>   g_keyW_a{false}, g_keyA_a{false}, g_keyS_a{false}, g_keyD_a{false};
static constexpr int8_t WASD_THROTTLE = 30;
static constexpr int8_t WASD_STEER    = 15;

// ── Hub connection ────────────────────────────────────────────────────────────

static constexpr int MAX_ROBOTS = SC_MAX_ROBOTS;
static volatile bool g_running = true;
static std::unordered_map<int, std::chrono::steady_clock::time_point> g_robotLastSeen;
static void onSignal(int) { g_running = false; }

// SwarmClient instance shared by the main loop and the WASD thread; all
// access (connect/setSpeed/registerRobot/flush) must hold g_swarmMutex.
static SwarmClient g_swarm;
static std::mutex  g_swarmMutex;

static bool tryHub() {
    std::lock_guard<std::mutex> lk(g_swarmMutex);
    return g_swarm.connect();
}

static void sendSwarm(int8_t motors[MAX_ROBOTS][2]) {
    std::lock_guard<std::mutex> lk(g_swarmMutex);
    for (int i = 0; i < MAX_ROBOTS; i++) g_swarm.setSpeed((uint8_t)i, motors[i][0], motors[i][1]);
    g_swarm.flush();
}

static void sendLeaderWasd(int lid, int8_t L, int8_t R) {
    std::lock_guard<std::mutex> lk(g_swarmMutex);
    g_swarm.setSpeed((uint8_t)lid, L, R);
    g_swarm.flush();
}

// ── Controller ───────────────────────────────────────────────────────────────

static constexpr float K_DIST         = 0.40f;
static constexpr float K_ANGLE        = 0.22f;   // heading P-gain
static constexpr float K_YAW_D        = 0.08f;   // heading D-gain: dampens oscillation
static constexpr float MAX_SPEED      = 51.7f;   // +10% from 47
static constexpr float MAX_TURN       = 16.0f;
static constexpr float ARRIVAL_MM     = 75.0f;
static constexpr float SEND_INTERVALS = 0.05f;
// Yaw EMA: lower α = smoother but more lag; higher α = faster tracking, less overshoot.
// 0.08, not the 0.25 this was tuned at originally: that value matched a
// ~30fps loop, but the 2026-06-15 sleep_for(50ms->1ms) fix (55e7d938) lets
// this loop run at the camera's ~115fps now, so the old alpha let raw
// per-frame ArUco corner-angle noise straight through into the D-term. See
// circle_demo.cpp's YAW_ALPHA comment for the full derivation.
static constexpr float YAW_ALPHA      = 0.08f;

static float normAngle(float a) {
    while (a >  180) a -= 360;
    while (a < -180) a += 360;
    return a;
}
static float clampf(float v, float lo, float hi) { return v<lo?lo:v>hi?hi:v; }

// ── Mouse / targeting ────────────────────────────────────────────────────────

struct Target { float x, y; bool set = false; };

static std::unordered_map<int, Target> g_targets;
static int  g_selectedRobot = -1;
static int  g_speedLevel = 45; // 0..255, 255 => 4x speed; default -30% from 64
static bool g_haveGlobalTarget = false;
static Target g_globalTarget = {0,0,false};
static bool g_leftClick = false, g_rightClick = false;
static cv::Point g_clickPt;

static void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) { g_leftClick  = true; g_clickPt = {x,y}; }
    if (event == cv::EVENT_RBUTTONDOWN) { g_rightClick = true; g_clickPt = {x,y}; }
}

// ── Calibration ──────────────────────────────────────────────────────────────

static cv::Mat  g_H;
static bool     g_hasH = false;
static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";

static cv::Point2f pixelToWorld(cv::Point2f px) {
    if (!g_hasH) return px;
    std::vector<cv::Point2f> src = {px}, dst;
    cv::perspectiveTransform(src, dst, g_H);
    return dst[0];
}

struct CalibState { std::vector<cv::Point2f> pixPts; bool done = false; };

static void onCalibMouse(int event, int x, int y, int, void* ud) {
    auto* s = (CalibState*)ud;
    if (event == cv::EVENT_LBUTTONDOWN && s->pixPts.size() < 4) {
        s->pixPts.push_back({(float)x,(float)y});
        printf("  corner %d: (%d, %d)\n", (int)s->pixPts.size(), x, y);
        if (s->pixPts.size() == 4) s->done = true;
    }
}

static bool runCalibration(ArucoTracker& tracker) {
    printf("\nCalibration: click 4 arena corners (TL TR BR BL)\n");
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
        for (auto& pt : cs.pixPts) cv::circle(disp, pt, 8, {0,0,255}, -1);
        tracker.drawText(disp,
            "Click corners TL TR BR BL  " + std::to_string(cs.pixPts.size()) + "/4",
            {10,40}, 20, {0,255,0});
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

// ── WASD thread — isolated 2 ms loop, zero-latency direct send ───────────────

static void wasdControlLoop() {
#ifndef __APPLE__
    if (g_keyboard.open() == 0) {
        fprintf(stderr, "[wasd] no readable keyboard in /dev/input — WASD disabled. "
                        "Add yourself to the 'input' group (sudo usermod -aG input $USER, "
                        "then log out and back in).\n");
        return;
    }
#endif
    int8_t lastL = 0, lastR = 0;
    while (g_running) {
        bool w = keyDown(kKey_W);
        bool a = keyDown(kKey_A);
        bool s = keyDown(kKey_S);
        bool d = keyDown(kKey_D);

        g_keyW_a.store(w, std::memory_order_relaxed);
        g_keyA_a.store(a, std::memory_order_relaxed);
        g_keyS_a.store(s, std::memory_order_relaxed);
        g_keyD_a.store(d, std::memory_order_relaxed);

        bool active = w || a || s || d;
        g_wasdActive_a.store(active, std::memory_order_relaxed);

        int8_t newL = 0, newR = 0;
        if (active) {
            int throttle = (w && !s) ? WASD_THROTTLE : (!w && s) ? -WASD_THROTTLE : 0;
            int steer    = (d && !a) ? WASD_STEER    : (!d && a) ? -WASD_STEER    : 0;
            int L = throttle + steer;
            int R = throttle - steer;
            if (L >  100) L =  100; if (L < -100) L = -100;
            if (R >  100) R =  100; if (R < -100) R = -100;
            newL = (int8_t)L;
            newR = (int8_t)R;
        }

        g_wasdL_a.store(newL, std::memory_order_relaxed);
        g_wasdR_a.store(newR, std::memory_order_relaxed);

        if (newL != lastL || newR != lastR) {
            lastL = newL;
            lastR = newR;
            int lid = g_leaderIdAtomic.load(std::memory_order_relaxed);
            if (lid >= 0) sendLeaderWasd(lid, newL, newR);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
#ifndef __APPLE__
    g_keyboard.close();
#endif
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    bool        doCalibrate = false;
    std::string serial, ip;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--serial")    && i+1<argc) serial      = argv[++i];
        if (!strcmp(argv[i], "--ip")        && i+1<argc) ip          = argv[++i];
        if (!strcmp(argv[i], "--calibrate"))             doCalibrate = true;
    }

    if (tryHub()) printf("[hub] Connected.\n");
    else          printf("[hub] Not available — will retry every 2s. "
                         "Start manually: ./swarm_hub /dev/tty.usbmodem*\n");

    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    ArucoTracker tracker(cfg);
    if (!tracker.open()) { fprintf(stderr, "Could not open Basler camera.\n"); return 1; }
    // auto undist = std::make_unique<FisheyeUndistortPreprocessor>();
    // if (undist->load("fisheye_calib.yaml", tracker.frameSize())) tracker.prependPreprocessor(std::move(undist));
    printf("Camera open at %dx%d.\n", tracker.frameSize().width, tracker.frameSize().height);

    if (!doCalibrate && tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
        printf("Loaded homography from %s\n", HOMOGRAPHY_FILE);
    }
    if (doCalibrate && !runCalibration(tracker)) printf("Calibration skipped.\n");

    const char* WIN = "Vision Controller";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
    cv::setMouseCallback(WIN, onMouse, nullptr);

    int8_t motors[MAX_ROBOTS][2] = {};
    static int8_t lastMotors[MAX_ROBOTS][2] = {};
    auto   lastSend     = std::chrono::steady_clock::now();
    auto   lastFpsT     = lastSend;
    auto   lastControlT = lastSend;
    auto   lastHubRetry = lastSend - std::chrono::seconds(10);
    std::unordered_map<int, float> smoothedYaw;
    std::unordered_map<int, float> prevAngleErr;
    int    frameCount   = 0;
    float  fps          = 0;

    printf("\nLeft-click=goal all  Right-click=goal selected  0-9=select  s=stop  c=calibrate  q=quit\n\n");

    std::thread wasdThread(wasdControlLoop);

    while (g_running) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        ++frameCount;
        float controlDt = clampf(std::chrono::duration<float>(now - lastControlT).count(), 0.01f, 0.2f);
        lastControlT = now;

        if (!g_swarm.isConnected() && std::chrono::duration<float>(now-lastHubRetry).count() >= 2.0f) {
            lastHubRetry = now;
            if (tryHub()) printf("[hub] Connected.\n");
        }

        float dt = std::chrono::duration<float>(now - lastFpsT).count();
        if (dt >= 1.0f) { fps = frameCount / dt; frameCount = 0; lastFpsT = now; }


        if (g_leftClick || g_rightClick) {
            cv::Point2f world = pixelToWorld({(float)g_clickPt.x, (float)g_clickPt.y});
            g_haveGlobalTarget = true;
            g_globalTarget = {world.x, world.y, true};

            if (g_leftClick || g_selectedRobot < 0) {
                for (auto& r : tracker.robots()) {
                    if (r.id < 0 || r.id >= MAX_ROBOTS) continue;
                    g_targets[r.id] = {world.x, world.y, true};
                }
                printf("Target all: (%.0f, %.0f)\n", world.x, world.y);
            } else {
                if (g_selectedRobot >= 0 && g_selectedRobot < MAX_ROBOTS) {
                    g_targets[g_selectedRobot] = {world.x, world.y, true};
                    printf("Target robot %d: (%.0f, %.0f)\n", g_selectedRobot, world.x, world.y);
                } else {
                    printf("Selected robot %d out of range. Ignored.\n", g_selectedRobot);
                }
            }
            g_leftClick = g_rightClick = false;
        }

        std::unordered_map<int, Target> followTargets;
        std::vector<int> robotIds;
        std::unordered_map<int, RobotPose> poseById;
        for (auto& r : tracker.robots()) {
            if (r.id < 0 || r.id >= MAX_ROBOTS) continue;
            robotIds.push_back(r.id);
            poseById[r.id] = r;
            g_robotLastSeen[r.id] = now;   // feed the watchdog
            { std::lock_guard<std::mutex> lk(g_swarmMutex); g_swarm.registerRobot((uint8_t)r.id); }
        }
        std::sort(robotIds.begin(), robotIds.end());

        // EMA-smooth yaw to reduce heading noise and lag-induced overshoot.
        for (auto& [id, pose] : poseById) {
            auto [it, fresh] = smoothedYaw.emplace(id, pose.yaw);
            if (!fresh) {
                float delta = normAngle(pose.yaw - it->second);
                it->second = normAngle(it->second + YAW_ALPHA * delta);
            }
            pose.yaw = it->second;
        }

        // Lock leader ID on first detection; never re-elect while a leader is set.
        // Resets when the user presses 's' (handled below).
        static int g_lockedLeaderId = -1;
        if (g_lockedLeaderId == -1 && !robotIds.empty()) {
            g_lockedLeaderId = robotIds[0];
            printf("[leader] Locked to robot %d\n", g_lockedLeaderId);
        }
        int leaderId = g_lockedLeaderId;

        // WASD is driven by the dedicated thread — just read the shared state here.
        g_leaderIdAtomic.store(leaderId, std::memory_order_relaxed);
        g_wasdActive = g_wasdActive_a.load(std::memory_order_relaxed);
        g_wasdL      = g_wasdL_a.load(std::memory_order_relaxed);
        g_wasdR      = g_wasdR_a.load(std::memory_order_relaxed);

        if (g_wasdActive) {
            g_haveGlobalTarget = false;
            if (leaderId >= 0) g_targets.erase(leaderId);
        }

        // Chain following: each robot targets the predecessor's *position* directly.
        // Stops at FOLLOW_DISTANCE — that gap becomes the natural formation spacing.
        // Chain: robotIds sorted ascending; each follows the next lower ID in the
        // visible set (robotIds[i] follows robotIds[i-1]).
        const float FOLLOW_DISTANCE = 110.0f;

        // Only generate follow targets when the locked leader is currently visible.
        // If the leader drops out, followers get no target and stop safely.
        if (g_haveGlobalTarget && leaderId >= 0 && poseById.count(leaderId)) {
            if (!g_wasdActive) {
                followTargets[leaderId] = g_globalTarget;
            }
            for (size_t i = 1; i < robotIds.size(); ++i) {
                int id     = robotIds[i];
                int prevId = robotIds[i - 1];
                if (!poseById.count(prevId)) break;  // chain broken — robots further back stop
                followTargets[id] = {poseById[prevId].x, poseById[prevId].y, true};
            }
        }

        float speedScale = (float)g_speedLevel / 255.0f * 4.0f;
        float baseSpeed  = MAX_SPEED * speedScale;

        // Detect WASD release: zero leader motors so the periodic sendSwarm
        // doesn't fight the WASD thread's immediate stop command.
        static bool prevWasdActive = false;
        if (prevWasdActive && !g_wasdActive && leaderId >= 0) {
            lastMotors[leaderId][0] = 0;
            lastMotors[leaderId][1] = 0;
        }
        prevWasdActive = g_wasdActive;

        memcpy(motors, lastMotors, sizeof(motors));
        for (auto& r : tracker.robots()) {
            if (r.id < 0 || r.id >= MAX_ROBOTS) continue;

            if (g_wasdActive && r.id == leaderId) {
                motors[r.id][0] = g_wasdL;
                motors[r.id][1] = g_wasdR;
                continue;
            }

            Target tgt;
            bool hasTgt = false;
            if (followTargets.count(r.id)) {
                tgt = followTargets[r.id];
                hasTgt = true;
            } else if (g_targets.count(r.id) && g_targets[r.id].set) {
                tgt = g_targets[r.id];
                hasTgt = true;
            }
            // No target → explicit stop so lastMotors doesn't hold stale values.
            if (!hasTgt) {
                motors[r.id][0] = 0;
                motors[r.id][1] = 0;
                continue;
            }

            float dx   = tgt.x - r.x;
            float dy   = tgt.y - r.y;
            float dist = sqrtf(dx * dx + dy * dy);

            bool  isFollower = (r.id != leaderId);
            float stopDist   = isFollower ? FOLLOW_DISTANCE : ARRIVAL_MM;

            if (dist < stopDist) {
                motors[r.id][0] = 0;
                motors[r.id][1] = 0;
                continue;
            }

            // Follower-specific gains: gentler P, lower top speed, softer turn cap.
            // maxSpd scales with g_speedLevel so +/- affects the whole swarm.
            static constexpr float FOLLOWER_SPD_FRAC = 34.0f / MAX_SPEED;  // ~72% of leader
            float kDist   = isFollower ? K_DIST * 0.40f : K_DIST;
            float maxSpd  = isFollower ? baseSpeed * FOLLOWER_SPD_FRAC : baseSpeed;
            float maxTurn = isFollower ? 15.0f : MAX_TURN;

            float yaw       = poseById.count(r.id) ? poseById[r.id].yaw : r.yaw;
            float tgtAngle  = (float)(std::atan2(dy, dx) * 180.0 / M_PI);
            float angleErr  = normAngle(tgtAngle - yaw);
            float headingNorm  = clampf(fabsf(angleErr) / 90.0f, 0.0f, 1.0f);
            float headingScale = 1.0f - headingNorm * headingNorm;  // inverse quadratic

            // Braking ramp for both leader and followers: speed decays linearly
            // to zero over a window equal to stopDist above the threshold.
            // Prevents the P-controller from arriving at speed and overshooting.
            float brakingScale = clampf((dist - stopDist) / stopDist, 0.0f, 1.0f);

            float dAngleErr = 0.f;
            {
                auto it = prevAngleErr.find(r.id);
                if (it != prevAngleErr.end())
                    dAngleErr = clampf(normAngle(angleErr - it->second) / controlDt,
                                       -300.f, 300.f);
                prevAngleErr[r.id] = angleErr;
            }

            float forward = clampf(kDist * dist, 0.0f, maxSpd) * headingScale * brakingScale;
            float turn    = clampf(K_ANGLE * angleErr + K_YAW_D * dAngleErr, -maxTurn, maxTurn);
            float rawL = forward + turn;
            float rawR = forward - turn;
            motors[r.id][0] = (int8_t)clampf(rawL, -100, 100);
            motors[r.id][1] = (int8_t)clampf(rawR, -100, 100);
        }

        // Watchdog: stop any robot that has been invisible for >1000 ms.
        // Also clears lastMotors so the stop is not overwritten on the next frame.
        for (int id = 0; id < MAX_ROBOTS; ++id) {
            if (poseById.count(id)) continue;   // robot is visible — nothing to do
            auto it = g_robotLastSeen.find(id);
            if (it == g_robotLastSeen.end()) continue;  // never seen — ignore
            auto gapMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count();
            if (gapMs > 1000) {
                motors[id][0]     = 0;
                motors[id][1]     = 0;
                lastMotors[id][0] = 0;
                lastMotors[id][1] = 0;
            }
        }

        if (std::chrono::duration<float>(now-lastSend).count() >= SEND_INTERVALS) {
            sendSwarm(motors); lastSend = now;
            memcpy(lastMotors, motors, sizeof(motors));
        }

        cv::Mat disp = tracker.debugFrame().clone();

        for (auto& r : tracker.robots()) {
            if (r.id < 0 || r.id >= MAX_ROBOTS) continue;
            Target tgt;
            bool hasTgt = false;
            if (followTargets.count(r.id)) {
                tgt = followTargets[r.id];
                hasTgt = true;
            } else if (g_targets.count(r.id) && g_targets[r.id].set) {
                tgt = g_targets[r.id];
                hasTgt = true;
            }
            if (!hasTgt) continue;

            cv::Point2f p0(r.px, r.py);
            cv::Point2f p1;
            if (g_hasH) {
                cv::Mat Hinv = g_H.inv();
                std::vector<cv::Point2f> ws = {{tgt.x, tgt.y}};
                std::vector<cv::Point2f> ps;
                cv::perspectiveTransform(ws, ps, Hinv);
                p1 = ps[0];
            } else {
                p1 = cv::Point2f(tgt.x, tgt.y);
            }
            cv::arrowedLine(disp, p0, p1, {255, 128, 0}, 2, cv::LINE_AA, 0, 0.2);
        }

        for (auto& r : tracker.robots()) {
            if (!g_targets.count(r.id) || !g_targets[r.id].set) continue;
            auto& tgt = g_targets[r.id];
            cv::Point2f tPix = g_hasH ? [&]{
                cv::Mat Hinv = g_H.inv();
                std::vector<cv::Point2f> src = {{tgt.x,tgt.y}}, dst;
                cv::perspectiveTransform(src, dst, Hinv);
                return dst[0];
            }() : cv::Point2f{tgt.x, tgt.y};
            cv::drawMarker(disp, tPix, {0,200,255}, cv::MARKER_CROSS, 20, 2);
            cv::line(disp, {(int)r.px,(int)r.py}, tPix, {0,200,255}, 1, cv::LINE_AA);
        }

        if (leaderId >= 0 && poseById.count(leaderId)) {
            auto& L = poseById[leaderId];
            cv::Point leaderPx((int)L.px, (int)L.py);
            cv::circle(disp, leaderPx, 30, {0,255,0}, 2, cv::LINE_AA);
            cv::circle(disp, leaderPx,  8, {0,255,0}, -1, cv::LINE_AA);
            tracker.drawText(disp, cv::format("LEADER %d", leaderId), leaderPx + cv::Point(10,-10), 20, {0,255,0});
            if (g_haveGlobalTarget) {
                cv::Point2f targetPix;
                if (g_hasH) {
                    cv::Mat Hinv = g_H.inv();
                    std::vector<cv::Point2f> src = {{g_globalTarget.x,g_globalTarget.y}}, dst;
                    cv::perspectiveTransform(src, dst, Hinv);
                    targetPix = dst[0];
                } else {
                    targetPix = cv::Point2f{g_globalTarget.x, g_globalTarget.y};
                }
                cv::line(disp, leaderPx, targetPix, {0,255,0}, 1, cv::LINE_AA);
                cv::drawMarker(disp, targetPix, {0,255,0}, cv::MARKER_TILTED_CROSS, 15, 2);
            }
        }

        char hud[160];
        snprintf(hud, sizeof(hud), "loop_fps:%.0f  Robots:%d  Sel:%s  %s  HUB:%s  Speed:%d(%.2fx)  WASD:%s",
                 fps, (int)tracker.robots().size(),
                 g_selectedRobot < 0 ? "all" : std::to_string(g_selectedRobot).c_str(),
                 g_hasH ? "world" : "pixels",
                 g_swarm.isConnected() ? "OK" : "INACTIVE",
                 g_speedLevel, (float)g_speedLevel / 255.0f * 4.0f,
                 g_wasdActive ? "ON" : "off");
        cv::Scalar hudCol = g_swarm.isConnected() ? cv::Scalar(0,255,0) : cv::Scalar(0,120,255);
        tracker.drawText(disp, hud, {10, disp.rows - 19}, 18, hudCol);

        char wasdState[64];
        snprintf(wasdState, sizeof(wasdState), "W:%c A:%c S:%c D:%c",
                 g_keyW_a.load(std::memory_order_relaxed) ? 'X' : '-',
                 g_keyA_a.load(std::memory_order_relaxed) ? 'X' : '-',
                 g_keyS_a.load(std::memory_order_relaxed) ? 'X' : '-',
                 g_keyD_a.load(std::memory_order_relaxed) ? 'X' : '-');
        tracker.drawText(disp, wasdState, {disp.cols - 190, 24}, 18, cv::Scalar(0, 255, 255));

        {
            cv::imshow(WIN, disp);
        }
        int key = cv::waitKey(1) & 0xFF;
        if (key=='q' || key==27) break;
        if (key=='s') { g_targets.clear(); g_haveGlobalTarget = false; g_lockedLeaderId = -1; memset(motors,0,sizeof(motors)); sendSwarm(motors); printf("Stopped. Leader unlocked.\n"); }
        if (key=='c') runCalibration(tracker);
        if (key>='0' && key<='9') { g_selectedRobot=key-'0'; printf("Selected robot %d.\n",g_selectedRobot); }
        if (key=='.') { g_selectedRobot=-1; printf("Targeting all.\n"); }
        if (key=='+') { g_speedLevel = std::min(255, g_speedLevel + 16); printf("Speed level %d\n", g_speedLevel); }
        if (key=='-') { g_speedLevel = std::max(0, g_speedLevel - 16); printf("Speed level %d\n", g_speedLevel); }
    }

    g_running = false;
    wasdThread.join();
    memset(motors, 0, sizeof(motors));
    sendSwarm(motors);
    { std::lock_guard<std::mutex> lk(g_swarmMutex); g_swarm.disconnect(); }
    cv::destroyAllWindows();
    printf("Stopped.\n");
    return 0;
}
