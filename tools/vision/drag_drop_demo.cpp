// drag_drop_demo.cpp — Click and drag robots to new positions.
//
// Usage: ./drag_drop_demo [--serial SN] [--ip IP] [--calibrate] [--speed PCT]
//
// Controls:
//   left-drag on robot = set live goal (robot follows cursor in real-time)
//   right-click robot  = clear its goal
//   s = stop all robots    +/- = global speed
//   0-9 = select robot     . = select all
//   c = recalibrate        q / Esc = quit

#include "aruco_tracker.h"
#include "SwarmClient.h"
#include "DemoHud.h"
#include "goto_controller.h"
#include "avoidance.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <thread>

#include <unistd.h>

// ── Tunables ──────────────────────────────────────────────────────────────────

// The control law itself lives in lib/SwarmControl/goto_controller.h; these are
// this demo's tuned gains for it. See that header before changing any of them —
// the other demos deliberately run different values.
static constexpr float K_DIST        = 0.40f;
static constexpr float K_ANGLE       = 0.50f;
static constexpr float K_YAW_D       = 0.08f;
// MAX_SPEED and MAX_TURN must be raised TOGETHER. computeGoto emits
// forward ± turn into a ±100 motor clamp, so a faster robot with unchanged turn
// authority saturates the outer wheel and turns lazily — and a lazy turn needs a
// longer dodge runway (see AvoidParams::dangerMm). Scaling both keeps the turn
// rate proportional to speed, which leaves the avoidance geometry intact:
// verified in sim at 1.5x (60/16 -> 90/24), all four encounter types still clear.
static constexpr float MAX_SPEED     = 90.0f;   // 1.5x
static constexpr float MAX_TURN      = 24.0f;   // 1.5x, in step with MAX_SPEED
static constexpr float ARRIVAL_MM    = 20.0f;
static constexpr float SEND_INT_S    = 0.05f;
// Yaw low-pass time-constant, seconds — see YawSmoother in goto_controller.h
// for why this is a time-constant and not a per-frame EMA coefficient.
static constexpr float YAW_TAU_S     = 0.70f;
// Avoidance is dodge-based, not brake-based — see lib/SwarmControl/avoidance.h.
// DANGER_MM triggers the dodge, SAFE_MM releases it (the gap between them is
// the hysteresis that keeps a maneuver committed). AVOID_BLEND is high and
// DODGE_SPEED_FRAC near full speed on purpose: the dodger needs to actually get
// around, and only the dodger is ever slowed.
// NOTE: DANGER_MM is bounded below by MAX_SPEED — a dodge cannot complete in
// less distance than it takes to turn (see AvoidParams::dangerMm for the
// inequality). At MAX_SPEED 60 that floor is ~340mm, hence 500. If the arena is
// too small to give pairs that much room, lower MAX_SPEED rather than this.
static constexpr float DANGER_MM     = 500.0f;
static constexpr float SAFE_MM       = 1000.0f;
static constexpr float AVOID_BLEND   = 0.95f;
static constexpr float DODGE_SPEED_FRAC = 0.70f;
static constexpr float DRAG_RADIUS_PX= 38.0f;  // pixel hit radius for drag pick
static constexpr int   MAX_ROBOTS    = 32;

using swarmctl::normAngle;
using swarmctl::clampf;

static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";

// ── Globals ───────────────────────────────────────────────────────────────────

static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

static cv::Mat g_H;
static bool    g_hasH = false;

// Latest tracked poses — updated each frame, read by mouse callback
static std::unordered_map<int, RobotPose> g_poses;

// Per-robot world-space goals; absent = no goal
static std::unordered_map<int, cv::Point2f> g_goals;

struct MouseState {
    int         draggingId = -1;
    cv::Point2f currentPx;
    bool        active     = false;
};
static MouseState g_mouse;

// ── World coordinate helpers ──────────────────────────────────────────────────

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

// ── Avoidance ─────────────────────────────────────────────────────────────────
// Pair scan, priority rules and detour math live in lib/SwarmControl/avoidance.h.
// This demo supplies only its own policy: what counts as "moving" (has a goal,
// and is not yet within the arrival radius).
//
// The engine is stateful — it latches each dodge so the maneuver holds instead
// of cancelling itself — so it must persist across frames, not be rebuilt.

static const swarmctl::AvoidParams AVOID_PARAMS = {
    .dangerMm = DANGER_MM,
    .safeMm   = SAFE_MM,
    .blend    = AVOID_BLEND,
    .faceDot  = 0.25f,
    .dodgeSpeedFrac = DODGE_SPEED_FRAC,
    // Scaled with MAX_SPEED: the last-resort stop has to fire early enough to
    // bleed off a proportionally faster approach (160 at MAX_SPEED 60).
    .emergencyMm = 240.0f,
};

static swarmctl::AvoidanceEngine g_avoid(AVOID_PARAMS);

static std::unordered_map<int, swarmctl::AvoidState> buildAvoidance(
    const std::unordered_map<int, RobotPose>& poses, float dt)
{
    auto isMoving = [&](int id) -> bool {
        auto git = g_goals.find(id);
        if (git == g_goals.end()) return false;
        const auto& p = poses.at(id);
        float dx = git->second.x - p.x, dy = git->second.y - p.y;
        return sqrtf(dx*dx + dy*dy) > ARRIVAL_MM;
    };
    return g_avoid.update(poses, isMoving, dt);
}

// ── Mouse callback ────────────────────────────────────────────────────────────

static void onMouse(int event, int x, int y, int, void*) {
    cv::Point2f px((float)x, (float)y);

    if (event == cv::EVENT_LBUTTONDOWN) {
        int   bestId   = -1;
        float bestDist = DRAG_RADIUS_PX;
        for (auto& [id, pose] : g_poses) {
            float d = sqrtf((pose.px-px.x)*(pose.px-px.x) + (pose.py-px.y)*(pose.py-px.y));
            if (d < bestDist) { bestDist = d; bestId = id; }
        }
        if (bestId >= 0) {
            g_mouse.draggingId = bestId;
            g_mouse.currentPx  = px;
            g_mouse.active     = true;
            g_goals[bestId]    = pixelToWorld(px);
        }
    } else if (event == cv::EVENT_MOUSEMOVE && g_mouse.active) {
        g_mouse.currentPx = px;
        if (g_mouse.draggingId >= 0)
            g_goals[g_mouse.draggingId] = pixelToWorld(px);  // live puppet update
    } else if (event == cv::EVENT_LBUTTONUP && g_mouse.active) {
        g_mouse.currentPx = px;
        if (g_mouse.draggingId >= 0)
            g_goals[g_mouse.draggingId] = pixelToWorld(px);
        g_mouse.active     = false;
        g_mouse.draggingId = -1;
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        for (auto& [id, pose] : g_poses) {
            float d = sqrtf((pose.px-px.x)*(pose.px-px.x) + (pose.py-px.y)*(pose.py-px.y));
            if (d < DRAG_RADIUS_PX) { g_goals.erase(id); break; }
        }
    }
}

// ── Calibration ───────────────────────────────────────────────────────────────

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
    g_H    = cv::findHomography(cs.pixPts, worldPts);
    g_hasH = !g_H.empty();
    if (g_hasH) {
        tracker.setHomography(cs.pixPts, worldPts);
        tracker.saveHomography(HOMOGRAPHY_FILE);
        printf("Saved homography.\n");
    }
    return g_hasH;
}

// ── Telemetry panel ───────────────────────────────────────────────────────────

// Overlays the demo HUD (summary line + uniform per-robot table) onto the live
// frame, docked top-right — same style and placement as every other demo.
static void drawTelHud(cv::Mat& disp,
    const std::unordered_map<int, RobotPose>& poses,
    const std::unordered_map<int, swarmctl::AvoidState>& avoidance,
    const int8_t motors[][2],
    SwarmClient& swarm, float fps)
{
    DemoHud hud;
    hud.title(DemoHud::fmt(
        "loop_fps:%.0f  Robots:%d  Goals:%d  HUB:%s",
        fps, (int)poses.size(), (int)g_goals.size(),
        swarm.isConnected() ? "OK" : "OFFLINE"),
        swarm.isConnected() ? DemoHud::COL_OK : DemoHud::COL_BAD);
    hud.header({"ID", "Vision", "Battery", "Latency", "Mot-L", "Mot-R", "Status"});

    std::set<int> allIds;
    for (auto& [id, _] : poses) allIds.insert(id);
    for (int id : swarm.knownIds()) allIds.insert(id);

    for (int id : allIds) {
        const auto& ss  = swarm.robotState((uint8_t)id);
        bool vis = poses.count(id) > 0;
        bool dodge = vis && avoidance.count(id) && avoidance.at(id).dodging;
        bool prio  = vis && avoidance.count(id) && avoidance.at(id).priority && !dodge;

        cv::Scalar col = !vis ? (ss.known ? DemoHud::COL_WARN : DemoHud::COL_TEXT)
                               : dodge ? DemoHud::COL_WARN : DemoHud::COL_OK;

        // DODGE / HOLD are the two halves of one conflict — seeing which robot
        // got which is the fastest way to tell whether the nomination was sane.
        const char* status = !vis ? (ss.known ? "RADIO" : "UNSEEN")
                                  : dodge ? "DODGE"
                                  : prio  ? "HOLD"
                                  : g_goals.count(id) ? "GOAL" : "IDLE";

        hud.row({
            DemoHud::fmt("%d", id),
            vis ? "YES" : "NO",
            ss.known ? DemoHud::formatBattery(ss.battery) : "--",
            ss.known ? DemoHud::formatLatency(ss.latencyUs) : "--",
            vis ? DemoHud::fmt("%+d", (int)motors[id][0]) : "--",
            vis ? DemoHud::fmt("%+d", (int)motors[id][1]) : "--",
            status,
        }, col);
    }
    hud.drawTopRight(disp);
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    std::string serial, ip;
    float speedPct = 40.f;
    bool  doCalib  = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--serial")    && i+1<argc) serial   = argv[++i];
        if (!strcmp(argv[i], "--ip")        && i+1<argc) ip       = argv[++i];
        if (!strcmp(argv[i], "--speed")     && i+1<argc) speedPct = atof(argv[++i]);
        if (!strcmp(argv[i], "--calibrate"))              doCalib  = true;
    }

    float defaultSpeed = clampf(speedPct / 100.f, 0.05f, 2.f);
    std::unordered_map<int, float> robotSpeedOverride;
    int  selectedRobot = -1;
    bool allSelected   = false;

    auto robotSpeed = [&](int id) -> float {
        auto it = robotSpeedOverride.find(id);
        return it != robotSpeedOverride.end() ? it->second : defaultSpeed;
    };

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

    if (!doCalib && tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
        printf("Loaded homography.\n");
    }

    const char* WIN = "Drag-Drop Demo";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
    cv::setMouseCallback(WIN, onMouse, nullptr);
    if (doCalib) runCalibration(tracker, WIN);

    swarmctl::YawSmoother smoothedYaw(YAW_TAU_S);
    std::unordered_map<int, swarmctl::GotoState> gotoState;

    const swarmctl::GotoParams GOTO_PARAMS = [] {
        swarmctl::GotoParams p;
        p.kDist = K_DIST; p.kAngle = K_ANGLE; p.kYawD = K_YAW_D;
        p.maxTurn = MAX_TURN; p.arrivalMm = ARRIVAL_MM;
        p.brake = true;   // single terminal goal — ease into it
        return p;
    }();
    int8_t motors[MAX_ROBOTS][2]     = {};
    int8_t lastMotors[MAX_ROBOTS][2] = {};

    auto t0        = std::chrono::steady_clock::now();
    auto lastSend  = t0;
    auto lastFpsT  = t0;
    auto lastCtrlT = t0;
    auto lastRetry = t0 - std::chrono::seconds(10);
    int  frameCount = 0;
    float fps = 0.f;

    printf("\nLeft-drag robot = set goal (live)   Right-click = clear goal\n"
           "s = stop all   +/- = speed   0-9 = select   . = all   c = calib   q = quit\n\n");

    while (g_running) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        ++frameCount;
        float dt = std::chrono::duration<float>(now - lastCtrlT).count();
        float controlDt = clampf(dt, 0.01f, 0.2f);
        lastCtrlT = now;

        if (!swarm.isConnected() &&
            std::chrono::duration<float>(now - lastRetry).count() >= 2.f) {
            lastRetry = now;
            if (swarm.connect()) printf("[hub] Connected.\n");
        }
        swarm.poll();

        float fpsDt = std::chrono::duration<float>(now - lastFpsT).count();
        if (fpsDt >= 1.f) { fps = frameCount/fpsDt; frameCount = 0; lastFpsT = now; }

        // Build pose map with smoothed yaw.
        g_poses.clear();
        for (auto& r : tracker.robots()) {
            RobotPose pose = r;
            pose.yaw = smoothedYaw.update(r.id, r.yaw, controlDt);
            g_poses[r.id] = pose;
        }

        // Avoidance
        // Called unconditionally, even with 0/1 robots visible: the engine holds
        // latch state, and skipping the call would strand stale dodges when
        // robots drop out of tracking.
        auto avoidance = buildAvoidance(g_poses, controlDt);

        // Motor control
        memcpy(motors, lastMotors, sizeof(motors));

        for (auto& [id, pose] : g_poses) {
            auto& av = avoidance[id];

            auto act = swarmctl::applyAvoidance(av, MAX_SPEED * robotSpeed(id),
                                                AVOID_PARAMS);

            if (act.hardStop) { motors[id][0] = motors[id][1] = 0; continue; }

            auto git = g_goals.find(id);
            if (git == g_goals.end()) { motors[id][0] = motors[id][1] = 0; continue; }

            float dx   = git->second.x - pose.x;
            float dy   = git->second.y - pose.y;
            float dist = sqrtf(dx*dx + dy*dy);

            if (dist < ARRIVAL_MM) { motors[id][0] = motors[id][1] = 0; continue; }

            // Bend toward arc detour direction
            dx += act.arcX * dist * act.arcBlend;
            dy += act.arcY * dist * act.arcBlend;

            auto cmd = swarmctl::computeGoto(dx, dy, pose.yaw, act.maxSpd,
                                             GOTO_PARAMS, gotoState[id], controlDt);
            motors[id][0] = cmd.left;
            motors[id][1] = cmd.right;
        }

        // Silence robots not seen
        for (int id = 0; id < MAX_ROBOTS; id++) {
            if (!g_poses.count(id)) motors[id][0] = motors[id][1] = 0;
        }

        if (std::chrono::duration<float>(now - lastSend).count() >= SEND_INT_S) {
            for (int id = 0; id < MAX_ROBOTS; id++)
                swarm.setSpeed((uint8_t)id, motors[id][0], motors[id][1]);
            swarm.flush();
            lastSend = now;
            memcpy(lastMotors, motors, sizeof(motors));
        }

        // ── Draw ─────────────────────────────────────────────────────────────
        cv::Mat disp = tracker.debugFrame().clone();

        // Committed goals
        for (auto& [id, goal] : g_goals) {
            if (!g_poses.count(id)) continue;
            const auto& pose  = g_poses.at(id);
            cv::Point2f goalPx = worldToPixel(goal);
            float dx = goal.x - pose.x, dy = goal.y - pose.y;
            bool arrived = sqrtf(dx*dx + dy*dy) < ARRIVAL_MM;
            cv::Scalar goalCol = arrived ? cv::Scalar(0,255,80) : cv::Scalar(0,200,255);
            // Target crosshair
            cv::circle(disp, goalPx, 10, goalCol, 2, cv::LINE_AA);
            cv::line(disp, goalPx - cv::Point2f(14,0), goalPx + cv::Point2f(14,0), goalCol, 1, cv::LINE_AA);
            cv::line(disp, goalPx - cv::Point2f(0,14), goalPx + cv::Point2f(0,14), goalCol, 1, cv::LINE_AA);
            if (!arrived)
                cv::arrowedLine(disp, {(int)pose.px,(int)pose.py}, goalPx,
                                goalCol, 2, cv::LINE_AA, 0, 0.12);
        }

        // Per-robot overlays
        for (auto& [id, pose] : g_poses) {
            bool sel      = (id == selectedRobot) || allSelected;
            bool isDragged = g_mouse.active && g_mouse.draggingId == id;
            bool isArc    = avoidance.count(id) && avoidance.at(id).dodging;

            // Drag hitbox ring
            cv::Scalar hitCol = isDragged ? cv::Scalar(0,255,200)
                                          : cv::Scalar(60,60,60);
            cv::circle(disp, {(int)pose.px,(int)pose.py}, (int)DRAG_RADIUS_PX, hitCol, 1, cv::LINE_AA);

            // Arc ring
            if (isArc)
                cv::circle(disp, {(int)pose.px,(int)pose.py}, 28, cv::Scalar(0,60,255), 2, cv::LINE_AA);

            // Selection ring
            if (sel)
                cv::circle(disp, {(int)pose.px,(int)pose.py}, 22, {0,255,255}, 2, cv::LINE_AA);

            // Speed label
            float spd = robotSpeed(id);
            if (spd != 1.f || sel) {
                char buf[16]; snprintf(buf, sizeof(buf), "%.0f%%", spd*100.f);
                ArucoTracker::drawText(disp, buf,
                    cv::Point2f(pose.px+14, pose.py+20), 15,
                    sel ? cv::Scalar(0,255,255) : cv::Scalar(170,170,170));
            }
        }

        // Live drag line + cursor crosshair
        if (g_mouse.active && g_mouse.draggingId >= 0) {
            cv::Point2f tgt = g_mouse.currentPx;
            cv::Scalar dragCol(0,255,200);
            cv::circle(disp, tgt, 12, dragCol, 2, cv::LINE_AA);
            cv::line(disp, tgt-cv::Point2f(16,0), tgt+cv::Point2f(16,0), dragCol, 1, cv::LINE_AA);
            cv::line(disp, tgt-cv::Point2f(0,16), tgt+cv::Point2f(0,16), dragCol, 1, cv::LINE_AA);
            if (g_poses.count(g_mouse.draggingId)) {
                const auto& p = g_poses.at(g_mouse.draggingId);
                cv::line(disp, {(int)p.px,(int)p.py}, tgt, dragCol, 1, cv::LINE_AA);
            }
        }

        // Demo HUD (summary line + uniform per-robot table, docked top-right)
        drawTelHud(disp, g_poses, avoidance, motors, swarm, fps);
        cv::imshow(WIN, disp);

        // Key handling
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27)  g_running = false;
        if (key == 's') {
            g_goals.clear();
            memset(motors, 0, sizeof(motors));
            for (int id = 0; id < MAX_ROBOTS; id++)
                swarm.setSpeed((uint8_t)id, 0, 0);
            swarm.flush();
            printf("[drag] All stopped.\n");
        }
        if (key == 'c') runCalibration(tracker, WIN);
        if (key == '.') { allSelected = !allSelected; selectedRobot = -1; }
        if (key >= '0' && key <= '9') {
            int rid = key - '0';
            selectedRobot = (selectedRobot == rid) ? -1 : rid;
            allSelected = false;
        }
        if (key == '+' || key == '=') {
            int tgt = (selectedRobot >= 0 && !allSelected) ? selectedRobot : -1;
            if (tgt < 0) { defaultSpeed = clampf(defaultSpeed + 0.05f, 0.05f, 2.f); }
            else robotSpeedOverride[tgt] = clampf(robotSpeed(tgt) + 0.05f, 0.05f, 2.f);
        }
        if (key == '-') {
            int tgt = (selectedRobot >= 0 && !allSelected) ? selectedRobot : -1;
            if (tgt < 0) { defaultSpeed = clampf(defaultSpeed - 0.05f, 0.05f, 2.f); }
            else robotSpeedOverride[tgt] = clampf(robotSpeed(tgt) - 0.05f, 0.05f, 2.f);
        }
    }

    // Halt all on exit
    for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed((uint8_t)id, 0, 0);
    swarm.flush();
    return 0;
}
