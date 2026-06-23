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

static constexpr float K_DIST        = 0.40f;
static constexpr float K_ANGLE       = 0.50f;
static constexpr float K_YAW_D       = 0.08f;
static constexpr float MAX_SPEED     = 60.0f;
static constexpr float MAX_TURN      = 16.0f;
static constexpr float ARRIVAL_MM    = 20.0f;
static constexpr float SEND_INT_S    = 0.05f;
// 0.08, not the 0.25 this was tuned at originally: that value matched a
// ~30fps loop, but the 2026-06-15 sleep_for(50ms->1ms) fix (55e7d938) lets
// this loop run at the camera's ~115fps now, so the old alpha let raw
// per-frame ArUco corner-angle noise straight through into the D-term. See
// circle_demo.cpp's YAW_ALPHA comment for the full derivation.
static constexpr float YAW_ALPHA     = 0.08f;
static constexpr float DANGER_MM     = 120.0f;
static constexpr float SAFE_MM       = 230.0f;
static constexpr float AVOID_BLEND   = 0.70f;
static constexpr float DRAG_RADIUS_PX = 38.0f;  // pixel hit radius for drag pick
static constexpr int   MAX_ROBOTS    = 32;
static constexpr int   TEL_HEIGHT    = 200;

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

// ── Math helpers ──────────────────────────────────────────────────────────────

static float normAngle(float a) {
    while (a >  180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}
static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : v > hi ? hi : v;
}

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

struct AvoidState {
    float minDist = 1e6f;  // closest robot — speed scaling + hard stop
    bool  arc     = false; // this robot should arc around an obstacle
    float arcDx   = 0.f;
    float arcDy   = 0.f;
};

// O(N²/2) pair scan.
//   Moving → Stationary : moving robot always arcs (ignores ID priority)
//   Moving → Moving     : face-to-face → lower priority (higher ID) arcs
//   Either pair         : both get proximity minDist for speed scaling
static std::unordered_map<int, AvoidState> buildAvoidance(
    const std::unordered_map<int, RobotPose>& poses)
{
    std::unordered_map<int, AvoidState> result;
    for (auto& [id, _] : poses) result[id] = {};

    std::vector<int> ids;
    ids.reserve(poses.size());
    for (auto& [id, _] : poses) ids.push_back(id);
    std::sort(ids.begin(), ids.end());

    auto isMoving = [&](int id) -> bool {
        auto git = g_goals.find(id);
        if (git == g_goals.end()) return false;
        const auto& p = poses.at(id);
        float dx = git->second.x - p.x, dy = git->second.y - p.y;
        return sqrtf(dx*dx + dy*dy) > ARRIVAL_MM;
    };

    // Add arc perpendicular to (a→b) choosing the side that keeps `who` moving forward
    auto giveArc = [&](int who, float ny_ab, float nx_ab, float hx, float hy) {
        auto& cs  = result[who];
        cs.arc    = true;
        // two candidate perps to the a→b line
        float p1x = -ny_ab, p1y =  nx_ab;
        float p2x =  ny_ab, p2y = -nx_ab;
        if (hx*p1x + hy*p1y >= hx*p2x + hy*p2y)
            { cs.arcDx += p1x; cs.arcDy += p1y; }
        else
            { cs.arcDx += p2x; cs.arcDy += p2y; }
    };

    for (size_t i = 0; i < ids.size(); i++) {
        for (size_t j = i + 1; j < ids.size(); j++) {
            int lo = ids[i], hi = ids[j];   // lo < hi  ⟹  lo has higher priority
            const auto& pLo = poses.at(lo);
            const auto& pHi = poses.at(hi);
            float dx   = pHi.x - pLo.x;
            float dy   = pHi.y - pLo.y;
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist >= SAFE_MM) continue;

            result[lo].minDist = std::min(result[lo].minDist, dist);
            result[hi].minDist = std::min(result[hi].minDist, dist);

            float nx = dx / dist, ny = dy / dist;  // unit lo→hi
            float loHx = cosf(pLo.yaw * (float)M_PI / 180.f);
            float loHy = sinf(pLo.yaw * (float)M_PI / 180.f);
            float hiHx = cosf(pHi.yaw * (float)M_PI / 180.f);
            float hiHy = sinf(pHi.yaw * (float)M_PI / 180.f);

            bool loMoving = isMoving(lo);
            bool hiMoving = isMoving(hi);

            if (loMoving && !hiMoving) {
                // lo moving toward stationary hi — lo avoids (ignores priority).
                // Also escape when already inside danger zone.
                if (dist < DANGER_MM || loHx*nx + loHy*ny > 0.25f)
                    giveArc(lo, ny, nx, loHx, loHy);

            } else if (!loMoving && hiMoving) {
                // hi moving toward stationary lo — hi avoids (ignores priority).
                if (dist < DANGER_MM || hiHx*(-nx) + hiHy*(-ny) > 0.25f)
                    giveArc(hi, ny, nx, hiHx, hiHy);

            } else if (loMoving && hiMoving) {
                // Both moving — face-to-face: higher ID (lower priority) arcs.
                // Also trigger in danger zone even without face-to-face — emergency escape.
                bool faceFace = (loHx*nx  + loHy*ny)  > 0.25f
                             && (hiHx*(-nx) + hiHy*(-ny)) > 0.25f;
                if (faceFace || dist < DANGER_MM)
                    giveArc(hi, ny, nx, hiHx, hiHy);
            }
        }
    }

    for (auto& [id, cs] : result) {
        if (cs.arc) {
            float len = sqrtf(cs.arcDx*cs.arcDx + cs.arcDy*cs.arcDy);
            if (len > 0.f) { cs.arcDx /= len; cs.arcDy /= len; }
        }
    }
    return result;
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

static cv::Mat buildTelPanel(int width,
    const std::unordered_map<int, RobotPose>& poses,
    const std::unordered_map<int, AvoidState>& avoidance,
    const int8_t motors[][2],
    SwarmClient& swarm)
{
    cv::Mat panel(TEL_HEIGHT, width, CV_8UC3, cv::Scalar(18,18,18));

    DemoHud hud;
    hud.title("Drag-Drop Demo");
    hud.header({"ID", "Vision", "Battery", "Latency", "Mot-L", "Mot-R", "Dist", "Status"});

    std::set<int> allIds;
    for (auto& [id, _] : poses) allIds.insert(id);
    for (int id : swarm.knownIds()) allIds.insert(id);

    for (int id : allIds) {
        const auto& ss  = swarm.robotState((uint8_t)id);
        bool vis = poses.count(id) > 0;
        bool arc = vis && avoidance.count(id) && avoidance.at(id).arc;

        cv::Scalar col = !vis ? (ss.known ? DemoHud::COL_WARN : DemoHud::COL_TEXT)
                               : arc ? DemoHud::COL_WARN : DemoHud::COL_OK;

        // Distance to goal
        float distMm = -1.f;
        if (vis && g_goals.count(id)) {
            const auto& p = poses.at(id);
            auto& g = g_goals.at(id);
            distMm = sqrtf((g.x-p.x)*(g.x-p.x) + (g.y-p.y)*(g.y-p.y));
        }

        const char* status = !vis ? (ss.known ? "RADIO" : "UNSEEN")
                                  : arc ? "ARC"
                                  : g_goals.count(id) ? "GOAL" : "IDLE";

        hud.row({
            DemoHud::fmt("%d", id),
            vis ? "YES" : "NO",
            ss.known ? DemoHud::formatBattery(ss.battery) : "--",
            ss.known ? DemoHud::formatLatency(ss.latencyUs) : "--",
            vis ? DemoHud::fmt("%+d", (int)motors[id][0]) : "--",
            vis ? DemoHud::fmt("%+d", (int)motors[id][1]) : "--",
            distMm >= 0.f ? DemoHud::fmt("%.0f", distMm) : "--",
            status,
        }, col);
    }
    hud.draw(panel, {0, 0}, width);
    return panel;
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

    std::unordered_map<int, float> smoothedYaw;
    std::unordered_map<int, float> prevAngleErr;
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

        // Build pose map with smoothed yaw
        g_poses.clear();
        for (auto& r : tracker.robots()) {
            RobotPose pose = r;
            auto [it, fresh] = smoothedYaw.emplace(r.id, r.yaw);
            if (!fresh) {
                float delta = normAngle(r.yaw - it->second);
                it->second += YAW_ALPHA * delta;
            }
            pose.yaw = it->second;
            g_poses[r.id] = pose;
        }

        // Avoidance
        std::unordered_map<int, AvoidState> avoidance;
        if (g_poses.size() > 1)
            avoidance = buildAvoidance(g_poses);

        // Motor control
        memcpy(motors, lastMotors, sizeof(motors));

        for (auto& [id, pose] : g_poses) {
            auto& av = avoidance[id];

            float maxSpd  = MAX_SPEED * robotSpeed(id);
            bool  hardStop = false;
            float arcX = 0.f, arcY = 0.f, arcBlend = 0.f;

            float halfD = DANGER_MM * 0.5f;
            if (av.minDist < halfD) {
                if (av.arc) {
                    maxSpd *= 0.15f;  // dodging: crawl through maneuver, don't hard stop
                } else {
                    hardStop = true;
                }
            } else if (av.minDist < SAFE_MM) {
                float s = clampf((av.minDist - halfD) / (SAFE_MM - halfD), 0.f, 1.f);
                maxSpd *= s;
            }
            if (av.arc) {
                float str = clampf(1.f - (av.minDist - DANGER_MM) / (SAFE_MM - DANGER_MM), 0.f, 1.f);
                arcBlend = AVOID_BLEND * str;
                arcX = av.arcDx;
                arcY = av.arcDy;
            }

            if (hardStop) { motors[id][0] = motors[id][1] = 0; continue; }

            auto git = g_goals.find(id);
            if (git == g_goals.end()) { motors[id][0] = motors[id][1] = 0; continue; }

            float dx   = git->second.x - pose.x;
            float dy   = git->second.y - pose.y;
            float dist = sqrtf(dx*dx + dy*dy);

            if (dist < ARRIVAL_MM) { motors[id][0] = motors[id][1] = 0; continue; }

            // Bend toward arc detour direction
            dx += arcX * dist * arcBlend;
            dy += arcY * dist * arcBlend;

            float tgtAngle  = atan2f(dy, dx) * 180.f / (float)M_PI;
            float angleErr  = normAngle(tgtAngle - pose.yaw);
            float headingN  = clampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
            float headingSc = 1.f - headingN * headingN;
            float brakeSc   = clampf((dist - ARRIVAL_MM) / ARRIVAL_MM, 0.f, 1.f);

            float dAngleErr = 0.f;
            {
                auto it = prevAngleErr.find(id);
                if (it != prevAngleErr.end())
                    dAngleErr = clampf(normAngle(angleErr - it->second) / controlDt,
                                       -300.f, 300.f);
                prevAngleErr[id] = angleErr;
            }

            float forward = clampf(K_DIST * dist, 0.f, maxSpd) * headingSc * brakeSc;
            float turn    = clampf(K_ANGLE * angleErr + K_YAW_D * dAngleErr, -MAX_TURN, MAX_TURN);
            motors[id][0] = (int8_t)clampf(forward + turn, -100, 100);
            motors[id][1] = (int8_t)clampf(forward - turn, -100, 100);
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
            bool isArc    = avoidance.count(id) && avoidance.at(id).arc;

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

        // HUD
        {
            char buf[64];
            snprintf(buf, sizeof(buf), "loop_fps:%.0f  Robots:%d  Goals:%d",
                     fps, (int)g_poses.size(), (int)g_goals.size());
            ArucoTracker::drawText(disp, buf, {10, 30}, 18, {200,200,200});
        }

        // Telemetry panel
        cv::Mat tel = buildTelPanel(disp.cols, g_poses, avoidance, motors, swarm);
        cv::Mat canvas(disp.rows + TEL_HEIGHT, disp.cols, CV_8UC3);
        disp.copyTo(canvas(cv::Rect(0, 0, disp.cols, disp.rows)));
        tel.copyTo(canvas(cv::Rect(0, disp.rows, disp.cols, TEL_HEIGHT)));
        cv::imshow(WIN, canvas);

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
