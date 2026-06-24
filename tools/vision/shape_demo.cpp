// shape_demo.cpp — Draw shapes; robots slowly track the path.
//
// Usage: ./shape_demo [--serial SN] [--ip IP] [--calibrate] [--speed PCT]
//
// Draw mode (default):
//   l = line   r = rectangle   o = circle   f = freehand
//   left-drag = draw shape     right-click  = undo last shape
//   x = clear all shapes       s = save     d = switch to Track mode
//
// Track mode:
//   d = back to Draw   s = stop all   +/- = speed
//   0-9 = select robot  . = select all   c = recalibrate   q/Esc = quit

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
#include <set>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <algorithm>

#include <unistd.h>
#include <glob.h>

// ── Tunables ──────────────────────────────────────────────────────────────────

static constexpr float K_DIST          = 0.40f;
static constexpr float K_ANGLE         = 0.50f;
static constexpr float K_YAW_D         = 0.08f;
static constexpr float MAX_SPEED       = 51.7f;
static constexpr float MAX_TURN        = 16.0f;
static constexpr float ARRIVAL_MM      = 40.0f;
static constexpr float SEND_INT_S      = 0.05f;
// Yaw low-pass, specified as a time-constant (seconds), NOT a fixed per-frame
// EMA coefficient: each frame we convert it to alpha = dt/(YAW_TAU_S + dt) using
// the actual frame dt, so the smoothing holds the same memory in *time* at any
// loop rate. A fixed per-frame alpha silently over-smooths (heading lag →
// limit-cycle) when the loop runs slower than it was tuned for — e.g. on a
// weaker PC or at higher camera resolution. 0.50s matches the value found to
// kill that oscillation in circle_demo; see its YAW_TAU_S comment for the full
// derivation. Lower toward ~0.10s on a fast camera/PC if the lag cuts corners.
static constexpr float YAW_TAU_S       = 0.50f;
static constexpr float EVICT_S         = 5.0f;
static constexpr float WAYPOINT_STEP   = 25.0f;  // mm between sampled waypoints
static constexpr int   MAX_ROBOTS      = 32;

static const char* SHAPE_FILE      = "/tmp/shape_demo.yml";
static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";

// ── Mode / tool ───────────────────────────────────────────────────────────────

enum class Mode { DRAW, TRACK };
enum class Tool { LINE, RECT, CIRCLE, FREEHAND };

static Mode g_mode    = Mode::DRAW;
static Tool g_tool    = Tool::FREEHAND;
static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

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

static cv::Mat g_H;
static bool    g_hasH = false;

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

// ── Shape system ──────────────────────────────────────────────────────────────

struct DrawnShape {
    Tool type;
    std::vector<cv::Point2f> pts;  // world-space control points
};

static std::vector<cv::Point2f> sampleShape(const DrawnShape& s, float step) {
    std::vector<cv::Point2f> out;

    auto addSegment = [&](cv::Point2f a, cv::Point2f b) {
        float len = (float)cv::norm(b - a);
        if (len < 1.f) return;
        int N = std::max(2, (int)(len / step));
        for (int i = 0; i < N; i++)
            out.push_back(a + (b - a) * ((float)i / N));
    };

    switch (s.type) {
    case Tool::LINE:
        if (s.pts.size() >= 2) addSegment(s.pts[0], s.pts[1]);
        break;

    case Tool::RECT:
        if (s.pts.size() >= 2) {
            cv::Point2f tl = s.pts[0], br = s.pts[1];
            cv::Point2f tr(br.x, tl.y), bl(tl.x, br.y);
            addSegment(tl, tr); addSegment(tr, br);
            addSegment(br, bl); addSegment(bl, tl);
        }
        break;

    case Tool::CIRCLE:
        if (s.pts.size() >= 2) {
            float r = (float)cv::norm(s.pts[1] - s.pts[0]);
            if (r < 1.f) break;
            int N = std::max(8, (int)(2.f * (float)M_PI * r / step));
            for (int i = 0; i < N; i++) {
                float a = 2.f * (float)M_PI * i / N;
                out.push_back({s.pts[0].x + r * cosf(a), s.pts[0].y + r * sinf(a)});
            }
        }
        break;

    case Tool::FREEHAND:
        if (s.pts.size() < 2) {
            if (!s.pts.empty()) out.push_back(s.pts[0]);
            break;
        }
        {
            std::vector<float> cum(s.pts.size(), 0.f);
            for (size_t i = 1; i < s.pts.size(); i++)
                cum[i] = cum[i-1] + (float)cv::norm(s.pts[i] - s.pts[i-1]);
            float total = cum.back();
            if (total < 1.f) { out.push_back(s.pts[0]); break; }
            int N = std::max(2, (int)(total / step) + 1);
            for (int i = 0; i < N; i++) {
                float t = total * i / (N - 1);
                auto it  = std::lower_bound(cum.begin(), cum.end(), t);
                int idx  = std::clamp((int)(it - cum.begin()), 1, (int)s.pts.size() - 1);
                float sl = cum[idx] - cum[idx-1];
                float al = sl > 0.f ? (t - cum[idx-1]) / sl : 0.f;
                out.push_back(s.pts[idx-1] + (s.pts[idx] - s.pts[idx-1]) * al);
            }
        }
        break;
    }
    return out;
}

static std::vector<cv::Point2f> buildWaypoints(const std::vector<DrawnShape>& shapes) {
    std::vector<cv::Point2f> all;
    for (auto& s : shapes) {
        auto pts = sampleShape(s, WAYPOINT_STEP);
        all.insert(all.end(), pts.begin(), pts.end());
    }
    return all;
}

// ── Shape save / load ─────────────────────────────────────────────────────────

static void saveShapes(const std::vector<DrawnShape>& shapes) {
    cv::FileStorage fs(SHAPE_FILE, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { fprintf(stderr, "[shape] save failed\n"); return; }
    fs << "count" << (int)shapes.size();
    for (int i = 0; i < (int)shapes.size(); i++) {
        std::string k = "shape_" + std::to_string(i);
        fs << k + "_type" << (int)shapes[i].type;
        fs << k + "_pts"  << shapes[i].pts;
    }
    printf("[shape] Saved %d shape(s)\n", (int)shapes.size());
}

static std::vector<DrawnShape> loadShapes() {
    std::vector<DrawnShape> out;
    cv::FileStorage fs(SHAPE_FILE, cv::FileStorage::READ);
    if (!fs.isOpened()) return out;
    int count = 0; fs["count"] >> count;
    for (int i = 0; i < count; i++) {
        std::string k = "shape_" + std::to_string(i);
        int type = 0; std::vector<cv::Point2f> pts;
        fs[k + "_type"] >> type;
        fs[k + "_pts"]  >> pts;
        out.push_back({(Tool)type, pts});
    }
    printf("[shape] Loaded %d shape(s)\n", (int)out.size());
    return out;
}

// ── Mouse drawing ─────────────────────────────────────────────────────────────

struct MouseState {
    bool                     drawing = false;
    cv::Point2f              startPx, livePx;
    std::vector<cv::Point2f> freehandPx;
};

static MouseState             g_mouse;
static std::vector<DrawnShape> g_shapes;
static std::vector<cv::Point2f> g_waypoints;
static bool                   g_waypointsDirty = false;

static void onMouse(int event, int x, int y, int, void*) {
    if (g_mode != Mode::DRAW) return;
    cv::Point2f px((float)x, (float)y);

    if (event == cv::EVENT_LBUTTONDOWN) {
        g_mouse.drawing = true;
        g_mouse.startPx = px; g_mouse.livePx = px;
        g_mouse.freehandPx.clear();
        g_mouse.freehandPx.push_back(px);
    } else if (event == cv::EVENT_MOUSEMOVE && g_mouse.drawing) {
        g_mouse.livePx = px;
        if (g_tool == Tool::FREEHAND) g_mouse.freehandPx.push_back(px);
    } else if (event == cv::EVENT_LBUTTONUP && g_mouse.drawing) {
        g_mouse.drawing = false; g_mouse.livePx = px;

        DrawnShape shape; shape.type = g_tool;
        if (g_tool == Tool::FREEHAND) {
            g_mouse.freehandPx.push_back(px);
            for (auto& p : g_mouse.freehandPx)
                shape.pts.push_back(pixelToWorld(p));
        } else {
            shape.pts = {pixelToWorld(g_mouse.startPx), pixelToWorld(px)};
        }

        bool valid = shape.pts.size() >= 2 &&
                     cv::norm(shape.pts.back() - shape.pts.front()) > 5.f;
        if (!valid && g_tool == Tool::FREEHAND) valid = shape.pts.size() >= 5;
        if (valid) { g_shapes.push_back(std::move(shape)); g_waypointsDirty = true; }
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        if (!g_shapes.empty()) {
            g_shapes.pop_back(); g_waypointsDirty = true;
            printf("[shape] Undo  (%d remaining)\n", (int)g_shapes.size());
        }
    }
}

// ── Shape rendering ───────────────────────────────────────────────────────────

// Draw a shape during drag (pixel coords, no homography needed).
static void drawShapePreview(cv::Mat& img, cv::Point2f s, cv::Point2f e,
                             Tool tool, const std::vector<cv::Point2f>& fh,
                             cv::Scalar col, int thick) {
    switch (tool) {
    case Tool::LINE:
        cv::line(img, s, e, col, thick, cv::LINE_AA); break;
    case Tool::RECT: {
        cv::Rect2f r(std::min(s.x,e.x), std::min(s.y,e.y),
                     fabsf(e.x-s.x), fabsf(e.y-s.y));
        cv::rectangle(img, r, col, thick, cv::LINE_AA); break;
    }
    case Tool::CIRCLE:
        cv::circle(img, s, (int)cv::norm(e - s), col, thick, cv::LINE_AA); break;
    case Tool::FREEHAND:
        for (size_t i = 1; i < fh.size(); i++)
            cv::line(img, fh[i-1], fh[i], col, thick, cv::LINE_AA);
        break;
    }
}

// Draw a committed shape using world→pixel transform.
static void drawShapeWorld(cv::Mat& img, const DrawnShape& s, cv::Scalar col, int thick) {
    auto px = [](cv::Point2f p) { return worldToPixel(p); };
    switch (s.type) {
    case Tool::LINE:
        if (s.pts.size() >= 2)
            cv::line(img, px(s.pts[0]), px(s.pts[1]), col, thick, cv::LINE_AA);
        break;
    case Tool::RECT:
        if (s.pts.size() >= 2) {
            cv::Point2f tl=px(s.pts[0]), br=px(s.pts[1]);
            cv::Point2f tr(br.x,tl.y), bl(tl.x,br.y);
            cv::line(img,tl,tr,col,thick,cv::LINE_AA); cv::line(img,tr,br,col,thick,cv::LINE_AA);
            cv::line(img,br,bl,col,thick,cv::LINE_AA); cv::line(img,bl,tl,col,thick,cv::LINE_AA);
        }
        break;
    case Tool::CIRCLE:
        if (s.pts.size() >= 2) {
            cv::Point2f cp=px(s.pts[0]), ep=px(s.pts[1]);
            cv::circle(img, cp, (int)cv::norm(ep-cp), col, thick, cv::LINE_AA);
        }
        break;
    case Tool::FREEHAND:
        for (size_t i = 1; i < s.pts.size(); i++)
            cv::line(img, px(s.pts[i-1]), px(s.pts[i]), col, thick, cv::LINE_AA);
        break;
    }
}

// ── Telemetry panel ───────────────────────────────────────────────────────────

struct RobotTelRow {
    int      id;
    bool     visible;
    bool     known;
    uint8_t  battery;
    uint16_t latencyUs;
    int8_t   motorL, motorR;
    float    distMm;         // -1 = no waypoint assigned
};

// Overlays the demo HUD (summary line + uniform per-robot table) onto the live
// frame, docked top-right — same style and placement as every other demo.
static void drawTelHud(cv::Mat& disp, const std::vector<RobotTelRow>& rows,
    float fps, int visible, int registered, float pathMm,
    float speedPct, bool hubOk, Mode mode, Tool tool)
{
    static const char* TOOL_NAMES[] = {"LINE","RECT","CIRCLE","FREEHAND"};

    DemoHud hud;
    if (mode == Mode::DRAW)
        hud.title(DemoHud::fmt(
            "loop_fps:%.0f  Robots:%d/%d  Path:%.0fmm  Speed:%.0f%%  MODE:DRAW  TOOL:%s  HUB:%s",
            fps, visible, registered, pathMm, speedPct,
            TOOL_NAMES[(int)tool], hubOk ? "OK" : "OFFLINE"),
            hubOk ? DemoHud::COL_OK : DemoHud::COL_BAD);
    else
        hud.title(DemoHud::fmt(
            "loop_fps:%.0f  Robots:%d/%d  Path:%.0fmm  Speed:%.0f%%  MODE:TRACK  HUB:%s",
            fps, visible, registered, pathMm, speedPct, hubOk ? "OK" : "OFFLINE"),
            hubOk ? DemoHud::COL_OK : DemoHud::COL_BAD);

    hud.header({"ID", "Vision", "Battery", "Latency", "Mot-L", "Mot-R", "Status"});
    for (auto& r : rows) {
        cv::Scalar col = r.visible ? DemoHud::COL_OK
                       : (r.known ? DemoHud::COL_WARN : DemoHud::COL_TEXT);
        hud.row({
            DemoHud::fmt("%d", r.id),
            r.visible ? "YES" : "NO",
            r.known ? DemoHud::formatBattery(r.battery) : "--",
            r.known ? DemoHud::formatLatency(r.latencyUs) : "--",
            r.visible ? DemoHud::fmt("%+d", (int)r.motorL) : "--",
            r.visible ? DemoHud::fmt("%+d", (int)r.motorR) : "--",
            r.visible ? "ACTIVE" : (r.known ? "RADIO" : "UNSEEN"),
        }, col);
    }
    hud.drawTopRight(disp);
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

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    std::string serial, ip;
    float speedPct  = 40.f;
    bool  doCalib   = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--serial")    && i+1<argc) serial  = argv[++i];
        if (!strcmp(argv[i], "--ip")        && i+1<argc) ip      = argv[++i];
        if (!strcmp(argv[i], "--speed")     && i+1<argc) speedPct = atof(argv[++i]);
        if (!strcmp(argv[i], "--calibrate"))              doCalib  = true;
    }

    float defaultSpeed = clampf(speedPct / 100.f, 0.05f, 2.f);
    std::unordered_map<int,float> robotSpeedOverride;
    int  selectedRobot = -1;
    bool allSelected   = false;

    auto robotSpeed = [&](int id) -> float {
        auto it = robotSpeedOverride.find(id);
        return it != robotSpeedOverride.end() ? it->second : defaultSpeed;
    };

    // Hub
    SwarmClient swarm;
    if (swarm.connect()) printf("[hub] Connected.\n");
    else                  printf("[hub] Not available — will retry.\n");

    // Tracker
    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    cfg.debugOverlay = true;
    ArucoTracker tracker(cfg);
    if (!tracker.open()) { fprintf(stderr, "Could not open Basler camera.\n"); return 1; }
    printf("Camera: %dx%d\n",
           tracker.frameSize().width, tracker.frameSize().height);

    if (!doCalib && tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
        printf("Loaded homography.\n");
    }

    const char* WIN = "Shape Demo";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
    cv::setMouseCallback(WIN, onMouse, nullptr);
    if (doCalib) runCalibration(tracker, WIN);

    g_shapes = loadShapes();
    g_waypointsDirty = !g_shapes.empty();

    // Per-robot tracking state
    std::unordered_map<int, int>   robotSlotIndex;
    int                            registeredCount = 0;
    std::unordered_map<int, std::chrono::steady_clock::time_point> robotLastSeen;
    std::unordered_map<int, std::chrono::steady_clock::time_point> robotLostSince;
    std::unordered_map<int, float> smoothedYaw;
    std::unordered_map<int, float> prevAngleErr;
    std::unordered_map<int, int>   robotWaypoint;   // id → waypoint index

    int8_t motors[MAX_ROBOTS][2]     = {};
    int8_t lastMotors[MAX_ROBOTS][2] = {};

    auto t0         = std::chrono::steady_clock::now();
    auto lastSend   = t0;
    auto lastFpsT   = t0;
    auto lastCtrlT  = t0;
    auto lastRetry  = t0 - std::chrono::seconds(10);
    int  frameCount = 0;
    float fps       = 0.f;

    printf("\nDRAW:  l=line  r=rect  o=circle  f=freehand\n"
           "       left-drag=draw  right-click=undo  x=clear  s=save  d=track\n"
           "TRACK: d=draw  s=stop  +/-=speed  0-9=robot  .=all  c=calib  q=quit\n\n");

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

        // Hub reconnect
        if (!swarm.isConnected() &&
            std::chrono::duration<float>(now - lastRetry).count() >= 2.f) {
            lastRetry = now;
            if (swarm.connect()) printf("[hub] Connected.\n");
        }
        swarm.poll();

        float fpsDt = std::chrono::duration<float>(now - lastFpsT).count();
        if (fpsDt >= 1.f) { fps = frameCount / fpsDt; frameCount = 0; lastFpsT = now; }

        // Rebuild waypoints on demand
        if (g_waypointsDirty) {
            g_waypoints = buildWaypoints(g_shapes);
            g_waypointsDirty = false;
            printf("[shape] %d waypoints\n", (int)g_waypoints.size());
        }

        // ── Pose map ─────────────────────────────────────────────────────────
        std::unordered_map<int, RobotPose> poseById;
        for (auto& r : tracker.robots()) {
            if (r.id < 0 || r.id >= MAX_ROBOTS) continue;
            poseById[r.id] = r;
            robotLastSeen[r.id] = now;
        }

        // EMA yaw — alpha derived per-frame from controlDt so the time-constant
        // (YAW_TAU_S) holds regardless of loop rate.
        float yawAlpha = controlDt / (YAW_TAU_S + controlDt);
        for (auto& [id, pose] : poseById) {
            auto [it, fresh] = smoothedYaw.emplace(id, pose.yaw);
            if (!fresh) {
                float delta = normAngle(pose.yaw - it->second);
                it->second  = normAngle(it->second + yawAlpha * delta);
            }
            pose.yaw = it->second;
        }

        // ── Registry / eviction ───────────────────────────────────────────────
        {
            bool changed = false;

            for (auto& [id, _] : poseById) {
                robotLostSince.erase(id);
                swarm.registerRobot((uint8_t)id);
                if (!robotSlotIndex.count(id)) {
                    robotSlotIndex[id] = registeredCount++;
                    printf("[shape] Robot %d → slot %d/%d\n",
                           id, robotSlotIndex[id], registeredCount);
                    changed = true;
                }
            }

            for (auto& [id, _] : robotSlotIndex)
                if (!poseById.count(id) && !robotLostSince.count(id))
                    robotLostSince[id] = now;

            std::vector<int> evict;
            for (auto& [id, t] : robotLostSince)
                if (robotSlotIndex.count(id) &&
                    std::chrono::duration<float>(now - t).count() > EVICT_S)
                    evict.push_back(id);

            for (int id : evict) {
                int evicted = robotSlotIndex[id];
                robotSlotIndex.erase(id); robotLostSince.erase(id);
                smoothedYaw.erase(id); prevAngleErr.erase(id);
                robotWaypoint.erase(id);
                for (auto& [rid, si] : robotSlotIndex) if (si > evicted) si--;
                registeredCount--;
                changed = true;
                printf("[shape] Robot %d evicted. Remaining: %d\n", id, registeredCount);
            }

            if (changed && registeredCount > 0 && !g_waypoints.empty()) {
                int M = (int)g_waypoints.size();
                for (auto& [id, si] : robotSlotIndex)
                    robotWaypoint[id] = (si * M / registeredCount) % M;
            }
        }

        // ── Motor control ─────────────────────────────────────────────────────
        memcpy(motors, lastMotors, sizeof(motors));

        if (g_mode == Mode::TRACK) {
            for (auto& [id, pose] : poseById) {
                if (g_waypoints.empty() || !robotWaypoint.count(id)) {
                    motors[id][0] = motors[id][1] = 0;
                    continue;
                }
                cv::Point2f wp  = g_waypoints[robotWaypoint[id]];
                float dx        = wp.x - pose.x;
                float dy        = wp.y - pose.y;
                float dist      = sqrtf(dx*dx + dy*dy);

                if (dist < ARRIVAL_MM) { motors[id][0] = motors[id][1] = 0; continue; }

                float tgtAngle  = atan2f(dy, dx) * 180.f / (float)M_PI;
                float angleErr  = normAngle(tgtAngle - pose.yaw);
                float headingN  = clampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
                float headingSc = 1.f - headingN * headingN;
                float brakeSc   = clampf((dist - ARRIVAL_MM) / ARRIVAL_MM, 0.f, 1.f);
                float maxSpd    = MAX_SPEED * robotSpeed(id);

                float dAngleErr = 0.f;
                {
                    auto it = prevAngleErr.find(id);
                    if (it != prevAngleErr.end())
                        dAngleErr = clampf(normAngle(angleErr - it->second) / controlDt,
                                           -300.f, 300.f);
                    prevAngleErr[id] = angleErr;
                }

                float forward = clampf(K_DIST * dist, 0.f, maxSpd) * headingSc * brakeSc;
                float turn    = clampf(K_ANGLE * angleErr + K_YAW_D * dAngleErr,
                                       -MAX_TURN, MAX_TURN);
                motors[id][0] = (int8_t)clampf(forward + turn, -100, 100);
                motors[id][1] = (int8_t)clampf(forward - turn, -100, 100);
            }
        } else {
            memset(motors, 0, sizeof(motors));
        }

        // Watchdog: silence unseen robots
        for (int id = 0; id < MAX_ROBOTS; id++) {
            if (poseById.count(id)) continue;
            auto it = robotLastSeen.find(id);
            if (it == robotLastSeen.end()) continue;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - it->second).count() > 1000)
                motors[id][0] = motors[id][1] = 0;
        }

        // Send at SEND_INT_S rate
        if (std::chrono::duration<float>(now - lastSend).count() >= SEND_INT_S) {
            for (int id = 0; id < MAX_ROBOTS; id++)
                swarm.setSpeed((uint8_t)id, motors[id][0], motors[id][1]);
            swarm.flush();
            lastSend = now;
            memcpy(lastMotors, motors, sizeof(motors));
        }

        // ── Draw ─────────────────────────────────────────────────────────────
        cv::Mat disp = tracker.debugFrame().clone();

        // Committed shapes
        cv::Scalar shapeCol = (g_mode == Mode::DRAW)
            ? cv::Scalar(0, 200, 255) : cv::Scalar(0, 160, 80);
        for (auto& s : g_shapes)
            drawShapeWorld(disp, s, shapeCol, 2);

        if (g_mode == Mode::TRACK && !g_waypoints.empty()) {
            // Waypoint dots
            for (auto& wp : g_waypoints) {
                cv::Point2f p = worldToPixel(wp);
                cv::circle(disp, p, 3, {60, 60, 200}, -1, cv::LINE_AA);
            }
            // Robot → waypoint arrows and highlights
            for (auto& [id, pose] : poseById) {
                if (!robotWaypoint.count(id)) continue;
                cv::Point2f wp  = g_waypoints[robotWaypoint[id]];
                cv::Point2f wpPx = worldToPixel(wp);
                float dist = (float)cv::norm(wp - cv::Point2f(pose.x, pose.y));
                if (dist > ARRIVAL_MM)
                    cv::arrowedLine(disp, {(int)pose.px,(int)pose.py}, wpPx,
                                    {255,128,0}, 2, cv::LINE_AA, 0, 0.15);
                cv::circle(disp, wpPx, 6, {0,255,128}, -1, cv::LINE_AA);
                char lbl[8]; snprintf(lbl, sizeof(lbl), "%d", id);
                ArucoTracker::drawText(disp, lbl, wpPx + cv::Point2f(8,-8), 16, {0,255,128});
            }
        }

        // Live drawing preview
        if (g_mode == Mode::DRAW && g_mouse.drawing)
            drawShapePreview(disp, g_mouse.startPx, g_mouse.livePx, g_tool,
                             g_mouse.freehandPx, {0,255,200}, 2);

        // Selection rings + speed labels
        for (auto& [id, pose] : poseById) {
            bool sel = (id == selectedRobot) || allSelected;
            if (sel) cv::circle(disp, {(int)pose.px,(int)pose.py}, 22, {0,255,255}, 2, cv::LINE_AA);
            float spd = robotSpeed(id);
            if (spd != 1.f || sel) {
                char buf[16]; snprintf(buf, sizeof(buf), "%.0f%%", spd*100.f);
                ArucoTracker::drawText(disp, buf,
                    cv::Point2f(pose.px+14, pose.py+20), 15,
                    sel ? cv::Scalar(0,255,255) : cv::Scalar(170,170,170));
            }
        }

        // ── Telemetry panel ───────────────────────────────────────────────────
        std::set<int> allIds;
        for (auto& [id,_] : robotSlotIndex) allIds.insert(id);
        for (int id : swarm.knownIds())      allIds.insert(id);

        std::vector<RobotTelRow> rows;
        for (int id : allIds) {
            const auto& ss = swarm.robotState((uint8_t)id);
            bool vis = poseById.count(id) > 0;
            float distMm = -1.f;
            int8_t mL = 0, mR = 0;
            if (vis) {
                mL = motors[id][0]; mR = motors[id][1];
                if (robotWaypoint.count(id) && !g_waypoints.empty()) {
                    cv::Point2f wp = g_waypoints[robotWaypoint[id]];
                    auto& p = poseById[id];
                    float dx = wp.x - p.x, dy = wp.y - p.y;
                    distMm = sqrtf(dx*dx + dy*dy);
                }
            }
            rows.push_back({id, vis, ss.known || vis,
                            ss.battery, ss.latencyUs, mL, mR, distMm});
        }

        float pathLen = 0.f;
        for (size_t i = 1; i < g_waypoints.size(); i++)
            pathLen += (float)cv::norm(g_waypoints[i] - g_waypoints[i-1]);

        drawTelHud(disp, rows, fps,
                   (int)poseById.size(), registeredCount,
                   pathLen, defaultSpeed * 100.f,
                   swarm.isConnected(), g_mode, g_tool);
        cv::imshow(WIN, disp);

        // ── Key handling ──────────────────────────────────────────────────────
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;

        if (key == 'd') {
            if (g_mode == Mode::DRAW) {
                if (g_shapes.empty()) { printf("Draw something first!\n"); }
                else {
                    saveShapes(g_shapes);
                    g_mode = Mode::TRACK;
                    printf("TRACK mode  (%d waypoints)\n", (int)g_waypoints.size());
                }
            } else {
                g_mode = Mode::DRAW;
                for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed(id, 0, 0);
                swarm.flush();
                memset(motors, 0, sizeof(motors));
                memset(lastMotors, 0, sizeof(lastMotors));
                printf("DRAW mode.\n");
            }
        }

        if (g_mode == Mode::DRAW) {
            if (key == 'l') { g_tool = Tool::LINE;     printf("Tool: LINE\n"); }
            if (key == 'r') { g_tool = Tool::RECT;     printf("Tool: RECT\n"); }
            if (key == 'o') { g_tool = Tool::CIRCLE;   printf("Tool: CIRCLE\n"); }
            if (key == 'f') { g_tool = Tool::FREEHAND; printf("Tool: FREEHAND\n"); }
            if (key == 'x') {
                g_shapes.clear(); g_waypointsDirty = true;
                printf("Shapes cleared.\n");
            }
            if (key == 's') saveShapes(g_shapes);
        }

        if (g_mode == Mode::TRACK && key == 's') {
            for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed(id, 0, 0);
            swarm.flush();
            memset(motors, 0, sizeof(motors));
            memset(lastMotors, 0, sizeof(lastMotors));
            printf("Stopped.\n");
        }

        if (key == 'c') runCalibration(tracker, WIN);

        if (key >= '0' && key <= '9') {
            int id = key - '0';
            if (selectedRobot == id) { selectedRobot = -1; printf("Deselected %d\n", id); }
            else { selectedRobot = id; allSelected = false; printf("Selected %d\n", id); }
        }
        if (key == '.') { allSelected = !allSelected; selectedRobot = -1; }

        if (key == '+' || key == '=') {
            if (allSelected || selectedRobot < 0) {
                defaultSpeed = clampf(defaultSpeed + 0.1f, 0.05f, 2.f);
                printf("Speed: %.0f%%\n", defaultSpeed * 100.f);
            } else {
                float s = clampf(robotSpeed(selectedRobot) + 0.1f, 0.05f, 2.f);
                robotSpeedOverride[selectedRobot] = s;
                printf("Robot %d speed: %.0f%%\n", selectedRobot, s * 100.f);
            }
        }
        if (key == '-') {
            if (allSelected || selectedRobot < 0) {
                defaultSpeed = clampf(defaultSpeed - 0.1f, 0.05f, 2.f);
                printf("Speed: %.0f%%\n", defaultSpeed * 100.f);
            } else {
                float s = clampf(robotSpeed(selectedRobot) - 0.1f, 0.05f, 2.f);
                robotSpeedOverride[selectedRobot] = s;
                printf("Robot %d speed: %.0f%%\n", selectedRobot, s * 100.f);
            }
        }
    }

    g_running = false;
    for (int id = 0; id < MAX_ROBOTS; id++) swarm.setSpeed(id, 0, 0);
    swarm.flush();
    swarm.disconnect();
    cv::destroyAllWindows();
    printf("Stopped.\n");
    return 0;
}
