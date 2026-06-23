// wingman.cpp — V-Formation Swarm Controller
// ═══════════════════════════════════════════════════════════════════════════════
//
// The robot with the lowest visible ArUco ID is the LEADER.
// The operator drives it manually with WASD (macOS) or arrow keys.
// All other robots autonomously hold a V-formation behind the leader.
//
// Formation layout (leader at apex, looking forward = +X in leader-local frame):
//
//   Row 1: 2 followers  at (-D, ±D)
//   Row 2: 3 followers  at (-2D, -2D), (-2D, 0), (-2D, +2D)
//   Row 3: 4 followers  at (-3D, -3D), (-3D, -D), (-3D, +D), (-3D, +3D)
//   …row r has (r+1) slots, spread ±r·D laterally.
//
// Usage:
//   ./wingman [--cam N] [--calibrate] [--marker-size MM]
//             [--spacing MM] [--dist MM] [--speed PCT]
//
//   --dist MM   alias for --spacing: sets the row/column distance between robots.
//               Larger values spread the formation out; smaller values tighten it.
//
// Controls:
//   WASD       Drive leader (macOS multi-key)
//   s          Stop all
//   c          Re-calibrate homography
//   +/-        Formation spacing ±25 mm
//   q / ESC    Quit

// Must come before aruco_tracker.h on macOS: pylon's api_autoconf.h defines
// `interface` as a macro which corrupts CoreGraphics/IOHIDTypes.h struct members.
#ifdef __APPLE__
#include <CoreGraphics/CoreGraphics.h>
#else
#include "evdev_keys.h"
#endif

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
#include <numeric>
#include <set>
#include <chrono>
#include <thread>
#include <mutex>

#include <unistd.h>
#include <termios.h>

#ifdef __APPLE__
static constexpr CGKeyCode kKey_A = 0x00;
static constexpr CGKeyCode kKey_S = 0x01;
static constexpr CGKeyCode kKey_D = 0x02;
static constexpr CGKeyCode kKey_W = 0x0D;
#else
static constexpr int kKey_A = KEY_A;
static constexpr int kKey_S = KEY_S;
static constexpr int kKey_D = KEY_D;
static constexpr int kKey_W = KEY_W;
static EvdevKeyboard g_keyboard;
#endif

// ── Tunables ──────────────────────────────────────────────────────────────────

static constexpr float DEFAULT_SPACING_MM = 250.0f;  // row/column step (mm)
static constexpr float DEFAULT_SPEED_PCT  = 70.0f;

static constexpr float K_DIST     = 0.40f;   // forward proportional gain
static constexpr float K_ANGLE    = 0.45f;   // turn proportional gain
static constexpr float MAX_SPEED  = 51.7f;   // mm/s full scale
static constexpr float MAX_TURN   = 24.0f;   // motor units, turn cap
static constexpr float ARRIVAL_MM = 55.0f;   // stop correcting within this radius

static constexpr int8_t LEADER_SPEED = 18;   // WASD throttle magnitude

// ── Shared state ──────────────────────────────────────────────────────────────

static constexpr int MAX_ROBOTS = SC_MAX_ROBOTS;

static volatile bool g_running = true;

// SwarmClient instance shared by the vision thread, the heartbeat thread and
// the input thread. All access (setSpeed/registerRobot/flush/connect/poll)
// must hold g_swarmMutex.
static SwarmClient g_swarm;
static std::mutex  g_swarmMutex;

// Motors array written by both the control thread (leader slot) and the vision
// thread (follower slots). Always access under g_motorMutex.
static int8_t     g_motors[MAX_ROBOTS][2] = {};
static std::mutex g_motorMutex;

// Set once after registration, then read-only.
static volatile int g_leaderId = -1;

static void onSignal(int) { g_running = false; }

static bool tryHub() {
    std::lock_guard<std::mutex> lk(g_swarmMutex);
    return g_swarm.connect();
}

// Send a single-robot frame for the leader immediately, without touching follower state.
// Called from the CGEventTap callback so it must be low-latency.
static void sendLeader(int lid, int8_t L, int8_t R) {
    std::lock_guard<std::mutex> lk(g_swarmMutex);
    g_swarm.setSpeed((uint8_t)lid, L, R);
    g_swarm.flush();
}

// Snapshot g_motors (under g_motorMutex) then write to hub (under g_swarmMutex).
// Safe to call from any thread without holding either lock beforehand.
static void sendSwarm() {
    int8_t snap[MAX_ROBOTS][2];
    { std::lock_guard<std::mutex> lk(g_motorMutex); memcpy(snap, g_motors, sizeof(snap)); }

    std::lock_guard<std::mutex> lk(g_swarmMutex);
    for (int i = 0; i < MAX_ROBOTS; i++) g_swarm.setSpeed((uint8_t)i, snap[i][0], snap[i][1]);
    g_swarm.flush();
}

// ── Helpers ────────────────────────────────────────────────────────────────────

static float normAngle(float a) {
    while (a >  180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

// ── Formation geometry ─────────────────────────────────────────────────────────
//
// Slot index → offset in leader-local frame (forward = +x, left = +y).
//
//  Row r (1-based) contains (r+1) slots:
//    • x offset : -r * spacing
//    • y offset : evenly spread from -r*spacing to +r*spacing
//
//  Cumulative slots before row r: r*(r-1)/2   [rows 1..r-1 each had 2..r slots]
//  Slot k within row r: y = (-r + k * 2.0/r ) * spacing  (when r > 0)

struct SlotOffset { float dx, dy; };

static SlotOffset slotOffset(int idx, float spacing) {
    // find which row this slot falls in
    int row = 1;
    int rowStart = 0;
    while (rowStart + row + 1 <= idx) {
        rowStart += row + 1;
        row++;
    }
    int col     = idx - rowStart;   // 0 .. row
    int numCols = row + 1;

    float dx = -row  * spacing;
    float dy = (numCols > 1)
               ? (-row + col * (2.f * row / (numCols - 1))) * spacing
               : 0.f;
    return {dx, dy};
}

// Convert leader-local offset to world coordinates given leader pose.
static cv::Point2f localToWorld(const RobotPose& leader, float ldx, float ldy) {
    float yawRad = leader.yaw * (float)M_PI / 180.f;
    float cosY = cosf(yawRad), sinY = sinf(yawRad);
    return {
        leader.x + ldx * cosY - ldy * sinY,
        leader.y + ldx * sinY + ldy * cosY
    };
}

// ── Slot assignment ────────────────────────────────────────────────────────────
// Anti-crossing assignment: sort robots and slots by lateral position in the
// leader's local frame, then match in order.  Two paths cross iff one robot is
// to the left of another but assigned to a slot that is to the right — sorting
// both lists by the same axis makes that impossible.
//
// assignNewRobots is used after registration to slot in any newly-seen robot
// without disturbing existing assignments.

// Project world point into leader-local lateral (y) coordinate.
static float leaderLateral(const RobotPose& leader, float wx, float wy) {
    float yawRad = leader.yaw * (float)M_PI / 180.f;
    float dx = wx - leader.x, dy = wy - leader.y;
    return -sinf(yawRad) * dx + cosf(yawRad) * dy;
}

// Assign 'ids' to 'slots' (both given as index lists) using lateral-sort matching.
static void assignByLateral(
    const std::vector<int>& ids,
    const std::vector<int>& slots,
    const std::unordered_map<int,RobotPose>& poses,
    const RobotPose& leader,
    float spacing,
    std::unordered_map<int,int>& out)
{
    int N = (int)ids.size();
    if (N == 0 || (int)slots.size() < N) return;

    std::vector<int> sortedIds = ids;
    std::sort(sortedIds.begin(), sortedIds.end(), [&](int a, int b) {
        const auto& pa = poses.at(a);
        const auto& pb = poses.at(b);
        return leaderLateral(leader, pa.x, pa.y) < leaderLateral(leader, pb.x, pb.y);
    });

    std::vector<int> sortedSlots = slots;
    std::sort(sortedSlots.begin(), sortedSlots.end(), [&](int a, int b) {
        return slotOffset(a, spacing).dy < slotOffset(b, spacing).dy;
    });

    for (int i = 0; i < N; i++)
        out[sortedIds[i]] = sortedSlots[i];
}

// Build initial assignment for all followers at registration time.
static std::unordered_map<int,int> assignSlots(
    const std::vector<int>& followerIds,
    const std::unordered_map<int,RobotPose>& poses,
    const RobotPose& leader,
    float spacing)
{
    int N = (int)followerIds.size();
    std::unordered_map<int,int> result;
    std::vector<int> slots(N);
    std::iota(slots.begin(), slots.end(), 0);
    assignByLateral(followerIds, slots, poses, leader, spacing, result);
    return result;
}

// Add any robots in 'followerIds' not yet in 'assignment' to the nearest free slot.
static void assignNewRobots(
    const std::vector<int>& followerIds,
    const std::unordered_map<int,RobotPose>& poses,
    const RobotPose& leader,
    float spacing,
    std::unordered_map<int,int>& assignment)
{
    std::vector<int> newIds;
    for (int id : followerIds)
        if (!assignment.count(id)) newIds.push_back(id);
    if (newIds.empty()) return;

    // Collect occupied slots.
    std::set<int> used;
    for (auto& [id, s] : assignment) used.insert(s);

    // Free slots up to the total size needed.
    int total = (int)assignment.size() + (int)newIds.size();
    std::vector<int> freeSlots;
    for (int s = 0; s < total; s++)
        if (!used.count(s)) freeSlots.push_back(s);

    assignByLateral(newIds, freeSlots, poses, leader, spacing, assignment);
}

// ── Calibration ────────────────────────────────────────────────────────────────

static cv::Mat  g_H;
static bool     g_hasH = false;
static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";

static cv::Point2f worldToPixel(cv::Point2f w) {
    if (!g_hasH) return w;
    cv::Mat Hinv = g_H.inv();
    std::vector<cv::Point2f> src = {w}, dst;
    cv::perspectiveTransform(src, dst, Hinv);
    return dst[0];
}

struct CalibState { std::vector<cv::Point2f> pts; bool done = false; };
static void onCalibMouse(int event, int x, int y, int, void* ud) {
    auto* s = (CalibState*)ud;
    if (event == cv::EVENT_LBUTTONDOWN && s->pts.size() < 4) {
        s->pts.push_back({(float)x, (float)y});
        printf("  corner %d: (%d, %d)\n", (int)s->pts.size(), x, y);
        if (s->pts.size() == 4) s->done = true;
    }
}

static bool runCalibration(ArucoTracker& tracker) {
    printf("\nCalibration: click 4 arena corners (TL TR BR BL)\n");
    for (int i = 0; i < 100 && !tracker.update(); i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    cv::Mat frame = tracker.debugFrame().clone();
    cv::namedWindow("Calibration", cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow("Calibration", frame.cols, frame.rows);
    CalibState cs;
    cv::setMouseCallback("Calibration", onCalibMouse, &cs);

    while (!cs.done) {
        cv::Mat disp = frame.clone();
        for (auto& pt : cs.pts) cv::circle(disp, pt, 8, {0,0,255}, -1);
        tracker.drawText(disp,
            "Click TL TR BR BL  " + std::to_string(cs.pts.size()) + "/4",
            {10, 40}, 20, {0,255,0});
        cv::imshow("Calibration", disp);
        if (cv::waitKey(30) == 27) { cv::destroyWindow("Calibration"); return false; }
    }
    cv::destroyWindow("Calibration");

    printf("Arena width height mm (e.g. 800 600): ");
    float W = 0, H = 0;
    if (scanf("%f %f", &W, &H) != 2 || W <= 0 || H <= 0) { printf("Invalid.\n"); return false; }

    std::vector<cv::Point2f> world = {{0,0},{W,0},{W,H},{0,H}};
    g_H    = cv::findHomography(cs.pts, world);
    g_hasH = !g_H.empty();
    if (g_hasH) {
        tracker.setHomography(cs.pts, world);
        tracker.saveHomography(HOMOGRAPHY_FILE);
        printf("Saved to %s\n", HOMOGRAPHY_FILE);
    }
    return g_hasH;
}

// ── Leader WASD input — CGEventTap (event-driven, zero polling latency) ───────
//
// Instead of polling CGEventSourceKeyState on a 1 ms sleep loop, we install a
// system-wide CGEventTap.  The OS delivers a callback the instant a key is
// pressed or released — no scheduler jitter, no busy-waiting.
//
// A separate heartbeat thread sends a swarm frame every 50 ms so the robot
// watchdog never times out even when no key event fires.
//
// Requires Accessibility permission (System Preferences → Privacy → Accessibility).

static std::atomic<bool> g_keyW{false}, g_keyA{false};
static std::atomic<bool> g_keyS{false}, g_keyD{false};

// Recompute leader motors from current key state and send immediately.
static void applyLeaderMotors() {
    int lid = g_leaderId;
    if (lid < 0) return;

    bool fwd = g_keyW.load(), bwd = g_keyS.load();
    bool lft = g_keyA.load(), rgt = g_keyD.load();

    int8_t throttle = (fwd && !bwd) ?  LEADER_SPEED
                    : (!fwd && bwd)  ? -LEADER_SPEED : 0;
    int8_t steer    = (rgt && !lft) ?  (int8_t)(LEADER_SPEED * 0.5f)
                    : (!rgt && lft)  ? -(int8_t)(LEADER_SPEED * 0.5f) : 0;

    int L = (int)throttle + (int)steer;
    int R = (int)throttle - (int)steer;
    if (L >  100) L =  100; else if (L < -100) L = -100;
    if (R >  100) R =  100; else if (R < -100) R = -100;

    { std::lock_guard<std::mutex> lk(g_motorMutex);
      g_motors[lid][0] = (int8_t)L;
      g_motors[lid][1] = (int8_t)R; }

    sendLeader(lid, (int8_t)L, (int8_t)R);
}

#ifdef __APPLE__

static CGEventRef keyTapCallback(CGEventTapProxy /*proxy*/,
                                  CGEventType    type,
                                  CGEventRef     event,
                                  void*          /*ud*/)
{
    if (type == kCGEventTapDisabledByTimeout || type == kCGEventTapDisabledByUserInput)
        return event;   // let the tap re-enable itself via the run loop

    CGKeyCode code = (CGKeyCode)CGEventGetIntegerValueField(event, kCGKeyboardEventKeycode);
    bool down = (type == kCGEventKeyDown);

    bool changed = false;
    if      (code == kKey_W && g_keyW.exchange(down) != down) changed = true;
    else if (code == kKey_A && g_keyA.exchange(down) != down) changed = true;
    else if (code == kKey_S && g_keyS.exchange(down) != down) changed = true;
    else if (code == kKey_D && g_keyD.exchange(down) != down) changed = true;

    if (changed) applyLeaderMotors();

    return event;   // kCGEventTapOptionListenOnly — we never consume events
}

// Runs the CFRunLoop that drives the event tap.  Exits when g_running goes false.
static void runEventTap() {
    CGEventMask mask = CGEventMaskBit(kCGEventKeyDown) | CGEventMaskBit(kCGEventKeyUp);
    CFMachPortRef tap = CGEventTapCreate(
        kCGSessionEventTap,
        kCGHeadInsertEventTap,
        kCGEventTapOptionListenOnly,
        mask,
        keyTapCallback,
        nullptr);

    if (!tap) {
        fprintf(stderr,
            "[input] CGEventTapCreate failed — grant Accessibility permission "
            "(System Preferences → Privacy → Accessibility).\n"
            "[input] Falling back to swarm_controller-style polling.\n");
        // Fallback: poll at 1 ms just like the old code so the binary still works.
        int8_t prevL = 0, prevR = 0;
        while (g_running) {
            int lid = g_leaderId;
            if (lid >= 0) {
                bool fwd = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kKey_W);
                bool bwd = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kKey_S);
                bool lft = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kKey_A);
                bool rgt = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kKey_D);
                int8_t thr = (fwd && !bwd) ?  LEADER_SPEED : (!fwd && bwd) ? -LEADER_SPEED : 0;
                int8_t str = (rgt && !lft) ?  (int8_t)(LEADER_SPEED*0.5f)
                           : (!rgt && lft) ? -(int8_t)(LEADER_SPEED*0.5f) : 0;
                int L = (int)thr+(int)str, R = (int)thr-(int)str;
                if (L> 100) L= 100; else if (L<-100) L=-100;
                if (R> 100) R= 100; else if (R<-100) R=-100;
                if ((int8_t)L != prevL || (int8_t)R != prevR) {
                    prevL=(int8_t)L; prevR=(int8_t)R;
                    { std::lock_guard<std::mutex> lk(g_motorMutex);
                      g_motors[lid][0]=prevL; g_motors[lid][1]=prevR; }
                    sendSwarm();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return;
    }

    CFRunLoopSourceRef src =
        CFMachPortCreateRunLoopSource(kCFAllocatorDefault, tap, 0);
    CFRunLoopAddSource(CFRunLoopGetCurrent(), src, kCFRunLoopDefaultMode);
    CGEventTapEnable(tap, true);

    // Spin the run loop in short slices so we can check g_running.
    while (g_running)
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.05, false);

    CGEventTapEnable(tap, false);
    CFRunLoopRemoveSource(CFRunLoopGetCurrent(), src, kCFRunLoopDefaultMode);
    CFRelease(src);
    CFRelease(tap);
}

#else  // Linux: evdev polling, 2 ms interval — functionally equivalent to the macOS fallback
       // (XQueryKeymap only sees keys delivered to an X11 surface, which a Wayland
       // session never does for native clients like the terminal wingman runs in;
       // evdev reads the kernel input layer directly and works under both).

static void runEventTap() {
    if (g_keyboard.open() == 0) {
        fprintf(stderr, "[input] no readable keyboard in /dev/input — WASD disabled. "
                        "Add yourself to the 'input' group (sudo usermod -aG input $USER, "
                        "then log out and back in).\n");
        return;
    }
    while (g_running) {
        bool w = g_keyboard.down(kKey_W);
        bool a = g_keyboard.down(kKey_A);
        bool s = g_keyboard.down(kKey_S);
        bool d = g_keyboard.down(kKey_D);
        bool changed = (w != g_keyW.exchange(w)) | (a != g_keyA.exchange(a)) |
                       (s != g_keyS.exchange(s)) | (d != g_keyD.exchange(d));
        if (changed) applyLeaderMotors();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    g_keyboard.close();
}

#endif  // __APPLE__

// ── Heartbeat thread ──────────────────────────────────────────────────────────
// Sends a swarm frame every 50 ms so the robot watchdog never starves.
static void runHeartbeat() {
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        sendSwarm();
    }
}

// ── Main ───────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    std::string serial, ip;
    float spacingMm  = DEFAULT_SPACING_MM;
    float speedPct   = DEFAULT_SPEED_PCT;
    bool  doCalib    = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--serial")      && i+1<argc) serial    = argv[++i];
        if (!strcmp(argv[i], "--ip")          && i+1<argc) ip        = argv[++i];
        if ((!strcmp(argv[i], "--spacing") || !strcmp(argv[i], "--dist")) && i+1<argc)
                                                           spacingMm = atof(argv[++i]);
        if (!strcmp(argv[i], "--speed")       && i+1<argc) speedPct  = atof(argv[++i]);
        if (!strcmp(argv[i], "--calibrate"))               doCalib   = true;
    }

    float speedMult = clampf(speedPct / 100.f, 0.05f, 2.0f);

    if (tryHub()) printf("[hub] Connected.\n");
    else          printf("[hub] Not available — will retry.\n");

    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    ArucoTracker tracker(cfg);
    if (!tracker.open()) { fprintf(stderr, "Could not open Basler camera.\n"); return 1; }
    // auto undist = std::make_unique<FisheyeUndistortPreprocessor>();
    // if (undist->load("fisheye_calib.yaml", tracker.frameSize())) tracker.prependPreprocessor(std::move(undist));

    printf("Camera open at %dx%d.\n",
           tracker.frameSize().width, tracker.frameSize().height);

    if (!doCalib && tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
        printf("Loaded homography from %s\n", HOMOGRAPHY_FILE);
    }
    if (doCalib && !runCalibration(tracker)) printf("Calibration skipped.\n");

    const char* WIN = "Wingman";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);

    std::unordered_map<int, std::chrono::steady_clock::time_point> robotLastSeen;
    auto lastHubRetry = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    auto lastFpsT     = std::chrono::steady_clock::now();
    int  frameCount   = 0;
    float fps         = 0.f;

    // ── Registration phase ────────────────────────────────────────────────────
    // Collect robots for ~3 s, then lock the lowest-ID as permanent leader and
    // compute a fixed slot assignment.  Slots are never reshuffled: if a robot
    // disappears its slot stays reserved; new arrivals get the nearest free slot.
    static constexpr int REGISTRATION_FRAMES = 90;  // ~3 s at 30 fps
    int  regFrames  = 0;
    bool registered = false;
    int  leaderId   = -1;   // local mirror of g_leaderId; set once at registration

    // Persistent robot→slot map.  Set at registration, only ever extended.
    std::unordered_map<int,int> assignment;

    printf("\nRegistering robots for ~3 s — make sure all robots are visible...\n");

    // ── Start input + heartbeat threads ──────────────────────────────────────
    // eventTapThread: CGEventTap fires immediately on key-down/up → zero latency.
    // heartbeatThread: sends a swarm frame every 50 ms to keep the watchdog alive.
    std::thread eventTapThread(runEventTap);
    std::thread heartbeatThread(runHeartbeat);

    while (g_running) {
        // ── Hub reconnect ─────────────────────────────────────────────────────
        {
            auto now = std::chrono::steady_clock::now();
            if (!g_swarm.isConnected() && std::chrono::duration<float>(now - lastHubRetry).count() >= 0.25f) {
                lastHubRetry = now;
                if (tryHub()) printf("[hub] Reconnected.\n");
            }
            std::lock_guard<std::mutex> lk(g_swarmMutex);
            g_swarm.poll();
        }

        // ── Vision update ─────────────────────────────────────────────────────
        // Do NOT sleep+continue here — that would starve WASD in the old code.
        // The control thread is independent, but we still want the vision loop
        // to tick quickly when no frame is available.
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        ++frameCount;
        {
            float dt = std::chrono::duration<float>(now - lastFpsT).count();
            if (dt >= 1.f) { fps = frameCount / dt; frameCount = 0; lastFpsT = now; }
        }

        // ── Build pose map ────────────────────────────────────────────────────
        std::unordered_map<int,RobotPose> poseById;
        for (auto& r : tracker.robots()) {
            if (r.id < 0 || r.id >= MAX_ROBOTS) continue;
            poseById[r.id] = r;
            robotLastSeen[r.id] = now;
            { std::lock_guard<std::mutex> lk(g_swarmMutex); g_swarm.registerRobot((uint8_t)r.id); }
        }

        // ── Registration phase: collect robots, then lock leader ──────────────
        if (!registered) {
            for (auto& [id, _] : poseById)
                if (leaderId < 0 || id < leaderId) leaderId = id;

            if (++regFrames >= REGISTRATION_FRAMES) {
                registered = true;
                g_leaderId = leaderId;   // publish to control thread
                if (leaderId >= 0) {
                    // Build the fixed assignment once using all currently visible followers.
                    std::vector<int> initFollowers;
                    for (auto& [id, _] : poseById)
                        if (id != leaderId) initFollowers.push_back(id);
                    std::sort(initFollowers.begin(), initFollowers.end());
                    if (!initFollowers.empty() && poseById.count(leaderId))
                        assignment = assignSlots(initFollowers, poseById,
                                                 poseById.at(leaderId), spacingMm);
                    printf("[wingman] Registration done. Leader locked: Robot %d  (%d robots seen)\n",
                           leaderId, (int)poseById.size());
                } else {
                    printf("[wingman] Registration done but no robots visible. Waiting...\n");
                }
            }
        }

        // ── Build follower list (all visible robots except leader) ────────────
        std::vector<int> followerIds;
        for (auto& [id, _] : poseById)
            if (id != leaderId) followerIds.push_back(id);
        std::sort(followerIds.begin(), followerIds.end());

        // ── Slot assignment ───────────────────────────────────────────────────
        // Assignment is persistent — only extend it for newly-seen robots.
        // Slots of disappeared robots stay reserved so no reshuffling occurs.
        if (registered && leaderId >= 0 && poseById.count(leaderId))
            assignNewRobots(followerIds, poseById, poseById.at(leaderId), spacingMm, assignment);

        // ── Follower controllers ──────────────────────────────────────────────
        // Write follower motor slots into g_motors under mutex.
        // The control thread owns the leader slot — we never touch it here.
        if (registered && leaderId >= 0 && poseById.count(leaderId)) {
            const RobotPose& leader = poseById.at(leaderId);
            std::lock_guard<std::mutex> lk(g_motorMutex);
            for (int id : followerIds) {
                if (!assignment.count(id)) { g_motors[id][0] = g_motors[id][1] = 0; continue; }

                int slotIdx = assignment.at(id);
                SlotOffset s = slotOffset(slotIdx, spacingMm);
                cv::Point2f tgt = localToWorld(leader, s.dx, s.dy);

                const RobotPose& r = poseById.at(id);
                float dx   = tgt.x - r.x;
                float dy   = tgt.y - r.y;
                float dist = sqrtf(dx*dx + dy*dy);

                if (dist < ARRIVAL_MM) {
                    g_motors[id][0] = g_motors[id][1] = 0;
                    continue;
                }

                float tgtAngle    = atan2f(dy, dx) * 180.f / (float)M_PI;
                float angleErr    = normAngle(tgtAngle - r.yaw);
                float angleErrRad = angleErr * (float)M_PI / 180.f;
                float maxSpd      = MAX_SPEED * speedMult;

                // Unicycle controller: cos(angleErr) naturally scales forward by
                // alignment — robot curves to target instead of stopping to spin.
                // brakeSc ramps speed to zero as dist approaches ARRIVAL_MM so the
                // robot doesn't overshoot the deadzone at full speed.
                float brakeSc = clampf((dist - ARRIVAL_MM) / (ARRIVAL_MM * 2.f), 0.f, 1.f);
                float forward = clampf(K_DIST * dist * cosf(angleErrRad), 0.f, maxSpd) * brakeSc;
                float turn    = clampf(K_ANGLE * angleErr, -MAX_TURN, MAX_TURN);

                g_motors[id][0] = (int8_t)clampf(forward + turn, -100.f, 100.f);
                g_motors[id][1] = (int8_t)clampf(forward - turn, -100.f, 100.f);
            }
        }

        // ── Watchdog: zero followers that have been invisible for > 1 s ───────
        {
            std::lock_guard<std::mutex> lk(g_motorMutex);
            for (int id = 0; id < MAX_ROBOTS; id++) {
                if (id == leaderId) continue;   // control thread owns this slot
                if (poseById.count(id)) continue;
                auto it = robotLastSeen.find(id);
                if (it == robotLastSeen.end()) continue;
                auto gapMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - it->second).count();
                if (gapMs > 1000) g_motors[id][0] = g_motors[id][1] = 0;
            }
        }

        // ── Draw HUD ──────────────────────────────────────────────────────────
        cv::Mat disp = tracker.debugFrame().clone();

        // Formation slot targets + leader indicator
        if (registered && leaderId >= 0) {
            // Only draw formation overlays when leader is currently visible.
            if (poseById.count(leaderId)) {
                const RobotPose& leader = poseById.at(leaderId);

                for (int id : followerIds) {
                    if (!assignment.count(id)) continue;
                    int slotIdx = assignment.at(id);
                    SlotOffset s = slotOffset(slotIdx, spacingMm);
                    cv::Point2f tgtW = localToWorld(leader, s.dx, s.dy);
                    cv::Point2f tgtP = worldToPixel(tgtW);

                    cv::drawMarker(disp, tgtP, {0, 220, 120},
                                   cv::MARKER_DIAMOND, 16, 2, cv::LINE_AA);

                    if (poseById.count(id)) {
                        const RobotPose& r = poseById.at(id);
                        float dx = tgtW.x - r.x, dy = tgtW.y - r.y;
                        if (sqrtf(dx*dx + dy*dy) > ARRIVAL_MM)
                            cv::arrowedLine(disp, {(int)r.px, (int)r.py}, tgtP,
                                            {255, 160, 30}, 2, cv::LINE_AA, 0, 0.12f);
                    }

                    tracker.drawText(disp, "S" + std::to_string(slotIdx),
                        tgtP + cv::Point2f(8, -8), 16, {0, 220, 120});
                }

                // Leader ring
                cv::circle(disp, {(int)leader.px, (int)leader.py},
                           28, {0, 120, 255}, 3, cv::LINE_AA);
                tracker.drawText(disp, "LEAD",
                    cv::Point((int)(leader.px + 30), (int)(leader.py - 10)),
                    18, {0, 120, 255});
            }
        }

        // ── HUD overlay (top-left info panel) ─────────────────────────────────
        {
            std::string leaderStr;
            cv::Scalar  leaderCol;
            if (!registered) {
                leaderStr = DemoHud::fmt("Registering... %d/%d", regFrames, REGISTRATION_FRAMES);
                leaderCol = DemoHud::COL_OK;
            } else if (leaderId >= 0) {
                leaderStr = "Leader: Robot " + std::to_string(leaderId) + "  (WASD)";
                leaderCol = DemoHud::COL_WARN;
            } else {
                leaderStr = "Leader: none visible";
                leaderCol = DemoHud::COL_TEXT;
            }

            DemoHud hud;
            hud.title("WINGMAN FORMATION");
            hud.row({leaderStr}, leaderCol);
            hud.row({DemoHud::fmt("Spacing: %d mm   Robots: %d",
                     (int)spacingMm, (int)poseById.size())});
            hud.row("loop_fps", DemoHud::fmt("%.1f", fps), DemoHud::COL_OK);
            hud.row("Hub", g_swarm.isConnected() ? "OK" : "DISCONNECTED",
                    g_swarm.isConnected() ? DemoHud::COL_OK : DemoHud::COL_BAD);

            hud.header({"ID", "Role", "Battery", "Latency", "Mot-L", "Mot-R"});
            for (auto& [id, r] : poseById) {
                bool isLeader = (id == leaderId);
                std::string role = isLeader ? "LEADER" : ("Flw S" + std::to_string(
                    assignment.count(id) ? assignment.at(id) : -1));
                int8_t mL, mR;
                { std::lock_guard<std::mutex> lk(g_motorMutex); mL = g_motors[id][0]; mR = g_motors[id][1]; }
                const auto& ss = g_swarm.robotState((uint8_t)id);
                hud.row({
                    DemoHud::fmt("%d", id), role,
                    ss.known ? DemoHud::formatBattery(ss.battery) : "--",
                    ss.known ? DemoHud::formatLatency(ss.latencyUs) : "--",
                    DemoHud::fmt("%+d", (int)mL), DemoHud::fmt("%+d", (int)mR),
                }, isLeader ? DemoHud::COL_WARN : DemoHud::COL_TEXT);
            }
            hud.draw(disp, {10, 10});

            // Controls hint at bottom
            tracker.drawText(disp, "WASD: lead   +/-: spacing   c: calib   q: quit",
                {10, disp.rows - 16}, 14, {100, 100, 100});
        }

        cv::imshow(WIN, disp);

        // ── Keyboard input ────────────────────────────────────────────────────
        int key = cv::waitKey(1) & 0xFF;
        switch (key) {
            case 'q': case 27: g_running = false; break;
            case 'c':
                if (runCalibration(tracker)) {
                    cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
                    if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
                }
                break;
            case '+': case '=':
                spacingMm += 25.f;
                printf("[formation] Spacing: %.0f mm\n", spacingMm);
                break;
            case '-':
                spacingMm = std::max(50.f, spacingMm - 25.f);
                printf("[formation] Spacing: %.0f mm\n", spacingMm);
                break;
        }
    }

    // ── Shutdown ──────────────────────────────────────────────────────────────
    eventTapThread.join();
    heartbeatThread.join();
    { std::lock_guard<std::mutex> lk(g_motorMutex); memset(g_motors, 0, sizeof(g_motors)); }
    sendSwarm();
    cv::destroyAllWindows();
    printf("Wingman stopped.\n");
    return 0;
}
