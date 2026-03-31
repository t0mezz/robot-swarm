// vision_controller.cpp — Vision-based swarm controller
// Usage: ./vision_controller [--cam N] [--calibrate] [--marker-size MM]
// Controls: left-click = goal all, right-click = goal selected, 0-9 select,
//           s stop, c calibrate, q/Esc quit

#include "aruco_tracker.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <glob.h>

#ifdef __APPLE__
#include <CoreGraphics/CoreGraphics.h>
static constexpr CGKeyCode kKey_A = 0x00;
static constexpr CGKeyCode kKey_S = 0x01;
static constexpr CGKeyCode kKey_D = 0x02;
static constexpr CGKeyCode kKey_W = 0x0D;
static inline bool keyDown(CGKeyCode code) {
    return CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, code);
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
static std::mutex           g_hubMutex;
static constexpr int8_t WASD_THROTTLE = 30;
static constexpr int8_t WASD_STEER    = 15;

// ── GoPro auto-detection ─────────────────────────────────────────────────────

static std::string pipeRead(const char* cmd) {
    FILE* fp = popen(cmd, "r");
    if (!fp) return {};
    std::string out; char buf[256];
    while (fgets(buf, sizeof(buf), fp)) out += buf;
    pclose(fp);
    return out;
}

static std::string jsonStrField(const std::string& json, const std::string& key, size_t from = 0) {
    std::string needle = "\"" + key + "\"";
    size_t kp = json.find(needle, from); if (kp == std::string::npos) return {};
    size_t colon = json.find(':', kp + needle.size()); if (colon == std::string::npos) return {};
    size_t q1 = json.find('"', colon + 1); if (q1 == std::string::npos) return {};
    size_t q2 = json.find('"', q1 + 1);   if (q2 == std::string::npos) return {};
    return json.substr(q1 + 1, q2 - q1 - 1);
}

static int findGoProIndex() {
    std::string profOut = pipeRead("system_profiler SPCameraDataType -json 2>/dev/null");
    std::string goProName;
    size_t pos = 0;
    while (true) {
        size_t heroPos = profOut.find("HERO", pos);
        if (heroPos == std::string::npos) break;
        size_t objStart = profOut.rfind('{', heroPos);
        if (objStart != std::string::npos) {
            std::string name = jsonStrField(profOut, "_name", objStart);
            if (!name.empty()) { goProName = name; break; }
        }
        pos = heroPos + 4;
    }
    if (goProName.empty()) {
        fprintf(stderr, "[camera] No GoPro found (no HERO model-id in system_profiler).\n");
        return -1;
    }
    printf("[camera] GoPro display name: \"%s\"\n", goProName.c_str());

    std::string ffOut = pipeRead("ffmpeg -f avfoundation -list_devices true -i '' 2>&1");
    bool inVideo = false;
    std::istringstream ss(ffOut);
    std::string line;
    while (std::getline(ss, line)) {
        if (line.find("AVFoundation video devices") != std::string::npos) { inVideo = true;  continue; }
        if (line.find("AVFoundation audio devices") != std::string::npos) { inVideo = false; continue; }
        if (!inVideo) continue;
        auto bracket = line.find('['), close = line.find(']', bracket);
        if (bracket == std::string::npos || close == std::string::npos) continue;
        int idx = std::stoi(line.substr(bracket + 1, close - bracket - 1));
        std::string name = line.substr(close + 2);
        while (!name.empty() && (name.back()=='\n'||name.back()=='\r'||name.back()==' ')) name.pop_back();
        printf("[camera]   [%d] %s%s\n", idx, name.c_str(), name == goProName ? "  ◄" : "");
        if (name == goProName) { printf("[camera] index %d\n", idx); return idx; }
    }
    fprintf(stderr, "[camera] \"%s\" not in AVFoundation list.\n", goProName.c_str());
    return -1;
}

// ── Protocol ─────────────────────────────────────────────────────────────────

static constexpr uint8_t MAGIC_0   = 0xAA;
static constexpr uint8_t MAGIC_1   = 0x55;
static constexpr uint8_t MSG_SWARM = 0x10;
static constexpr int     MAX_ROBOTS = 32;

static uint8_t crc8(const uint8_t* d, uint8_t n) {
    uint8_t c = 0;
    for (uint8_t i=0;i<n;i++){c^=d[i];for(uint8_t b=0;b<8;b++)c=(c&0x80)?(c<<1)^7:(c<<1);}
    return c;
}
static void buildFrame(uint8_t* buf, uint8_t type, const uint8_t* payload, uint8_t plen) {
    buf[0]=MAGIC_0; buf[1]=MAGIC_1; buf[2]=type; buf[3]=plen;
    memcpy(&buf[4], payload, plen);
    buf[4+plen] = crc8(&buf[2], plen+2);
}

// ── Hub connection ────────────────────────────────────────────────────────────

static int  g_hubFd  = -1;
static volatile bool g_running = true;
static std::unordered_map<int, std::chrono::steady_clock::time_point> g_robotLastSeen;
static void onSignal(int) { g_running = false; }

static int hubConnect() {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, "/tmp/swarm_hub.sock", sizeof(addr.sun_path)-1);
    if (connect(fd, (sockaddr*)&addr, sizeof(addr)) < 0) { close(fd); return -1; }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    return fd;
}

static bool tryHub() {
    g_hubFd = hubConnect();
    if (g_hubFd >= 0) return true;

    glob_t gl{};
    const char* patterns[] = {
        "/dev/tty.usbmodem*", "/dev/tty.usbserial*", "/dev/ttyUSB*", "/dev/ttyACM*"
    };
    std::string port;
    for (auto* p : patterns) {
        if (glob(p, 0, nullptr, &gl) == 0 && gl.gl_pathc > 0) { port = gl.gl_pathv[0]; globfree(&gl); break; }
        globfree(&gl);
    }
    if (port.empty()) return false;

    printf("[hub] Launching swarm_hub on %s\n", port.c_str());
    if (fork() == 0) {
        execl("./swarm_hub", "./swarm_hub", port.c_str(), nullptr);
        execl("/usr/local/bin/swarm_hub", "swarm_hub", port.c_str(), nullptr);
        _exit(1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    g_hubFd = hubConnect();
    return g_hubFd >= 0;
}

static void sendSwarm(int8_t motors[MAX_ROBOTS][2]) {
    if (g_hubFd < 0) return;
    uint8_t payload[MAX_ROBOTS * 3];
    for (int i = 0; i < MAX_ROBOTS; ++i) {
        payload[i*3+0] = (uint8_t)i;
        payload[i*3+1] = (uint8_t)motors[i][0];
        payload[i*3+2] = (uint8_t)motors[i][1];
    }
    uint8_t frame[4 + MAX_ROBOTS*3 + 1];
    buildFrame(frame, MSG_SWARM, payload, MAX_ROBOTS * 3);
    ssize_t n;
    {
        std::lock_guard<std::mutex> lock(g_hubMutex);
        n = write(g_hubFd, frame, sizeof(frame));
    }
    if (n < 0) {
        perror("[hub] write");
        close(g_hubFd);
        g_hubFd = -1;
    }
}

// ── Controller ───────────────────────────────────────────────────────────────

static constexpr float K_DIST         = 0.40f;
static constexpr float K_ANGLE        = 0.40f;
static constexpr float MAX_SPEED      = 47.0f;   // -15%
static constexpr float MAX_TURN       = 22.0f;
static constexpr float ARRIVAL_MM     = 75.0f;
static constexpr float SEND_INTERVALS = 0.05f;

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
static int  g_speedLevel = 64; // 0..255, 255 => 4x speed
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
    for (int i = 0; i < 10; ++i) tracker.update();
    cv::Mat frame = tracker.debugFrame().clone();

    cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
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

#ifdef __APPLE__
static void wasdControlLoop() {
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
            if (lid >= 0) {
                std::lock_guard<std::mutex> lock(g_hubMutex);
                int fd = g_hubFd;
                if (fd >= 0) {
                    uint8_t payload[3] = { (uint8_t)lid, (uint8_t)newL, (uint8_t)newR };
                    uint8_t frame[8];
                    buildFrame(frame, MSG_SWARM, payload, 3);
                    write(fd, frame, 8);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
#endif

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    int   camIndex    = -1;
    float markerMm    = 60.0f;
    bool  doCalibrate = false;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--cam")         && i+1<argc) camIndex    = atoi(argv[++i]);
        if (!strcmp(argv[i], "--marker-size") && i+1<argc) markerMm    = atof(argv[++i]);
        if (!strcmp(argv[i], "--calibrate"))               doCalibrate = true;
    }

    if (camIndex < 0) {
        camIndex = findGoProIndex();
        if (camIndex < 0) {
            fprintf(stderr, "GoPro not found. Connect GoPro + enable Webcam extension, or pass --cam N.\n");
            return 1;
        }
    }

    if (tryHub()) printf("[hub] Connected.\n");
    else          printf("[hub] Not available — will retry every 2s. "
                         "Start manually: ./swarm_hub /dev/tty.usbmodem*\n");

    ArucoTracker tracker(camIndex, markerMm);
    if (!tracker.open()) { fprintf(stderr, "Could not open camera %d.\n", camIndex); return 1; }
    printf("Camera %d open at %dx%d.\n", camIndex, tracker.frameSize().width, tracker.frameSize().height);

    if (!doCalibrate && tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
        printf("Loaded homography from %s\n", HOMOGRAPHY_FILE);
    }
    if (doCalibrate && !runCalibration(tracker)) printf("Calibration skipped.\n");

    const char* WIN = "Vision Controller";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::setMouseCallback(WIN, onMouse, nullptr);

    int8_t motors[MAX_ROBOTS][2] = {};
    static int8_t lastMotors[MAX_ROBOTS][2] = {};
    auto   lastSend     = std::chrono::steady_clock::now();
    auto   lastFpsT     = lastSend;
    auto   lastHubRetry = lastSend - std::chrono::seconds(10);
    int    frameCount   = 0;
    float  fps          = 0;

    printf("\nLeft-click=goal all  Right-click=goal selected  0-9=select  s=stop  c=calibrate  q=quit\n\n");

#ifdef __APPLE__
    std::thread wasdThread(wasdControlLoop);
#endif

    while (g_running) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        ++frameCount;

        if (g_hubFd < 0 && std::chrono::duration<float>(now-lastHubRetry).count() >= 2.0f) {
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
        }
        std::sort(robotIds.begin(), robotIds.end());

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
            float kDist   = isFollower ? 0.16f : K_DIST;   // -10%
            float maxSpd  = isFollower ? 34.0f : baseSpeed; // -10%
            float maxTurn = isFollower ? 15.0f : MAX_TURN;

            float tgtAngle    = (float)(std::atan2(dy, dx) * 180.0 / M_PI);
            float angleErr    = normAngle(tgtAngle - r.yaw);
            float headingNorm  = clampf(fabsf(angleErr) / 90.0f, 0.0f, 1.0f);
            float headingScale = 1.0f - headingNorm * headingNorm;  // inverse quadratic

            // Braking ramp for both leader and followers: speed decays linearly
            // to zero over a window equal to stopDist above the threshold.
            // Prevents the P-controller from arriving at speed and overshooting.
            float brakingScale = clampf((dist - stopDist) / stopDist, 0.0f, 1.0f);

            float forward = clampf(kDist * dist, 0.0f, maxSpd) * headingScale * brakingScale;
            float turn    = clampf(K_ANGLE * angleErr, -maxTurn, maxTurn);
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
        snprintf(hud, sizeof(hud), "FPS:%.0f  Robots:%d  Sel:%s  %s  HUB:%s  Speed:%d(%.2fx)  WASD:%s",
                 fps, (int)tracker.robots().size(),
                 g_selectedRobot < 0 ? "all" : std::to_string(g_selectedRobot).c_str(),
                 g_hasH ? "world" : "pixels",
                 g_hubFd >= 0 ? "OK" : "INACTIVE",
                 g_speedLevel, (float)g_speedLevel / 255.0f * 4.0f,
                 g_wasdActive ? "ON" : "off");
        cv::Scalar hudCol = g_hubFd >= 0 ? cv::Scalar(0,255,0) : cv::Scalar(0,120,255);
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
#ifdef __APPLE__
    wasdThread.join();
#endif
    memset(motors, 0, sizeof(motors));
    sendSwarm(motors);
    if (g_hubFd >= 0) close(g_hubFd);
    cv::destroyAllWindows();
    printf("Stopped.\n");
    return 0;
}
