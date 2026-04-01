// circle_demo.cpp — Circle formation swarm controller
// Usage: ./circle_demo [--cam N] [--calibrate] [--marker-size MM]
//                      [--radius MM] [--min-gap MM] [--orbit-speed DEG_S]
//                      [--speed PCT]
// Controls: left-click = set circle centre, s = stop, c = calibrate,
//           0-9 = select robot (again to deselect),
//           +/- = radius ±25mm  (or robot speed ±10% when robot selected),
//           t = toggle orbit tracking,  [ / ] = orbit speed ±5 deg/s,
//           q/Esc = quit

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
#include <mutex>
#include <algorithm>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <glob.h>

// ── Controller tunables ───────────────────────────────────────────────────────

static constexpr float DEFAULT_RADIUS_MM      = 300.0f;
static constexpr float DEFAULT_MIN_GAP_MM     = 0.0f;   // min arc-chord distance between neighbours
static constexpr float DEFAULT_ORBIT_SPEED    = 30.0f;  // deg/s, default orbit speed in tracking mode

static constexpr float K_DIST    = 0.40f;
static constexpr float K_ANGLE   = 0.40f;
static constexpr float K_RAD     = 0.35f;  // radial correction gain (mm/s per mm of error)
static constexpr float MAX_SPEED = 51.7f;  // +10% from 47
static constexpr float MAX_TURN  = 22.0f;
static constexpr float ARRIVAL_MM       = 60.0f;   // stop when within this distance of slot
static constexpr float SEND_INTERVAL_S  = 0.05f;

// ── Protocol ─────────────────────────────────────────────────────────────────

static constexpr uint8_t MAGIC_0   = 0xAA;
static constexpr uint8_t MAGIC_1   = 0x55;
static constexpr uint8_t MSG_SWARM = 0x10;
static constexpr int     MAX_ROBOTS = 32;

static uint8_t crc8(const uint8_t* d, uint8_t n) {
    uint8_t c = 0;
    for (uint8_t i = 0; i < n; i++) {
        c ^= d[i];
        for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 7 : (c << 1);
    }
    return c;
}
static void buildFrame(uint8_t* buf, uint8_t type, const uint8_t* payload, uint8_t plen) {
    buf[0] = MAGIC_0; buf[1] = MAGIC_1; buf[2] = type; buf[3] = plen;
    memcpy(&buf[4], payload, plen);
    buf[4 + plen] = crc8(&buf[2], plen + 2);
}

// ── Hub connection ────────────────────────────────────────────────────────────

static int  g_hubFd  = -1;
static volatile bool g_running = true;
static std::mutex g_hubMutex;
static void onSignal(int) { g_running = false; }

static int hubConnect() {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, "/tmp/swarm_hub.sock", sizeof(addr.sun_path) - 1);
    if (connect(fd, (sockaddr*)&addr, sizeof(addr)) < 0) { close(fd); return -1; }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    return fd;
}

static std::string pipeRead(const char* cmd) {
    FILE* fp = popen(cmd, "r");
    if (!fp) return {};
    std::string out; char buf[256];
    while (fgets(buf, sizeof(buf), fp)) out += buf;
    pclose(fp);
    return out;
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
        if (glob(p, 0, nullptr, &gl) == 0 && gl.gl_pathc > 0) {
            port = gl.gl_pathv[0]; globfree(&gl); break;
        }
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
    uint8_t frame[4 + MAX_ROBOTS * 3 + 1];
    buildFrame(frame, MSG_SWARM, payload, MAX_ROBOTS * 3);
    std::lock_guard<std::mutex> lock(g_hubMutex);
    ssize_t n = write(g_hubFd, frame, sizeof(frame));
    if (n < 0) {
        perror("[hub] write");
        close(g_hubFd);
        g_hubFd = -1;
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

static float normAngle(float a) {
    while (a >  180) a -= 360;
    while (a < -180) a += 360;
    return a;
}
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

// ── Calibration ───────────────────────────────────────────────────────────────

static cv::Mat  g_H;
static bool     g_hasH = false;
static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";
static const char* CIRCLE_FILE     = "/tmp/circle_demo.yml";

static std::unordered_map<int,float> g_robotSpeed;         // per-robot multiplier (forward-decl for save/load)

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

    cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
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
    // Persist per-robot speed overrides as a flat [id, mult, id, mult, ...] sequence.
    std::vector<float> speedEntries;
    for (auto& [id, spd] : g_robotSpeed) {
        speedEntries.push_back((float)id);
        speedEntries.push_back(spd);
    }
    fs << "robot_speeds" << speedEntries;
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
    // Restore per-robot speed overrides.
    std::vector<float> speedEntries;
    fs["robot_speeds"] >> speedEntries;
    for (size_t i = 0; i + 1 < speedEntries.size(); i += 2) {
        int id = (int)speedEntries[i];
        if (id >= 0 && id < MAX_ROBOTS)
            g_robotSpeed[id] = speedEntries[i + 1];
    }
    printf("[circle] Loaded: centre=(%.0f,%.0f) radius=%.0fmm gap=%.0fmm orbit=%.0fdeg/s\n",
           c.centre.x, c.centre.y, c.radius, c.minGapMm, c.orbitSpeed);
    return true;
}

// Returns sorted robot IDs and their current angles on the circle (degrees, 0..360).
// angleDeg[i] = atan2(robot.y - centre.y, robot.x - centre.x) wrapped to [0,360).
static std::vector<std::pair<int,float>> robotAngles(
    const std::vector<RobotPose>& robots, cv::Point2f c)
{
    std::vector<std::pair<int,float>> out;
    for (auto& r : robots) {
        if (r.id < 0 || r.id >= MAX_ROBOTS) continue;
        float a = (float)(std::atan2(r.y - c.y, r.x - c.x) * 180.0 / M_PI);
        if (a < 0) a += 360.f;
        out.push_back({r.id, a});
    }
    return out;
}

// Assign N evenly-spaced slots to N robots, minimising total angular travel.
// Returns map: robotId → target angle (degrees).
static std::unordered_map<int,float> assignSlots(
    const std::vector<std::pair<int,float>>& rAngles)
{
    int N = (int)rAngles.size();
    if (N == 0) return {};

    // Sort robots by their current angle so the greedy step is well-behaved.
    auto sorted = rAngles;
    std::sort(sorted.begin(), sorted.end(),
              [](auto& a, auto& b){ return a.second < b.second; });

    // Try all N rotational offsets of the slot grid; pick the one with
    // minimum sum of |robot_angle - slot_angle|.
    float step = 360.f / N;
    float bestCost = 1e30f;
    int   bestOffset = 0;

    for (int off = 0; off < N; ++off) {
        float cost = 0;
        for (int i = 0; i < N; ++i) {
            float slot = fmodf((off + i) * step, 360.f);
            float err  = fabsf(normAngle(sorted[i].second - slot));
            cost += err;
        }
        if (cost < bestCost) { bestCost = cost; bestOffset = off; }
    }

    std::unordered_map<int,float> out;
    for (int i = 0; i < N; ++i) {
        float slot = fmodf((bestOffset + i) * step, 360.f);
        out[sorted[i].first] = slot;
    }
    return out;
}

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

// ── Per-robot speed ───────────────────────────────────────────────────────────

static float g_defaultSpeedMult = 0.70f;                   // set by --speed; default -30%
// g_robotSpeed declared above (needed by saveCircle/loadCircle)
static int   g_selectedRobot = -1;                         // -1 = none selected
static bool  g_allSelected   = false;                      // '.' = all selected → +/- changes default speed

static float robotSpeed(int id) {
    auto it = g_robotSpeed.find(id);
    return it != g_robotSpeed.end() ? it->second : g_defaultSpeedMult;
}

// ── GoPro auto-detection (identical to vision_controller) ────────────────────

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
        fprintf(stderr, "[camera] No GoPro found.\n");
        return -1;
    }
    printf("[camera] GoPro: \"%s\"\n", goProName.c_str());

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
        if (name == goProName) { printf("[camera] index %d\n", idx); return idx; }
    }
    fprintf(stderr, "[camera] \"%s\" not in AVFoundation list.\n", goProName.c_str());
    return -1;
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    int   camIndex    = -1;
    float markerMm    = 60.0f;
    float radiusMm    = -1.f;   // -1 = use saved / default
    float minGapMm    = -1.f;
    float orbitSpeed  = -1.f;
    float speedPct    = 70.f;   // global default speed, percent of MAX_SPEED
    bool  doCalibrate = false;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--cam")          && i+1<argc) camIndex   = atoi(argv[++i]);
        if (!strcmp(argv[i], "--marker-size")  && i+1<argc) markerMm   = atof(argv[++i]);
        if (!strcmp(argv[i], "--radius")       && i+1<argc) radiusMm   = atof(argv[++i]);
        if (!strcmp(argv[i], "--min-gap")      && i+1<argc) minGapMm   = atof(argv[++i]);
        if (!strcmp(argv[i], "--orbit-speed")  && i+1<argc) orbitSpeed = atof(argv[++i]);
        if (!strcmp(argv[i], "--speed")        && i+1<argc) speedPct   = atof(argv[++i]);
        if (!strcmp(argv[i], "--calibrate"))                doCalibrate = true;
    }
    g_defaultSpeedMult = clampf(speedPct / 100.f, 0.05f, 2.0f);

    if (camIndex < 0) {
        camIndex = findGoProIndex();
        if (camIndex < 0) {
            fprintf(stderr, "GoPro not found. Pass --cam N.\n");
            return 1;
        }
    }

    if (tryHub()) printf("[hub] Connected.\n");
    else          printf("[hub] Not available — will retry.\n");

    ArucoTracker tracker(camIndex, markerMm);
    if (!tracker.open()) { fprintf(stderr, "Could not open camera %d.\n", camIndex); return 1; }
    printf("Camera %d open at %dx%d.\n", camIndex, tracker.frameSize().width, tracker.frameSize().height);

    if (!doCalibrate && tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        if (fs.isOpened()) { fs["H"] >> g_H; g_hasH = !g_H.empty(); }
        printf("Loaded homography from %s\n", HOMOGRAPHY_FILE);
    }
    if (doCalibrate && !runCalibration(tracker)) printf("Calibration skipped.\n");

    const char* WIN = "Circle Demo";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
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

    auto lastSend     = std::chrono::steady_clock::now();
    auto lastFpsT     = lastSend;
    auto lastHubRetry = lastSend - std::chrono::seconds(10);
    int  frameCount   = 0;
    float fps         = 0;

    std::unordered_map<int, std::chrono::steady_clock::time_point> robotLastSeen;

    printf("\nLeft-click = centre  +/- = radius  . = select all (then +/- = speed)  0-9 = select robot\nt = orbit  [ / ] = orbit speed  s = stop  c = calibrate  q = quit\n\n");

    while (g_running) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        ++frameCount;

        if (g_hubFd < 0 && std::chrono::duration<float>(now - lastHubRetry).count() >= 2.0f) {
            lastHubRetry = now;
            if (tryHub()) printf("[hub] Connected.\n");
        }

        float dt = std::chrono::duration<float>(now - lastFpsT).count();
        if (dt >= 1.0f) { fps = frameCount / dt; frameCount = 0; lastFpsT = now; }

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

        // ── Slot assignment (position mode only) ──────────────────────────────
        // In orbit mode slot angles are not used for control; we still compute
        // them so the HUD can draw the ideal equal-spacing markers.
        std::unordered_map<int, float> slotAngles;
        if (circle.centreSet && !poseById.empty()) {
            auto rAngles = robotAngles(tracker.robots(), circle.centre);
            slotAngles   = assignSlots(rAngles);
            enforceMinGap(slotAngles, circle.radius, circle.minGapMm);
        }

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
                // robotSpeed multiplies the cap (not the raw velocity) so it has a linear
                // effect across its full range regardless of how fast the orbit geometry wants
                // to run — consistent with position mode's "maxSpd = MAX_SPEED * robotSpeed".
                float omegaRad = fabsf(circle.orbitSpeed) * (float)M_PI / 180.f;
                float vTan = std::min(omegaRad * circle.radius, MAX_SPEED) * robotSpeed(id);

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

                float forward = clampf(vMag, 0.f, MAX_SPEED) * headingSc;
                float turn    = clampf(K_ANGLE * angleErr, -MAX_TURN, MAX_TURN);

                motors[id][0] = (int8_t)clampf(forward + turn, -100, 100);
                motors[id][1] = (int8_t)clampf(forward - turn, -100, 100);

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
                float maxSpd    = MAX_SPEED * robotSpeed(id);

                float forward = clampf(K_DIST * dist, 0.f, maxSpd) * headingSc * brakeSc;
                float turn    = clampf(K_ANGLE * angleErr, -MAX_TURN, MAX_TURN);

                motors[id][0] = (int8_t)clampf(forward + turn, -100, 100);
                motors[id][1] = (int8_t)clampf(forward - turn, -100, 100);
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
            sendSwarm(motors);
            lastSend = now;
            memcpy(lastMotors, motors, sizeof(motors));
        }

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
                float vTan = std::min(omegaRad * circle.radius, MAX_SPEED) * robotSpeed(id);
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

        // ── Per-robot speed labels + selection ring ───────────────────────────
        for (auto& [id, pose] : poseById) {
            float spd = robotSpeed(id);
            bool selected = (id == g_selectedRobot) || g_allSelected;
            if (selected)
                cv::circle(disp, {(int)pose.px,(int)pose.py}, 22, {0,255,255}, 2, cv::LINE_AA);
            if (spd != 1.0f || selected) {
                char buf[16]; snprintf(buf, sizeof(buf), "%.0f%%", spd * 100.f);
                tracker.drawText(disp, buf,
                    cv::Point2f(pose.px + 14, pose.py + 20), 16,
                    selected ? cv::Scalar(0,255,255) : cv::Scalar(180,180,180));
            }
        }

        // ── Status bar ────────────────────────────────────────────────────────
        char hudSel[48];
        if (g_allSelected)
            snprintf(hudSel, sizeof(hudSel), "SEL:all(%.0f%%)  +/-=spd",
                     g_defaultSpeedMult * 100.f);
        else if (g_selectedRobot >= 0)
            snprintf(hudSel, sizeof(hudSel), "SEL:%d(%.0f%%)  +/-=spd",
                     g_selectedRobot, robotSpeed(g_selectedRobot) * 100.f);
        else
            snprintf(hudSel, sizeof(hudSel), "SEL:none  +/-=radius");

        char hud[256];
        snprintf(hud, sizeof(hud),
            "FPS:%.0f  Robots:%d  R:%.0fmm  Gap:%.0fmm  %s  HUB:%s  %s  %s",
            fps, (int)poseById.size(),
            circle.radius, circle.minGapMm,
            g_hasH ? "world" : "pixels",
            g_hubFd >= 0 ? "OK" : "INACTIVE",
            circle.tracking
                ? cv::format("ORBIT %.0fdeg/s", circle.orbitSpeed).c_str()
                : "POSITION",
            hudSel);
        cv::Scalar hudCol = g_hubFd >= 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 120, 255);
        tracker.drawText(disp, hud, {10, disp.rows - 19}, 18, hudCol);

        cv::imshow(WIN, disp);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 's') {
            memset(motors, 0, sizeof(motors));
            memset(lastMotors, 0, sizeof(lastMotors));
            sendSwarm(motors);
            printf("Stopped.\n");
        }
        if (key == 'c') runCalibration(tracker);
        if (key == '+' || key == '=') {
            if (g_allSelected) {
                g_defaultSpeedMult = clampf(g_defaultSpeedMult + 0.1f, 0.05f, 2.0f);
                printf("All speed: %.0f%%\n", g_defaultSpeedMult * 100.f);
            } else if (g_selectedRobot >= 0) {
                float s = clampf(robotSpeed(g_selectedRobot) + 0.1f, 0.05f, 2.0f);
                g_robotSpeed[g_selectedRobot] = s;
                printf("Robot %d speed: %.0f%%\n", g_selectedRobot, s * 100.f);
            } else {
                circle.radius += 25.f;
                printf("Radius: %.0f mm\n", circle.radius);
                saveCircle(circle);
            }
        }
        if (key == '-') {
            if (g_allSelected) {
                g_defaultSpeedMult = clampf(g_defaultSpeedMult - 0.1f, 0.05f, 2.0f);
                printf("All speed: %.0f%%\n", g_defaultSpeedMult * 100.f);
            } else if (g_selectedRobot >= 0) {
                float s = clampf(robotSpeed(g_selectedRobot) - 0.1f, 0.05f, 2.0f);
                g_robotSpeed[g_selectedRobot] = s;
                printf("Robot %d speed: %.0f%%\n", g_selectedRobot, s * 100.f);
            } else {
                circle.radius = std::max(50.f, circle.radius - 25.f);
                printf("Radius: %.0f mm\n", circle.radius);
                saveCircle(circle);
            }
        }
        if (key >= '0' && key <= '9') {
            int id = key - '0';
            if (g_selectedRobot == id) {
                g_selectedRobot = -1;
                printf("Deselected robot %d\n", id);
            } else {
                g_selectedRobot = id;
                g_allSelected   = false;
                printf("Selected robot %d  (speed %.0f%%)\n", id, robotSpeed(id) * 100.f);
            }
        }
        if (key == '.') {
            g_allSelected   = !g_allSelected;
            g_selectedRobot = -1;
            printf(g_allSelected ? "All selected  (speed %.0f%%)  +/-=speed\n"
                                 : "Deselected all\n", g_defaultSpeedMult * 100.f);
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
    sendSwarm(motors);
    if (g_hubFd >= 0) close(g_hubFd);
    cv::destroyAllWindows();
    printf("Stopped.\n");
    return 0;
}
