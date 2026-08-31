// car_following.cpp — drive the swarm around a ring with the car-following
// models of the Sugiyama et al. (2007) experiment.
//
// Headless by default: no window, one status line per second. --debug opens
// the usual OpenCV view + DemoHud. --bridge serves the vendored NetLogo page
// (tools/car-following-models/) and follows its live model and slider values,
// so the simulation and the real robots run the same dynamics side by side.
//
// Usage:
//   ./car_following [--model NAME] [--speed-max M/S] [--car-size M]
//                   [--time-gap S] [--reaction-time S] [--sigma A]
//                   [--sim-length M] [--radius MM] [--centre X Y] [--dir cw|ccw]
//                   [--ring-file PATH] [--fit] [--robot-max-speed MM_S]
//                   [--time-scale K]
//                   [--bridge] [--port N] [--debug] [--serial SN] [--ip IP]
//                   [--count N]
//
// Debug keys: left-click = ring centre, +/- = radius +/-25mm,
//             f = fit the ring to the robots, s = stop, q/Esc = quit
//
// -- The ring is a saved fixture, not something inferred per run -------------
//
// The ring is loaded from /tmp/car_following_ring.yml the way circle_demo.cpp
// loads its circle, and every change to it — a flag, a click, a radius key —
// is written straight back, so the geometry is reproducible across runs.
//
// Earlier versions instead fitted the ring at startup to wherever the robots
// happened to be standing: the centroid of the detected poses, with their mean
// distance from it as the radius. Those poses only describe a circle if the
// robots are already on one, so a bad initial scatter — or robots still being
// placed, or a couple of them not yet detected — produced an off-centre,
// wrong-radius ring that the controller then fought for the whole run, and
// that silently rescaled the model as well (simPerMm divides by the radius).
//
// The fit is still there, but as an explicit one-shot (--fit, or 'f' in
// --debug) that saves its result like any other edit. With no saved ring at
// all, circle_demo's /tmp/circle_demo.yml is read instead — both tools drive
// the same physical circle.
//
// ── How the paper's road maps onto the arena ─────────────────────────────────
//
// The models are written in the paper's units: a 230m ring, 5m cars, 15m/s.
// Our ring is under a metre across, so positions and speeds are converted
// through one scale factor. What that factor has to preserve is *density* —
// the stop-and-go wave is a function of metres per vehicle, not of the ring's
// absolute size — so by default the physical ring is mapped to
// N * (230/22) simulated metres for however many robots are on it. Four
// robots then see the same spacing 22 cars see in the paper. --sim-length
// pins the virtual ring length instead, if you want to explore other
// densities.
//
// That factor maps *space* only: one simulated second was one real second, so
// a lap took as long here as it does on the paper's 230m ring — 4.6s for three
// robots on a 300mm ring, at a commanded motor value of 137. Dynamically
// similar, and far too fast to watch or to trust on hardware.
//
// --time-scale is the missing half of the mapping. K real seconds become one
// simulated second: measured speeds are scaled up by K, the model integrates a
// dt that is K times smaller, and the commanded speed is scaled back down by
// K. The loop stays self-consistent, so the trajectories and the wave are
// unchanged — the whole experiment just runs in slow motion. The heading
// controller is untouched by it and keeps working in real time.
//
// The heading controller underneath is circle_demo.cpp's orbit controller:
// a yaw feedforward carries the steady turn and a PD corrects the residual
// (see the long comment there for why pure feedback oscillates on a circle).
// The one difference is where the tangential speed comes from — per robot,
// from the car-following model, instead of one global orbit rate.
//
// That controller's velocity field is in *motor units*, not mm/s: circle_demo
// never converts, and K_FF_YAW / K_RAD were tuned on hardware against that
// scale. So the model's metres per second are converted all the way down to
// motor units (via --robot-max-speed) *before* the heading law sees them.
// Leaving the field in mm/s scales the feedforward up and the radial pull down
// by robotMaxMms/MOTOR_MAX, and since the feedforward is proportional to speed
// the robots then orbit at a radius that depends on how fast they are going —
// a fast one settles into a lane inside a slow one instead of catching it up.
//
// The other thing the model needs from the loop is that its speed stays a
// *state*. cfStep returns `in.speed + a*dt`, so feeding it the vision-measured
// speed each tick would hand the drive command that measurement back nearly
// unchanged — noise and all — and diverge on any bias. Vision corrects the
// state slowly instead; cfSyncAlpha() in car_following.h has the algebra.

#include "aruco_tracker.h"
#include "SwarmClient.h"
#include "DemoHud.h"
#include "car_following.h"
#include "http_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// ── Tunables ─────────────────────────────────────────────────────────────────

static constexpr float PAPER_SPACING_M  = 230.0f / 22.0f;  // metres per vehicle in the experiment
static constexpr float MODEL_DT_S       = 0.10f;   // the paper's integration step
// Slow-motion bounds. Below 1 the experiment runs faster than the paper, which
// is almost never wanted on hardware; the upper bound is where a model tick's
// travel shrinks into the tracker's own noise (see --time-scale in main).
static constexpr float TIME_SCALE_MIN  = 0.25f;
static constexpr float TIME_SCALE_MAX  = 50.0f;
static constexpr float TIME_SCALE_STEP = 1.25f;   // ',' / '.' in the debug view
static constexpr float SEND_INTERVAL_S  = 0.01f;
static constexpr float DEFAULT_RADIUS_MM = 300.0f; // fallback when nothing has been saved yet
static constexpr float RADIUS_STEP_MM   = 25.0f;   // +/- in the debug view, as in circle_demo
// Two poses always fit a "circle" through their midpoint; three is the least
// that can disagree with one, and so the least that says anything about where
// the ring is. A fit waits this long for that many robots to be detected
// before giving up — they are rarely all seen on the first frame.
static constexpr int   FIT_MIN_ROBOTS = 3;
static constexpr float FIT_WAIT_S     = 5.0f;
// How long a robot's last motor command is held while it is not detected.
// Single-frame dropouts are routine, and cutting the motors on each one would
// both jolt the ring and corrupt the speed measurement, which is taken from
// vision. Anything longer than this stops — the robot's own
// WATCHDOG_TIMEOUT_MS is the backstop if the link itself dies.
static constexpr float MOTOR_HOLD_S = 0.20f;

// Heading controller — carried over from circle_demo.cpp's orbit mode, where
// these were tuned on hardware.
static constexpr float K_ANGLE       = 0.45f;
static constexpr float K_YAW_D       = 0.15f;
static constexpr float K_FF_YAW      = 1.00f;
static constexpr float K_RAD         = 0.30f;   // radial pull back onto the ring
static constexpr float MOTOR_MAX     = 100.0f;
static constexpr float MAX_TURN      = 20.0f;
static constexpr float MAX_TURN_RATE = 120.0f;  // turn-units/s
static constexpr float YAW_TAU_S     = 0.50f;   // yaw low-pass time constant

static constexpr float DEG2RAD = (float)M_PI / 180.f;
static constexpr float RAD2DEG = 180.f / (float)M_PI;

static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";
static const char* RING_FILE       = "/tmp/car_following_ring.yml";
static const char* CIRCLE_FILE     = "/tmp/circle_demo.yml";   // circle_demo's, read as a fallback

static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

// --debug only. OpenCV runs the callback on its own thread, so a click is just
// recorded here and consumed by the main loop.
static bool      g_leftClick = false;
static cv::Point g_clickPt;
static void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) { g_leftClick = true; g_clickPt = {x, y}; }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

static float normAngle(float a) {
    while (a >  180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// ArucoTracker keeps its homography private and only ever maps pixel -> world
// internally, so both directions are re-read from the same file here and
// cached once (circle_demo recomputes the inverse per call). pixelToWorld
// turns a click into a ring centre; worldToPixel draws the ring.
static cv::Mat g_H, g_Hinv;

static cv::Point2f pixelToWorld(cv::Point2f px) {
    if (g_H.empty()) return px;
    std::vector<cv::Point2f> src = {px}, dst;
    cv::perspectiveTransform(src, dst, g_H);
    return dst[0];
}

static cv::Point worldToPixel(cv::Point2f w) {
    if (g_Hinv.empty()) return {(int)w.x, (int)w.y};
    std::vector<cv::Point2f> src = {w}, dst;
    cv::perspectiveTransform(src, dst, g_Hinv);
    return {(int)dst[0].x, (int)dst[0].y};
}

// Resolve a path next to the executable, so the tool works from any cwd —
// same trick as ArucoConfig::defaultConfigPath().
static std::string exeRelative(const char* rel) {
    char buf[4096];
#ifdef __APPLE__
    uint32_t size = sizeof(buf);
    if (_NSGetExecutablePath(buf, &size) != 0) return rel;
#else
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return rel;
    buf[n] = '\0';
#endif
    std::string p(buf);
    size_t slash = p.rfind('/');
    return slash == std::string::npos ? rel : p.substr(0, slash + 1) + rel;
}

// ── The ring ─────────────────────────────────────────────────────────────────
// Persisted in the same shape circle_demo.cpp saves its circle, so that file
// can be read directly and the two tools can share one calibration.

struct Ring {
    cv::Point2f centre{0.f, 0.f};       // world units (mm), or pixels without a homography
    float       radius = DEFAULT_RADIUS_MM;
    bool        centreSet = false;
};

static void saveRing(const Ring& ring, const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        fprintf(stderr, "[ring] could not save to %s\n", path.c_str());
        return;
    }
    fs << "cx"         << ring.centre.x
       << "cy"         << ring.centre.y
       << "centre_set" << (int)ring.centreSet
       << "radius"     << ring.radius;
    printf("[ring] saved centre=(%.0f, %.0f) radius=%.0f -> %s\n",
           ring.centre.x, ring.centre.y, ring.radius, path.c_str());
}

// Only plausible values are applied, so a stale or partial file (missing keys
// read back as 0) can't wipe out a good default.
static bool loadRing(Ring& ring, const std::string& path) {
    // Probed first: "no ring saved yet" is the normal first-run state, and
    // FileStorage logs a global [ERROR] for a missing file that reads like a
    // real fault.
    if (!std::ifstream(path).good()) return false;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    float cx = 0.f, cy = 0.f, r = 0.f;
    int   centreSet = 0;
    fs["cx"]         >> cx;
    fs["cy"]         >> cy;
    fs["centre_set"] >> centreSet;
    fs["radius"]     >> r;
    if (centreSet) { ring.centre = {cx, cy}; ring.centreSet = true; }
    if (r > 0.f)     ring.radius = r;
    printf("[ring] loaded centre=(%.0f, %.0f) radius=%.0f <- %s\n",
           ring.centre.x, ring.centre.y, ring.radius, path.c_str());
    return true;
}

// The old startup behaviour, kept as an explicit one-shot: centroid of the
// visible robots, their mean distance from it as the radius. Sound when the
// robots really are standing on the ring you want, misleading as a default —
// see the header — which is why it is opt-in and saved like a manual edit.
// The caller waits for FIT_MIN_ROBOTS poses before calling.
static bool fitRing(const std::unordered_map<int, RobotPose>& poses, Ring& ring) {
    if ((int)poses.size() < FIT_MIN_ROBOTS) return false;
    cv::Point2f c{0.f, 0.f};
    for (auto& [id, p] : poses) { c.x += p.x; c.y += p.y; }
    c.x /= poses.size(); c.y /= poses.size();

    float r = 0.f;
    for (auto& [id, p] : poses) r += std::hypot(p.x - c.x, p.y - c.y);
    r /= poses.size();
    if (r < 50.f) {
        printf("[ring] fit rejected: robots sit %.0f from their centroid — "
               "keeping the saved ring\n", r);
        return false;
    }
    ring.centre = c; ring.radius = r; ring.centreSet = true;
    return true;
}

// ── Live parameters from the page ────────────────────────────────────────────
// Body is one "name=value" per line, using the page's own widget labels.
// The page posts the full set every time, so this is idempotent — there is no
// partial-update state to keep in sync.
static void applyParams(const std::string& body, CfParams& p, CfModel& model) {
    size_t pos = 0;
    while (pos < body.size()) {
        size_t nl = body.find('\n', pos);
        if (nl == std::string::npos) nl = body.size();
        std::string line = body.substr(pos, nl - pos);
        pos = nl + 1;

        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq);
        std::string v = line.substr(eq + 1);
        while (!v.empty() && (v.back() == '\r' || v.back() == ' ')) v.pop_back();

        if      (k == "model")         cfModelFromName(v.c_str(), model);
        else if (k == "speed-max")     p.speedMax     = (float)atof(v.c_str());
        else if (k == "car-size")      p.carSize      = (float)atof(v.c_str());
        else if (k == "time-gap")      p.timeGap      = (float)atof(v.c_str());
        else if (k == "reaction-time") p.reactionTime = (float)atof(v.c_str());
        else if (k == "sigma")         p.sigma        = (float)atof(v.c_str());
    }
}

// ── Per-robot state ──────────────────────────────────────────────────────────

struct Car {
    // Which model tick this robot was last measured on. Speed is a difference
    // between consecutive ticks, so a robot that missed one must re-seed
    // rather than divide a whole dropout's worth of travel by one tick.
    uint64_t tickSeen = 0;
    float prevAng  = 0.f;  // deg around the ring, sampled at the last model tick
    float prevYaw  = 0.f;  // deg, sampled at the last model tick
    float yaw      = 0.f;  // deg, low-passed heading
    float yawRate  = 0.f;  // deg/s, held between model ticks
    float speed    = 0.f;  // simulated m/s, measured from the ring position
    // The vehicle's speed *state* — what the paper's v_n is, and what cfStep
    // integrates. Measured speed corrects it slowly rather than replacing it;
    // see cfSyncAlpha() in car_following.h for why replacing it diverges.
    float vModel   = 0.f;  // simulated m/s
    float vCmd     = 0.f;  // simulated m/s, the model's output (vModel, clamped)
    float gap      = 0.f;  // simulated m, clear distance to the predecessor
    float prevTurn = 0.f;  // slew-limited turn output
    bool  haveYaw  = false;
    bool  haveSpeed = false;  // a measurement has been taken, so vModel is seeded
    bool  known     = false;  // registered with the hub
    std::chrono::steady_clock::time_point lastSeen;
};

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);

    std::string serial, ip;
    CfParams    params;
    CfModel     model = CfModel::FVDM;   // the page's default chooser entry
    float  simLengthM  = -1.f;           // <0 = derive from the robot count
    float  argRadiusMm = -1.f;           // <0 = keep whatever the ring file holds
    float  argCentreX = 0.f, argCentreY = 0.f;
    bool   argCentre   = false;
    bool   fitAtStart  = false;
    std::string ringFile = RING_FILE;
    float  dirSign     = 1.f;            // +1 = counter-clockwise
    float  timeScale   = 1.f;            // >1 = slow motion; see the header
    float  robotMaxMms = 300.f;          // physical speed at motor command 100
    int    robotCount  = -1;
    bool   debug       = false;
    bool   bridge      = false;
    int    port        = 8770;

    for (int i = 1; i < argc; ++i) {
        auto arg = [&](const char* n) { return strcmp(argv[i], n) == 0 && i + 1 < argc; };
        if      (arg("--serial"))          serial      = argv[++i];
        else if (arg("--ip"))              ip          = argv[++i];
        else if (arg("--count"))           robotCount  = atoi(argv[++i]);
        else if (arg("--model")) {
            if (!cfModelFromName(argv[++i], model)) {
                fprintf(stderr, "unknown model: %s\n", argv[i]);
                return 2;
            }
        }
        else if (arg("--speed-max"))       params.speedMax     = (float)atof(argv[++i]);
        else if (arg("--car-size"))        params.carSize      = (float)atof(argv[++i]);
        else if (arg("--time-gap"))        params.timeGap      = (float)atof(argv[++i]);
        else if (arg("--reaction-time"))   params.reactionTime = (float)atof(argv[++i]);
        else if (arg("--sigma"))           params.sigma        = (float)atof(argv[++i]);
        else if (arg("--sim-length"))      simLengthM  = (float)atof(argv[++i]);
        else if (arg("--radius"))          argRadiusMm = (float)atof(argv[++i]);
        else if (arg("--ring-file"))       ringFile    = argv[++i];
        else if (arg("--robot-max-speed")) robotMaxMms = (float)atof(argv[++i]);
        else if (arg("--time-scale"))      timeScale   = (float)atof(argv[++i]);
        else if (arg("--port"))            port        = atoi(argv[++i]);
        else if (arg("--dir")) {
            const char* d = argv[++i];
            if      (strcmp(d, "cw")  == 0) dirSign = -1.f;
            else if (strcmp(d, "ccw") == 0) dirSign =  1.f;
            else { fprintf(stderr, "--dir must be cw or ccw, got: %s\n", d); return 2; }
        }
        else if (strcmp(argv[i], "--centre") == 0 && i + 2 < argc) {
            argCentreX = (float)atof(argv[++i]);
            argCentreY = (float)atof(argv[++i]);
            argCentre  = true;
        }
        else if (strcmp(argv[i], "--fit")    == 0) fitAtStart = true;
        else if (strcmp(argv[i], "--debug")  == 0) debug  = true;
        else if (strcmp(argv[i], "--bridge") == 0) bridge = true;
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("usage: %s [--model NAME] [--speed-max M/S] [--car-size M] [--time-gap S]\n"
                   "       [--reaction-time S] [--sigma A] [--sim-length M] [--radius MM]\n"
                   "       [--centre X Y] [--ring-file PATH] [--fit] [--dir cw|ccw]\n"
                   "       [--time-scale K] [--robot-max-speed MM_S] [--bridge] [--port N]\n"
                   "       [--debug] [--serial SN] [--ip IP] [--count N]\n\n"
                   "models: Reuschel Pipes OVM CF-OVM FVDM ATG IDM\n\n"
                   "--time-scale K runs the experiment in slow motion: K real seconds per\n"
                   "simulated second, same trajectories, K times slower. Start around 4-6 —\n"
                   "at K=1 the defaults ask for full throttle on a sub-metre ring.\n\n"
                   "The ring is read from %s (falling back to circle_demo's %s) and\n"
                   "re-saved whenever --radius/--centre/--fit or a debug-view edit changes it.\n",
                   argv[0], RING_FILE, CIRCLE_FILE);
            return 0;
        }
        else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 2; }
    }

    // Divides the model's speed on the way to the motors, so it cannot be zero.
    if (robotMaxMms <= 0.f) {
        fprintf(stderr, "--robot-max-speed must be positive\n");
        return 2;
    }
    if (timeScale < TIME_SCALE_MIN || timeScale > TIME_SCALE_MAX) {
        fprintf(stderr, "--time-scale must be between %.2f and %.0f\n",
                TIME_SCALE_MIN, TIME_SCALE_MAX);
        return 2;
    }

    SwarmClient swarm;
    printf(swarm.connect() ? "[hub] connected\n" : "[hub] not available — will retry\n");

    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    if (robotCount > 0)  cfg.robotCount   = robotCount;
    cfg.debugOverlay = debug;

    ArucoTracker tracker(cfg);
    if (!tracker.open()) { fprintf(stderr, "Could not open Basler camera.\n"); return 1; }
    printf("[vision] camera open at %dx%d\n",
           tracker.frameSize().width, tracker.frameSize().height);

    // World coordinates are what makes the ring geometry metric. Without a
    // homography everything below still works, but in pixels — so the scale
    // factor is calibrated against pixels rather than millimetres.
    if (tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        cv::Mat H;
        if (fs.isOpened()) fs["H"] >> H;
        if (!H.empty()) { g_H = H; g_Hinv = H.inv(); }
        printf("[vision] homography loaded from %s\n", HOMOGRAPHY_FILE);
    } else {
        printf("[vision] no homography at %s — working in pixels "
               "(run `circle_demo --calibrate` to create one)\n", HOMOGRAPHY_FILE);
    }

    // Defaults, then the saved ring, then whatever the flags override — so a
    // flag wins for this run *and* becomes the saved ring for the next one.
    Ring ring;
    ring.centre = pixelToWorld({tracker.frameSize().width  / 2.f,
                                tracker.frameSize().height / 2.f});
    if (!loadRing(ring, ringFile)) {
        if (ringFile == RING_FILE && loadRing(ring, CIRCLE_FILE))
            printf("[ring] (that is circle_demo's circle; it will be saved to %s "
                   "on the first change)\n", RING_FILE);
        else
            printf("[ring] nothing saved — using the arena centre and radius %.0f. "
                   "Set it with --centre/--radius, --fit, or a click in --debug.\n",
                   ring.radius);
    }
    bool ringEdited = false;
    if (argRadiusMm > 0.f) {
        ring.radius = argRadiusMm;
        ringEdited  = true;
    }
    if (argCentre) {
        ring.centre    = {argCentreX, argCentreY};
        ring.centreSet = true;
        ringEdited     = true;
    }
    if (ringEdited) saveRing(ring, ringFile);

    HttpBridge http;
    if (bridge) {
        std::string page = readFile(exeRelative(
            "../car-following-models/Experiment_by_Sugiyama_et_al.__2007_.html"));
        std::string js = readFile(exeRelative("../vision/car_following_bridge.js"));
        if (page.empty() || js.empty()) {
            fprintf(stderr, "[bridge] page or script missing — bridge disabled\n");
            bridge = false;
        } else {
            size_t at = page.rfind("</body>");
            if (at == std::string::npos) at = page.size();
            page.insert(at, "<script>\n" + js + "\n</script>\n");
            bridge = http.start(port, std::move(page));
            printf(bridge ? "[bridge] serving http://127.0.0.1:%d/\n"
                          : "[bridge] could not bind port %d\n", port);
        }
    }

    const char* WIN = "Car Following";
    if (debug) {
        cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
        cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
        cv::setMouseCallback(WIN, onMouse, nullptr);
        printf("[cf] left-click = ring centre  +/- = radius %.0f  f = fit ring to robots\n"
               "     , / . = time scale  s = stop  q/Esc = quit\n", RADIUS_STEP_MM);
    }

    std::unordered_map<int, Car> cars;
    int8_t motors[SC_MAX_ROBOTS][2] = {};

    auto sendMotors = [&]() {
        for (int id = 0; id < SC_MAX_ROBOTS; ++id)
            swarm.setSpeed((uint8_t)id, motors[id][0], motors[id][1]);
        swarm.flush();
    };

    std::mt19937                     rng(12345);
    std::normal_distribution<float>  gauss(0.f, 1.f);

    float    simPerMm   = 0.f;   // simulated metres per world unit
    bool     fitPending = fitAtStart;
    int      lastCount = -1;
    uint64_t tickNo    = 1;     // model ticks; 0 means "never measured"
    bool     paused    = false; // 's' in --debug; holds every motor at zero

    auto now       = std::chrono::steady_clock::now();
    auto lastModel = now, lastSend = now, lastStatus = now, lastHubRetry = now, lastFrame = now;
    auto fitSince  = now;   // when the pending fit started waiting for robots
    DemoHud::LoopFps loopFps;

    printf("[cf] model=%s  speed-max=%.1f  car-size=%.1f  time-gap=%.2f  "
           "reaction-time=%.2f  sigma=%.2f  time-scale=%.2gx\n",
           cfModelName(model), params.speedMax, params.carSize,
           params.timeGap, params.reactionTime, params.sigma, timeScale);
    if (timeScale <= 1.f)
        printf("[cf] time-scale is %.2gx — on a ring this small the models ask for "
               "near-full throttle. Try --time-scale 4.\n", timeScale);

    while (g_running) {
        if (!tracker.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        now = std::chrono::steady_clock::now();
        loopFps.tick();

        if (!swarm.isConnected() &&
            std::chrono::duration<float>(now - lastHubRetry).count() >= 2.0f) {
            lastHubRetry = now;
            if (swarm.connect()) printf("[hub] connected\n");
        }
        swarm.poll();

        if (bridge)
            for (const auto& body : http.poll()) applyParams(body, params, model);

        // ── Poses, low-passed heading ────────────────────────────────────────
        std::unordered_map<int, RobotPose> poseById;
        for (auto& r : tracker.robots())
            if (r.id >= 0 && r.id < SC_MAX_ROBOTS) poseById[r.id] = r;

        // The low-pass is specified as a time constant and converted with the
        // real frame interval, so it keeps the same memory in seconds whatever
        // rate the loop runs at (see YAW_TAU_S).
        float frameDt  = clampf(std::chrono::duration<float>(now - lastFrame).count(), 0.001f, 0.2f);
        lastFrame = now;
        float yawAlpha = frameDt / (YAW_TAU_S + frameDt);
        for (auto& [id, pose] : poseById) {
            Car& c = cars[id];
            c.lastSeen = now;
            if (!c.known) { swarm.registerRobot((uint8_t)id); c.known = true; }
            if (!c.haveYaw) { c.yaw = pose.yaw; c.prevYaw = pose.yaw; c.haveYaw = true; }
            else            c.yaw = normAngle(c.yaw + yawAlpha * normAngle(pose.yaw - c.yaw));
        }

        // Brief detection dropouts are normal, so state is held for a second
        // rather than dropped on the first missed frame — but not forever: a
        // stale entry would feed a phantom neighbour into someone's gap.
        for (auto it = cars.begin(); it != cars.end(); )
            it = std::chrono::duration<float>(now - it->second.lastSeen).count() > 1.0f
                     ? cars.erase(it) : std::next(it);

        // ── Ring edits ───────────────────────────────────────────────────────
        // A pending --fit / 'f' waits for enough robots to be detected; a
        // click is already in world units once it goes through the
        // homography. Both persist immediately, so the ring a run ends with is
        // the ring the next run starts on.
        if (fitPending) {
            if ((int)poseById.size() >= FIT_MIN_ROBOTS) {
                fitPending = false;
                if (fitRing(poseById, ring)) saveRing(ring, ringFile);
            } else if (std::chrono::duration<float>(now - fitSince).count() > FIT_WAIT_S) {
                fitPending = false;
                printf("[ring] fit gave up: %d of %d robots visible after %.0fs — "
                       "keeping the saved ring\n",
                       (int)poseById.size(), FIT_MIN_ROBOTS, FIT_WAIT_S);
            }
        }
        if (g_leftClick) {
            g_leftClick = false;
            ring.centre    = pixelToWorld({(float)g_clickPt.x, (float)g_clickPt.y});
            ring.centreSet = true;
            saveRing(ring, ringFile);
        }

        // ── Ring order ───────────────────────────────────────────────────────
        // Sorted counter-clockwise; the predecessor is the next robot in the
        // direction of travel.
        std::vector<std::pair<float,int>> byAngle;   // {angle deg, id}
        for (auto& [id, p] : poseById) {
            float a = atan2f(p.y - ring.centre.y, p.x - ring.centre.x) * RAD2DEG;
            byAngle.push_back({a < 0 ? a + 360.f : a, id});
        }
        std::sort(byAngle.begin(), byAngle.end());
        const int M = (int)byAngle.size();

        // Density: the virtual ring grows with the number of robots so the
        // metres-per-vehicle stay at the experiment's value (see the header).
        //
        // Counted over `cars` — the robots seen within the last second — and not
        // over this frame's detections. The road length divides into every gap
        // and every measured speed, so taking it from M would rescale the whole
        // experiment for as long as one marker is missed: four robots dropping
        // to three shortens the road by 25%, and every car sees its gap and its
        // own speed jump with it. Single-frame dropouts are routine (that is
        // what MOTOR_HOLD_S is for), so the population has to outlive them.
        // --count pins it outright when the arena's robot count is known.
        int roadCount = robotCount > 0 ? robotCount : (int)cars.size();
        if (roadCount > 0) {
            float lengthM = simLengthM > 0.f ? simLengthM : roadCount * PAPER_SPACING_M;
            simPerMm = lengthM / (2.f * (float)M_PI * ring.radius);
            if (roadCount != lastCount) {
                printf("[cf] %d robots → virtual ring %.1f m (%.2f m per vehicle)\n",
                       roadCount, lengthM, lengthM / roadCount);
                lastCount = roadCount;
            }
        }

        // ── Model tick ───────────────────────────────────────────────────────
        float modelDt = std::chrono::duration<float>(now - lastModel).count();
        if (M > 0 && modelDt >= MODEL_DT_S) {
            lastModel = now;
            modelDt   = clampf(modelDt, 0.02f, 0.5f);
            ++tickNo;

            // The tick still fires on a real-time interval; only the elapsed
            // time the model is told about is dilated. cfStep is a stateless
            // explicit-Euler step, so a dt smaller than the paper's 0.1s only
            // makes the integration finer.
            float simDt = modelDt / timeScale;

            // Pass 1: measure. Every robot's speed and gap must be read before
            // any model runs, or a robot would see its predecessor's
            // already-updated speed instead of this tick's.
            for (int i = 0; i < M; ++i) {
                auto [ang, id] = byAngle[i];
                Car& c = cars[id];

                if (c.tickSeen + 1 == tickNo) {
                    // Speed is in simulated metres per *simulated* second, so
                    // it divides by simDt — the same K the command below is
                    // divided by, which is what keeps the loop consistent.
                    c.speed   = dirSign * normAngle(ang - c.prevAng) * DEG2RAD * ring.radius
                                / simDt * simPerMm;
                    // First measurement seeds the model state; after that the
                    // measurement only corrects it (see pass 2).
                    if (!c.haveSpeed) { c.vModel = c.speed; c.haveSpeed = true; }
                    // Yaw rate feeds the heading PD, which is a real-time
                    // controller: real seconds, whatever the model's clock does.
                    c.yawRate = normAngle(c.yaw - c.prevYaw) / modelDt;
                }
                c.prevAng  = ang;
                c.prevYaw  = c.yaw;
                c.tickSeen = tickNo;

                // Angular gap to the predecessor, converted to a clear
                // bumper-to-bumper distance in simulated metres.
                float gapDeg = 360.f;
                if (M > 1) {
                    int j = (i + (dirSign > 0 ? 1 : M - 1)) % M;
                    gapDeg = dirSign > 0 ? byAngle[j].first - ang
                                         : ang - byAngle[j].first;
                    if (gapDeg < 0.f) gapDeg += 360.f;
                }
                c.gap = gapDeg * DEG2RAD * ring.radius * simPerMm - params.carSize;
            }

            // Pass 2: step every model off that snapshot.
            //
            // The model is stepped from its own speed state, with the measured
            // speed blended in slowly. Stepping straight from the measurement
            // instead — which is what cfStep's `in.speed + a*dt` shape invites —
            // closes a unity-gain loop around vision: it hands the drive command
            // the robot's own last-100ms speed back almost unchanged (the model
            // contributes only a*simDt, ~4% of it), so vision noise goes to the
            // motors unfiltered, and any bias at all makes the command run away
            // to speedMax. See cfSyncAlpha() in car_following.h for the algebra.
            float syncAlpha = cfSyncAlpha(simDt);
            for (int i = 0; i < M; ++i) {
                int  id   = byAngle[i].second;
                int  pred = byAngle[(i + (dirSign > 0 ? 1 : M - 1)) % M].second;
                Car& c    = cars[id];

                if (c.haveSpeed) c.vModel += syncAlpha * (c.speed - c.vModel);

                CfInput in{c.gap, c.vModel, cars[pred].vModel, cars[pred].gap};
                float v = cfStep(model, in, params, simDt,
                                 params.sigma > 0.f ? gauss(rng) : 0.f);
                // The robots only ever go forwards around the ring: a model
                // that undershoots into reverse would have them driving into
                // their follower. Clamped before it is stored, so the state
                // cannot wind up outside the range the robots can be asked for.
                c.vModel = clampf(v, 0.f, params.speedMax);
                c.vCmd   = c.vModel;
            }
        }

        // ── Servo each robot onto its commanded speed ────────────────────────
        if (std::chrono::duration<float>(now - lastSend).count() >= SEND_INTERVAL_S) {
            float sendDt = clampf(std::chrono::duration<float>(now - lastSend).count(),
                                  0.001f, 0.2f);
            lastSend = now;

            // Robots that have gone unseen for longer than the hold window
            // stop; everyone visible is recomputed below.
            for (int id = 0; id < SC_MAX_ROBOTS; ++id) {
                auto it = cars.find(id);
                if (paused || it == cars.end() ||
                    std::chrono::duration<float>(now - it->second.lastSeen).count() > MOTOR_HOLD_S)
                    motors[id][0] = motors[id][1] = 0;
            }

            for (auto& [id, pose] : poseById) {
                if (paused) break;
                Car& c = cars[id];
                if (simPerMm <= 0.f) { motors[id][0] = motors[id][1] = 0; continue; }

                float dx = pose.x - ring.centre.x, dy = pose.y - ring.centre.y;
                float distC = std::hypot(dx, dy);
                if (distC < 1.f) { motors[id][0] = motors[id][1] = 0; continue; }

                float rx = dx / distC,          ry = dy / distC;
                float tx = dirSign * -ry,       ty = dirSign * rx;

                // sim m/s -> world units per *real* second (undo the density
                // scale, then the time dilation), then -> motor units.
                //
                // That last conversion is the one that matters. circle_demo's
                // velocity field is in *motor units* — it never converts, it
                // feeds vTan straight to the motors as `forward` — and K_FF_YAW
                // and K_RAD were tuned on hardware against that scale. Carrying
                // the gains over while expressing the field in mm/s scales both
                // by robotMaxMms/MOTOR_MAX: the yaw feedforward over-commands by
                // that factor and the radial pull is weaker by it. The
                // feedforward is proportional to speed, so the error is too, and
                // robots settle onto a smaller circle the faster they are going
                // — a fast robot cutting inside a slow one on its own lane
                // rather than closing on it. Keep the field in motor units.
                float mmPerUnit = robotMaxMms / MOTOR_MAX;
                float vTan = std::min(c.vCmd / simPerMm / timeScale / mmPerUnit,
                                      MOTOR_MAX);
                float vRad = clampf(-K_RAD * (distC - ring.radius),
                                    -MOTOR_MAX * 0.5f, MOTOR_MAX * 0.5f);

                float vx = vTan * tx + vRad * rx;
                float vy = vTan * ty + vRad * ry;
                float vMag = std::hypot(vx, vy);
                if (vMag < 0.5f) { motors[id][0] = motors[id][1] = 0; continue; }

                float angleErr  = normAngle(atan2f(vy, vx) * RAD2DEG - c.yaw);
                float headingN  = clampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
                float headingSc = 1.f - headingN * headingN;

                // Feedforward carries the steady turn (v/R); feedback only
                // corrects the residual. Gated by headingSc so a robot that is
                // still spinning to align can't deadlock at zero output — see
                // circle_demo.cpp for the full derivation. vTan is capped at
                // MOTOR_MAX above before it gets here, for the reason given
                // there: feeding forward a speed the robot cannot reach
                // overdrives the turn and brings the oscillation back.
                float ffOmega = dirSign * (vTan / ring.radius) * RAD2DEG;
                float dErr    = clampf(ffOmega - c.yawRate, -300.f, 300.f);

                float forward = clampf(vMag, 0.f, MOTOR_MAX) * headingSc;
                float turnTgt = clampf(K_FF_YAW * ffOmega * headingSc
                                       + K_ANGLE * angleErr + K_YAW_D * dErr,
                                       -MAX_TURN, MAX_TURN);
                float maxStep = MAX_TURN_RATE * sendDt;
                float turn    = clampf(turnTgt, c.prevTurn - maxStep, c.prevTurn + maxStep);
                c.prevTurn    = turn;

                motors[id][0] = (int8_t)clampf(forward + turn, -MOTOR_MAX, MOTOR_MAX);
                motors[id][1] = (int8_t)clampf(forward - turn, -MOTOR_MAX, MOTOR_MAX);
            }
            sendMotors();
        }

        // ── Report ───────────────────────────────────────────────────────────
        if (!debug) {
            if (std::chrono::duration<float>(now - lastStatus).count() >= 1.0f) {
                lastStatus = now;
                printf("[cf] %-8s loop:%3.0f  robots:%d  %.2gx  hub:%s",
                       cfModelName(model), loopFps.fps(), M, timeScale,
                       swarm.isConnected() ? "ok" : "--");
                for (auto& [ang, id] : byAngle)
                    printf("  %d:%.1fm/s@%.1fm", id, cars[id].vCmd, cars[id].gap);
                printf("\n");
                fflush(stdout);
            }
            continue;
        }

        cv::Mat disp = tracker.debugFrame().clone();
        if (disp.empty()) continue;   // no overlay frame yet; nothing to show or key off

        // The ring is defined in world units; project it back through the
        // homography by transforming points on it rather than assuming pixels
        // and world units share a scale.
        std::vector<cv::Point> poly;
        for (int a = 0; a < 360; a += 4)
            poly.push_back(worldToPixel({ring.centre.x + ring.radius * cosf(a * DEG2RAD),
                                         ring.centre.y + ring.radius * sinf(a * DEG2RAD)}));
        cv::polylines(disp, poly, true, {0, 200, 255}, 2, cv::LINE_AA);

        // The centre is what a click moves, so it is drawn as a target rather
        // than left implicit in the ring.
        cv::Point cpx = worldToPixel(ring.centre);
        cv::drawMarker(disp, cpx, {0, 200, 255}, cv::MARKER_CROSS, 22, 2, cv::LINE_AA);
        tracker.drawText(disp,
                         DemoHud::fmt("ring r=%.0f  c=(%.0f, %.0f)",
                                      ring.radius, ring.centre.x, ring.centre.y),
                         {cpx.x + 14, cpx.y - 6}, 18, {0, 200, 255});
        tracker.drawText(disp,
                         "click = centre   +/- = radius   f = fit   , / . = time scale   "
                         "s = stop   q = quit",
                         {12, disp.rows - 16}, 18, {0, 200, 255});

        for (auto& [ang, id] : byAngle) {
            const Car& c = cars[id];
            tracker.drawText(disp, DemoHud::fmt("%.1fm/s  gap %.1fm", c.vCmd, c.gap),
                             {(int)poseById[id].px + 12, (int)poseById[id].py - 12},
                             18, {0, 255, 128});
        }

        DemoHud hud;
        hud.title(DemoHud::fmt("loop_fps:%.0f  %s  robots:%d  ring:%.0f  %.2gx  HUB:%s%s",
                               loopFps.fps(), cfModelName(model), M, ring.radius, timeScale,
                               swarm.isConnected() ? "OK" : "INACTIVE",
                               bridge ? "  BRIDGE" : ""),
                  swarm.isConnected() ? DemoHud::COL_OK : DemoHud::COL_BAD);
        hud.header({"ID", "Gap m", "v m/s", "Mot-L", "Mot-R", "Battery"});
        for (auto& [ang, id] : byAngle) {
            const auto& ss = swarm.robotState((uint8_t)id);
            hud.row({DemoHud::fmt("%d", id),
                     DemoHud::fmt("%.1f", cars[id].gap),
                     DemoHud::fmt("%.2f", cars[id].vCmd),
                     DemoHud::fmt("%+d", (int)motors[id][0]),
                     DemoHud::fmt("%+d", (int)motors[id][1]),
                     ss.known ? DemoHud::formatBattery(ss.battery) : "--"},
                    DemoHud::COL_OK);
        }
        hud.drawTopRight(disp);
        cv::imshow(WIN, disp);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 'f') {
            fitPending = true; fitSince = now;
            printf("[ring] fitting to the visible robots\n");
        }
        if (key == '+' || key == '=') {
            ring.radius += RADIUS_STEP_MM;
            saveRing(ring, ringFile);
        }
        if (key == '-' || key == '_') {
            ring.radius = std::max(50.f, ring.radius - RADIUS_STEP_MM);
            saveRing(ring, ringFile);
        }
        if (key == '.' || key == '>') {
            timeScale = clampf(timeScale * TIME_SCALE_STEP, TIME_SCALE_MIN, TIME_SCALE_MAX);
            printf("[cf] time scale %.2gx\n", timeScale);
        }
        if (key == ',' || key == '<') {
            timeScale = clampf(timeScale / TIME_SCALE_STEP, TIME_SCALE_MIN, TIME_SCALE_MAX);
            printf("[cf] time scale %.2gx\n", timeScale);
        }
        if (key == 's') {
            paused = !paused;
            printf("[cf] %s\n", paused ? "paused" : "running");
        }
    }

    memset(motors, 0, sizeof(motors));
    sendMotors();
    if (debug) cv::destroyAllWindows();
    printf("[cf] stopped\n");
    return 0;
}
