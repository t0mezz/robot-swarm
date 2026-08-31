// car_following.cpp — drive the swarm around a ring with the car-following
// models of the Sugiyama et al. (2007) experiment.
//
// Headless by default: no window, one status line per second. --debug opens
// the usual OpenCV view + DemoHud. --bridge serves the vendored NetLogo page
// (tools/car-following-models/) and follows its live model, slider values and
// run state, so the simulation and the real robots run the same dynamics side
// by side.
//
// Usage:
//   ./car_following [--model NAME] [--speed-max M/S] [--car-size M]
//                   [--time-gap S] [--reaction-time S] [--sigma A]
//                   [--sim-length M] [--radius MM] [--centre X Y] [--dir cw|ccw]
//                   [--ring-file PATH] [--fit] [--robot-max-speed MM_S]
//                   [--time-scale K] [--start] [--buffer-b B] [--buffer-id ID]
//                   [--bridge] [--port N] [--debug] [--serial SN] [--ip IP]
//                   [--count N]
//
// ── Setup, cue, run ──────────────────────────────────────────────────────────
//
// The tool comes up in *setup*: the camera, the hub and the ring are live and
// every motor is held at zero, so robots can be placed and the ring dialled in
// without anything driving off. A run starts on a cue and continues until it
// is stopped. The cue can come from any of:
//
//   • the NetLogo page's "Move" button, with --bridge
//   • space in the --debug view, or 's' to stop
//   • a line on stdin when headless: <enter> or "go" starts, "s"/"stop" stops,
//     "q" quits
//   • --start, which latches the cue at launch for scripted runs
//
// A cue is latched rather than obeyed on the spot: the run begins on the first
// frame where the hub is connected, the roster has settled and the ring has a
// radius, and says once what it is waiting for if it cannot start yet. A stop
// returns the models to rest, so the next run begins from standstill the way
// the experiment's own setup does.
//
// A second cue, *align*, drives the robots to evenly spaced slots on the ring
// instead — what the NetLogo page's own "Setup" button asks for, so pressing
// it (or 'a' in --debug, or "a"/"align" on stdin) first rests the robots the
// same way a stop does and then spaces them out, finishing on its own once
// every visible robot is in its slot and leaving the tool back in setup, ready
// for a clean "Move". It is a position controller, not the car-following
// model: CfRing::computeAlignTargets() picks the rotation of an evenly spaced
// slot pattern that minimizes total travel, and the per-robot loop below
// steers each one's raw angular error onto that slot the same way it steers
// the model's tangential speed during a run — same heading controller, same
// radial pull onto the ring, only the source of the tangential term differs.
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
// densities, and --count pins N.
//
// N is the *settled* roster, not the count of robots detected in the current
// frame (see CfRingConfig::settleS): a single dropped detection would
// otherwise move the virtual ring length by 1/N and rescale every gap and
// every speed the models see for that one frame.
//
// That factor maps *space* only: one simulated second was one real second, so
// a lap took as long here as it does on the paper's 230m ring — 4.6s for three
// robots on a 300mm ring, at a commanded motor value of 137. Dynamically
// similar, and far too fast to watch or to trust on hardware.
//
// --time-scale is the missing half of the mapping. K real seconds become one
// simulated second: the model integrates a dt that is K times smaller, the
// measured speed is reported in the same dilated units, and the commanded
// speed is scaled back down by K. The loop stays self-consistent, so the
// trajectories and the wave are unchanged — the whole experiment just runs in
// slow motion. The heading controller is untouched by it and keeps working in
// real time.
//
// ── Cooperative buffering ───────────────────────────────────────────────────
//
// --buffer-id names one robot as the buffering vehicle and --buffer-b sets how
// much room it takes: it drives at B times the nominal time gap while the rest
// of the ring gives up (N-B)/(N-1) of theirs, so the mean is unchanged and the
// experiment stays at the density it was set up for. See the long comment in
// lib/CarFollowing/car_following.h — including why Pipes cannot buffer.
//
// Two guards that only matter on hardware, not on the reference page. Both
// live in CfRing::step(), with the ring's own live vehicle count and roster:
//
//   * B is capped at cfMaxBuffering(N) — where the followers are down to half
//     the nominal gap. The page can afford its fixed slider max of 10 because
//     it always has 20 vehicles; three robots on the arena ring cannot.
//   * Buffering is applied only while the buffering robot is actually on the
//     ring. If it is lifted off or drops out of the tracker, the compensation
//     would otherwise shrink every remaining robot's gap with nothing holding
//     the space open — tighter than the baseline, which is the wrong way to
//     fail.
//
// Turning B up is not monotonically better under every model, because the
// compensation is also tightening the other N-1. Under IDM — the model the
// strategy was published for — it is: on a 20-vehicle ring the speed spread
// falls steadily as B rises. Under FVDM and CF-OVM a large B destabilises the
// followers before the buffer can absorb anything, so it is worth sweeping B
// rather than assuming more is better.
//
// The heading controller underneath is circle_demo.cpp's orbit controller:
// a yaw feedforward carries the steady turn and a PD corrects the residual
// (see the long comment there for why pure feedback oscillates on a circle).
// The one difference is where the tangential speed comes from — per robot,
// from the car-following model, instead of one global orbit rate. Its inputs
// are circle_demo's too: the *raw* pose yaw for the heading error, and a
// sample-and-held rate for the D-term. An earlier version fed the controller a
// half-second EMA of the yaw instead, which on a circle — where the true
// heading rotates continuously at v/R — lags by about tau*v/R and so hands the
// P-term a standing error (~9 deg at 100 mm/s on a 300 mm ring) that it steers
// out of a robot that was already pointing the right way.
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
// The ring bookkeeping itself — order, gaps, roster, scale, the models' own
// speed state, and the run-state machine — lives in lib/CarFollowing/ring.h,
// free of OpenCV and unit-tested; this file is vision, control and I/O.

#include "aruco_tracker.h"
#include "SwarmClient.h"
#include "DemoHud.h"
#include "car_following.h"
#include "ring.h"
#include "http_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <poll.h>
#include <unistd.h>

// ── Tunables ─────────────────────────────────────────────────────────────────

static constexpr float PAPER_SPACING_M  = 230.0f / 22.0f;  // metres per vehicle in the experiment
static constexpr float MODEL_DT_S       = 0.10f;   // the paper's integration step
// Slow-motion bounds. Below 1 the experiment runs faster than the paper, which
// is almost never wanted on hardware; the upper bound is where a model tick's
// travel shrinks into the tracker's own noise (see --time-scale in main).
static constexpr float TIME_SCALE_MIN  = 0.25f;
static constexpr float TIME_SCALE_MAX  = 50.0f;
static constexpr float TIME_SCALE_STEP = 1.25f;   // ',' / '.' in the debug view
static constexpr float CONTROL_INTERVAL_S = 0.01f;
// The motor frame is only written when a command actually changed, plus this
// keepalive so the robots' own WATCHDOG_TIMEOUT_MS (1s, lib/SwarmProtocol/
// hardware.h) never expires. In setup — and any time the ring is coasting on
// unchanged commands — that is 10 frames a second instead of 100.
static constexpr float MOTOR_KEEPALIVE_S = 0.10f;
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
// jolt the ring. Anything longer than this stops — the robot's own
// WATCHDOG_TIMEOUT_MS is the backstop if the link itself dies. The robot keeps
// its *place* on the ring for longer than this (CfRingConfig::holdS), so its
// follower still brakes for it.
static constexpr float MOTOR_HOLD_S = 0.20f;
// A marker id has to hold for this long before it is registered with the hub.
// registerRobot() is one-way, so a single frame of a misread id would
// otherwise put that id in every MSG_SWARM frame for the rest of the run.
static constexpr float REGISTER_DEBOUNCE_S = 0.30f;

// Alignment — driving to evenly spaced slots ahead of a run, rather than a
// car-following tick. Deliberately gentler than a run's own top speed: this
// is a setup maneuver, not the experiment.
static constexpr float ALIGN_TOLERANCE_DEG = 5.0f;   // "in its slot" for allAligned()
// Debounce on "aligned", the same instinct as REGISTER_DEBOUNCE_S: a vehicle
// only has to cross the tolerance band once, e.g. mid-jitter, not settle in
// it, so the alignment would otherwise finish on a frame it is still moving.
static constexpr float ALIGN_HOLD_S        = 0.5f;
static constexpr float ALIGN_SPEED_MAX_MMS = 120.f;  // world units/s, real time — no time-scale
static constexpr float K_ALIGN             = 2.0f;   // deg of error -> mm/s of tangential command

// Heading controller — carried over from circle_demo.cpp's orbit mode, where
// these were tuned on hardware.
static constexpr float K_ANGLE       = 0.45f;
static constexpr float K_YAW_D       = 0.15f;
static constexpr float K_FF_YAW      = 1.00f;
static constexpr float K_RAD         = 0.30f;   // radial pull back onto the ring
static constexpr float MOTOR_MAX     = 100.0f;
static constexpr float MAX_TURN      = 20.0f;
static constexpr float MAX_TURN_RATE = 120.0f;  // turn-units/s
// Window the yaw rate feeding the D-term is sampled and held over, as in
// circle_demo: one control period, with the MAX_TURN_RATE slew limit doing the
// smoothing rather than a filter on the measurement.
static constexpr float D_TERM_WINDOW_S = CONTROL_INTERVAL_S;

static constexpr float DEG2RAD = (float)M_PI / 180.f;
static constexpr float RAD2DEG = 180.f / (float)M_PI;

static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";
static const char* RING_FILE       = "/tmp/car_following_ring.yml";
static const char* CIRCLE_FILE     = "/tmp/circle_demo.yml";   // circle_demo's, read as a fallback

static volatile std::sig_atomic_t g_running = 1;
static void onSignal(int) { g_running = 0; }

// --debug only. OpenCV runs the callback on its own thread, so a click is just
// recorded here and consumed by the main loop.
static bool      g_leftClick = false;
static cv::Point g_clickPt;
static void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) { g_leftClick = true; g_clickPt = {x, y}; }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

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

// Yaw rate, sampled and held over a fixed window — circle_demo's estimator.
// Differencing the yaw over a whole window rather than per frame is what makes
// this real angular velocity instead of frame-to-frame ArUco jitter.
struct RateEstimator {
    bool  init      = false;
    float baseAngle = 0.f;
    float rate      = 0.f;
    std::chrono::steady_clock::time_point baseTime;
};

static float updateRate(RateEstimator& r, float angle,
                        std::chrono::steady_clock::time_point now, float windowS) {
    if (!r.init) {
        r.baseAngle = angle;
        r.baseTime  = now;
        r.init      = true;
        return 0.f;
    }
    float elapsed = std::chrono::duration<float>(now - r.baseTime).count();
    if (elapsed >= windowS) {
        r.rate      = cfNormAngleDeg(angle - r.baseAngle) / elapsed;
        r.baseAngle = angle;
        r.baseTime  = now;
    }
    return r.rate;
}

// ── The ring geometry ────────────────────────────────────────────────────────
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

// ── Live parameters and run state from the page ──────────────────────────────
// Body is one "name=value" per line, using the page's own widget labels. The
// page posts the full set every time, so this is idempotent — there is no
// partial-update state to keep in sync.
//
// `run` is the "Move" forever-button's state and `setup` a click counter on
// the "Setup" button, so pressing them on the page starts and re-arms the real
// robots the same way it starts and resets the simulation.
struct PageState {
    bool run     = false;
    long setupNo = -1;   // <0 = the page has not reported yet
};

// The buffering knobs. Not part of CfParams: `id` is a robot id, which the
// model library has no notion of, and `b` is resolved against the live robot
// count at each tick rather than stored as a time gap. They come from the
// panel car_following_bridge.js injects, not from a NetLogo widget.
struct Buffering {
    float b  = 1.f;    // buffering parameter B; 1 = the non-cooperative baseline
    int   id = -1;     // robot id of the buffering vehicle; -1 = nobody
    bool on() const { return id >= 0 && b > 1.f; }
};

static void applyParams(const std::string& body, CfParams& p, CfModel& model,
                        PageState& page, Buffering& buf) {
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
        else if (k == "run")           page.run       = atoi(v.c_str()) != 0;
        else if (k == "setup")         page.setupNo   = atol(v.c_str());
        else if (k == "buffer-b")      buf.b          = (float)atof(v.c_str());
        else if (k == "buffer-id")     buf.id         = atoi(v.c_str());
    }
}

// ── Per-robot control state ──────────────────────────────────────────────────
// The model's state lives in CfRing; this is only what the heading controller
// needs between frames.

struct Servo {
    RateEstimator yawRate;
    float         prevTurn  = 0.f;   // slew-limited turn output
    double        firstSeen  = 0.0;   // for the registerRobot debounce
    double        lastSeen   = 0.0;
    bool          everSeen   = false;
    bool          registered = false;
};

// A line typed at the terminal when headless. Non-blocking, so a run that
// nobody is watching is never held up by it.
static bool readStdinLine(std::string& out) {
    struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
    if (::poll(&pfd, 1, 0) <= 0 || !(pfd.revents & POLLIN)) return false;
    char    buf[256];
    ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf) - 1);
    if (n <= 0) return false;
    buf[n] = '\0';
    out.assign(buf);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r' || out.back() == ' '))
        out.pop_back();
    return true;
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);

    std::string serial, ip;
    CfParams    params;
    CfModel     model = CfModel::FVDM;   // the page's default chooser entry
    Buffering   buf;
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
    bool   autoStart   = false;
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
        else if (arg("--buffer-b"))        buf.b       = (float)atof(argv[++i]);
        else if (arg("--buffer-id"))       buf.id      = atoi(argv[++i]);
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
        else if (strcmp(argv[i], "--start")  == 0) autoStart = true;
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("usage: %s [--model NAME] [--speed-max M/S] [--car-size M] [--time-gap S]\n"
                   "       [--reaction-time S] [--sigma A] [--sim-length M] [--radius MM]\n"
                   "       [--centre X Y] [--ring-file PATH] [--fit] [--dir cw|ccw]\n"
                   "       [--time-scale K] [--robot-max-speed MM_S] [--start]\n"
                   "       [--buffer-b B] [--buffer-id ID]\n"
                   "       [--bridge] [--port N] [--debug] [--serial SN] [--ip IP] [--count N]\n\n"
                   "models: Reuschel Pipes OVM CF-OVM FVDM ATG IDM\n\n"
                   "The robots are set up but held still until a run is cued: the page's\n"
                   "\"Move\" button with --bridge, space in --debug, <enter> on stdin when\n"
                   "headless, or --start at launch. \"s\"/\"stop\" returns them to rest;\n"
                   "\"q\" quits. The page's \"Setup\" button (or 'a' in --debug, or\n"
                   "\"a\"/\"align\" on stdin) rests them the same way and then drives them to\n"
                   "evenly spaced slots on the ring, finishing on its own once everyone\n"
                   "visible is in place.\n\n"
                   "--buffer-id ID makes robot ID the buffering vehicle: it keeps B times the\n"
                   "nominal time gap while the other N-1 keep (N-B)/(N-1) of theirs, so the\n"
                   "mean gap — and the density — is unchanged. B = 1 is the non-cooperative\n"
                   "baseline. B is capped where the followers reach half the nominal gap.\n"
                   "Pipes has no desired gap, so buffering does nothing under it.\n\n"
                   "--time-scale K runs the experiment in slow motion: K real seconds per\n"
                   "simulated second, same trajectories, K times slower. Start around 4-6 —\n"
                   "at K=1 the defaults ask for full throttle on a sub-metre ring.\n\n"
                   "--count N pins the vehicle count the virtual ring is sized for, so a\n"
                   "dropped detection cannot rescale the model mid-run.\n\n"
                   "The ring is read from %s (falling back to circle_demo's %s) and\n"
                   "re-saved whenever --radius/--centre/--fit or a debug-view edit changes it.\n",
                   argv[0], RING_FILE, CIRCLE_FILE);
            return 0;
        }
        else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 2; }
    }

    if (buf.b < 1.f) {
        fprintf(stderr, "--buffer-b must be at least 1 (1 = no buffering)\n");
        return 2;
    }
    if (buf.id >= SC_MAX_ROBOTS) {
        fprintf(stderr, "--buffer-id must be below %d\n", SC_MAX_ROBOTS);
        return 2;
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
    PageState  page;
    if (bridge) {
        std::string pageHtml = readFile(exeRelative(
            "../car-following-models/Experiment_by_Sugiyama_et_al.__2007_.html"));
        std::string js = readFile(exeRelative("../vision/car_following_bridge.js"));
        if (pageHtml.empty() || js.empty()) {
            fprintf(stderr, "[bridge] page or script missing — bridge disabled\n");
            bridge = false;
        } else {
            size_t at = pageHtml.rfind("</body>");
            if (at == std::string::npos) at = pageHtml.size();
            pageHtml.insert(at, "<script>\n" + js + "\n</script>\n");
            bridge = http.start(port, std::move(pageHtml));
            printf(bridge ? "[bridge] serving http://127.0.0.1:%d/ — press \"Move\" there to run\n"
                          : "[bridge] could not bind port %d\n", port);
        }
    }

    const char* WIN = "Car Following";
    if (debug) {
        cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
        cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
        cv::setMouseCallback(WIN, onMouse, nullptr);
        printf("[cf] space = run/stop  s = stop  a = align to ring  left-click = ring centre  "
               "+/- = radius %.0f\n"
               "     f = fit ring to robots  , / . = time scale  q/Esc = quit\n",
               RADIUS_STEP_MM);
    }

    CfRingConfig ringCfg;
    ringCfg.dirSign       = dirSign;
    ringCfg.simLengthM    = simLengthM;
    ringCfg.paperSpacingM = PAPER_SPACING_M;
    ringCfg.pinnedCount   = robotCount;
    CfRing cfRing(ringCfg);

    CfRunState run;
    if (autoStart) run.requestStart("--start");

    std::unordered_map<int, Servo> servos;
    std::unordered_map<int, RobotPose> poseById;
    int8_t motors[SC_MAX_ROBOTS][2] = {};
    bool   motorsDirty = true;

    std::mt19937                    rng(12345);
    std::normal_distribution<float> gauss(0.f, 1.f);

    bool  fitPending = fitAtStart;
    cv::Mat blank;   // --debug placeholder while no overlay frame exists yet
    float lastReportedScale = -1.f;

    auto t0  = std::chrono::steady_clock::now();
    auto now = t0;
    auto secondsSince = [&](std::chrono::steady_clock::time_point a) {
        return std::chrono::duration<double>(now - a).count();
    };

    auto lastModel = now, lastControl = now, lastStatus = now,
         lastHubRetry = now, lastMotorTx = now;
    auto fitSince = now;   // when the pending fit started waiting for robots
    auto alignHoldStart = now;   // when allAligned() last became true (ALIGN_HOLD_S debounce)
    DemoHud::LoopFps loopFps;

    // Zeroes the whole command vector. Used on stop, on exit, and whenever the
    // run is not active — motors[] is the single source of truth for what the
    // robots are being told, so nothing else needs to know about the phase.
    auto allStop = [&]() {
        for (int id = 0; id < SC_MAX_ROBOTS; ++id) {
            if (motors[id][0] != 0 || motors[id][1] != 0) motorsDirty = true;
            motors[id][0] = motors[id][1] = 0;
        }
    };

    // Writes the command vector to the hub, but only when it changed or the
    // keepalive is due — see MOTOR_KEEPALIVE_S.
    auto sendMotors = [&](bool force) {
        if (!force && !motorsDirty &&
            std::chrono::duration<float>(now - lastMotorTx).count() < MOTOR_KEEPALIVE_S)
            return;
        for (int id = 0; id < SC_MAX_ROBOTS; ++id)
            swarm.setSpeed((uint8_t)id, motors[id][0], motors[id][1]);
        swarm.flush();
        lastMotorTx = now;
        motorsDirty = false;
    };

    auto restToSetup = [&](const char* why) {
        cfRing.rest();
        for (auto& [id, s] : servos) { s.prevTurn = 0.f; s.yawRate = RateEstimator{}; }
        allStop();
        sendMotors(true);
        printf("[cf] setup — robots at rest (%s)\n", why);
    };

    if (buf.on() && !cfModelHasDesiredGap(model))
        printf("[cf] note: %s has no desired gap — buffering has no effect under it\n",
               cfModelName(model));
    if (buf.on())
        printf("[cf] buffering: robot %d at B=%.2g\n", buf.id, buf.b);
    printf("[cf] model=%s  speed-max=%.1f  car-size=%.1f  time-gap=%.2f  "
           "reaction-time=%.2f  sigma=%.2f  time-scale=%.2gx\n",
           cfModelName(model), params.speedMax, params.carSize,
           params.timeGap, params.reactionTime, params.sigma, timeScale);
    if (timeScale <= 1.f)
        printf("[cf] time-scale is %.2gx — on a ring this small the models ask for "
               "near-full throttle. Try --time-scale 4.\n", timeScale);
    printf("[cf] setup — robots held still. %s\n",
           debug   ? "Press space to run, 'a' to align to the ring."
         : bridge  ? "Press \"Move\" on the page to run, \"Setup\" to align to the ring."
                   : "Press <enter> to run, \"a\"/\"align\" to align, \"s\" to stop, \"q\" to quit.");

    while (g_running) {
        // A new frame is the trigger for control, but never a precondition for
        // housekeeping: the cues, the hub retry and the stop path have to keep
        // working even if the camera stalls, which is exactly when someone
        // reaches for the stop.
        bool haveFrame = tracker.update();
        now = std::chrono::steady_clock::now();
        if (haveFrame) loopFps.tick();

        if (!swarm.isConnected() && secondsSince(lastHubRetry) >= 2.0) {
            lastHubRetry = now;
            if (swarm.connect()) printf("[hub] connected\n");
        }
        swarm.poll();

        // ── Cues ─────────────────────────────────────────────────────────────
        if (bridge) {
            for (const auto& body : http.poll()) {
                bool wasRun  = page.run;
                long wasSetup = page.setupNo;
                applyParams(body, params, model, page, buf);
                if (page.setupNo != wasSetup && wasSetup >= 0)
                    run.requestAlign("page setup");
                else if (page.run != wasRun)
                    page.run ? run.requestStart("page") : run.requestStop("page");
            }
        }

        if (!debug) {
            std::string line;
            if (readStdinLine(line)) {
                for (auto& ch : line) ch = (char)tolower((unsigned char)ch);
                if      (line == "q" || line == "quit") g_running = 0;
                else if (line == "s" || line == "stop") run.requestStop("stdin");
                else if (line.empty() || line == "g" || line == "go" || line == "start")
                    run.requestStart("stdin");
                else if (line == "a" || line == "align")
                    run.requestAlign("stdin");
                else printf("[cf] <enter>/go = run, a/align = align, s = stop, q = quit\n");
            }
        }

        // ── Poses ────────────────────────────────────────────────────────────
        const double tNow = secondsSince(t0);
        if (haveFrame) {
            poseById.clear();
            for (auto& r : tracker.robots())
                if (r.id >= 0 && r.id < SC_MAX_ROBOTS) poseById[r.id] = r;

            cfRing.beginFrame();
            for (auto& [id, p] : poseById) {
                float a = atan2f(p.y - ring.centre.y, p.x - ring.centre.x) * RAD2DEG;
                cfRing.observe(id, a, tNow);

                Servo& s = servos[id];
                if (!s.everSeen) { s.firstSeen = tNow; s.everSeen = true; }
                s.lastSeen = tNow;
            }
            cfRing.endFrame(tNow);

            for (auto it = servos.begin(); it != servos.end(); )
                it = cfRing.has(it->first) ? std::next(it) : servos.erase(it);
        }

        // A marker that has held for the debounce joins the swarm frame. This
        // is one-way in SwarmClient, hence the wait: a single misread id would
        // otherwise ride along in every frame for the rest of the run.
        for (auto& [id, s] : servos) {
            if (s.everSeen && !s.registered &&
                tNow - s.firstSeen >= REGISTER_DEBOUNCE_S) {
                swarm.registerRobot((uint8_t)id);
                s.registered = true;
            }
        }

        if (cfRing.takeRosterChange())
            printf("[cf] roster settled at %d robot%s\n",
                   cfRing.rosterCount(), cfRing.rosterCount() == 1 ? "" : "s");

        // ── Ring edits ───────────────────────────────────────────────────────
        // A pending --fit / 'f' waits for enough robots to be detected; a
        // click is already in world units once it goes through the
        // homography. Both persist immediately, so the ring a run ends with is
        // the ring the next run starts on.
        if (fitPending) {
            if ((int)poseById.size() >= FIT_MIN_ROBOTS) {
                fitPending = false;
                if (fitRing(poseById, ring)) saveRing(ring, ringFile);
            } else if (secondsSince(fitSince) > FIT_WAIT_S) {
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

        // The scale factor divides by the radius, so it is read after any ring
        // edit this frame rather than before it.
        const float simPerMm = cfRing.simPerMm(ring.radius);
        if (simPerMm > 0.f && simPerMm != lastReportedScale) {
            lastReportedScale = simPerMm;
            printf("[cf] virtual ring %.1f m over %d vehicles (%.2f m each)\n",
                   cfRing.simLengthM(), cfRing.rosterCount(),
                   cfRing.simLengthM() / std::max(1, cfRing.rosterCount()));
        }

        // ── Run state ────────────────────────────────────────────────────────
        // Everything either cue needs before a wheel turns: a link to the
        // robots, a ring to drive round, a settled roster to scale the model
        // with, and at least one robot actually on it.
        const bool ready = swarm.isConnected() && ring.radius > 0.f &&
                           cfRing.scaleReady(ring.radius) && cfRing.visibleCount() > 0;

        // Debounced the same way REGISTER_DEBOUNCE_S is: allAligned() crossing
        // true for one frame (e.g. mid-jitter) should not end the maneuver
        // while a robot is still visibly moving.
        if (!cfRing.allAligned(ALIGN_TOLERANCE_DEG)) alignHoldStart = now;
        const bool alignDone = cfRing.allAligned(ALIGN_TOLERANCE_DEG) &&
                               secondsSince(alignHoldStart) >= ALIGN_HOLD_S;

        switch (run.update(ready, alignDone)) {
            case CfRunEvent::Started:
                // The model's clock restarts with the run: without this the
                // first tick would integrate the whole setup period.
                lastModel = now;
                cfRing.rest();
                printf("[cf] running (%s) — %d robots, %s, %.2gx\n",
                       run.source(), cfRing.visibleCount(), cfModelName(model), timeScale);
                break;
            case CfRunEvent::Stopped:
                restToSetup(run.source());
                break;
            case CfRunEvent::Waiting:
                printf("[cf] start cued (%s) — waiting for%s%s%s%s\n", run.source(),
                       swarm.isConnected()            ? "" : " the hub",
                       ring.radius > 0.f              ? "" : " a ring radius",
                       cfRing.scaleReady(ring.radius) ? "" : " the roster to settle",
                       cfRing.visibleCount() > 0      ? "" : " a robot in view");
                break;
            case CfRunEvent::AlignStarted:
                alignHoldStart = now;
                printf("[cf] aligning (%s) — %d robots to evenly spaced slots\n",
                       run.source(), cfRing.visibleCount());
                break;
            case CfRunEvent::AlignWaiting:
                printf("[cf] align cued (%s) — waiting for%s%s%s%s\n", run.source(),
                       swarm.isConnected()            ? "" : " the hub",
                       ring.radius > 0.f              ? "" : " a ring radius",
                       cfRing.scaleReady(ring.radius) ? "" : " the roster to settle",
                       cfRing.visibleCount() > 0      ? "" : " a robot in view");
                break;
            case CfRunEvent::Aligned:
                allStop();
                sendMotors(true);
                printf("[cf] aligned — ready to run\n");
                break;
            case CfRunEvent::None:
                break;
        }

        // ── Model tick ───────────────────────────────────────────────────────
        if (run.running() && secondsSince(lastModel) >= MODEL_DT_S) {
            float modelDt = clampf((float)secondsSince(lastModel), 0.02f, 0.5f);
            lastModel = now;

            // The tick still fires on a real-time interval; only the elapsed
            // time the model is told about is dilated. cfStep is a stateless
            // explicit-Euler step, so a dt smaller than the paper's 0.1s only
            // makes the integration finer.
            cfRing.step(model, params, modelDt / timeScale, ring.radius,
                        [&] { return gauss(rng); }, buf.id, buf.b);
        }

        // ── Servo each robot onto its commanded speed ────────────────────────
        if (secondsSince(lastControl) >= CONTROL_INTERVAL_S) {
            float controlDt = clampf((float)secondsSince(lastControl), 0.001f, 0.2f);
            lastControl = now;

            if (!run.running() && !run.aligning()) {
                allStop();
            } else {
                // Robots that have gone unseen for longer than the hold window
                // stop; everyone visible is recomputed below. They keep their
                // place on the ring for longer than that (CfRingConfig::holdS)
                // so their follower still brakes for them.
                for (int id = 0; id < SC_MAX_ROBOTS; ++id) {
                    auto it = servos.find(id);
                    if (it == servos.end() || tNow - it->second.lastSeen > MOTOR_HOLD_S) {
                        if (motors[id][0] || motors[id][1]) motorsDirty = true;
                        motors[id][0] = motors[id][1] = 0;
                    }
                }

                // circle_demo's heading gains (K_FF_YAW, K_RAD) are tuned
                // against motor units, since it never converts — it feeds its
                // orbit rate straight to the motors as `forward`. So the
                // model's world-units/s field is converted down to motor
                // units before the heading law below ever sees it; leaving it
                // in mm/s scales the feedforward up and the radial pull down
                // by robotMaxMms/MOTOR_MAX, and since the feedforward is
                // proportional to speed the robots then orbit at a
                // speed-dependent radius (see the file header).
                float mmPerUnit = robotMaxMms / MOTOR_MAX;

                for (auto& [id, pose] : poseById) {
                    const CfRingCar* c = cfRing.car(id);
                    Servo&           s = servos[id];
                    int8_t l = 0, r = 0;

                    float dx = pose.x - ring.centre.x, dy = pose.y - ring.centre.y;
                    float distC = std::hypot(dx, dy);

                    // The rate estimator is fed every control step whatever the
                    // outcome below, so a robot that spends a moment stopped
                    // does not come back with a stale yaw rate.
                    float yawRate = updateRate(s.yawRate, pose.yaw, now, D_TERM_WINDOW_S);

                    // Aligning: raw angular error onto the assigned slot, real
                    // time throughout, no density/time-scale conversion — this
                    // is not a car-following tick. Running: the model's speed,
                    // converted from simulated to world units. Both feed the
                    // same tangential/radial mix and heading controller below;
                    // the sign flip matches vTan to the tx/ty basis, which is
                    // itself signed by dirSign. Both are then converted from
                    // world units (mm/s) to motor units, per the comment above.
                    bool  haveCmd = false;
                    float vTan    = 0.f;   // motor units
                    if (run.aligning()) {
                        if (c) {
                            float vTanMms = dirSign * clampf(K_ALIGN * c->alignErrorDeg,
                                                             -ALIGN_SPEED_MAX_MMS, ALIGN_SPEED_MAX_MMS);
                            vTan    = clampf(vTanMms / mmPerUnit, -MOTOR_MAX, MOTOR_MAX);
                            haveCmd = true;
                        }
                    } else if (c && simPerMm > 0.f) {
                        // sim m/s -> world units per *real* second: undo the
                        // density scale, then the time dilation.
                        float vTanMms = c->speed / simPerMm / timeScale;    // world units/s
                        vTan    = clampf(vTanMms / mmPerUnit, -MOTOR_MAX, MOTOR_MAX);
                        haveCmd = true;
                    }

                    if (haveCmd && distC >= 1.f) {
                        float rx = dx / distC,          ry = dy / distC;
                        float tx = dirSign * -ry,       ty = dirSign * rx;

                        float vRad = clampf(-K_RAD * (distC - ring.radius),
                                            -MOTOR_MAX * 0.5f, MOTOR_MAX * 0.5f);

                        float vx = vTan * tx + vRad * rx;
                        float vy = vTan * ty + vRad * ry;
                        float vMag = std::hypot(vx, vy);

                        if (vMag >= 0.5f) {
                            // Raw pose yaw, not a filtered one: on a circle the
                            // true heading rotates at v/R, so any lag in the
                            // measurement becomes a standing heading error the
                            // P-term steers out (see the file header).
                            float angleErr  = cfNormAngleDeg(atan2f(vy, vx) * RAD2DEG - pose.yaw);
                            float headingN  = clampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
                            float headingSc = 1.f - headingN * headingN;

                            // Feedforward carries the steady turn (v/R);
                            // feedback only corrects the residual. Gated by
                            // headingSc so a robot that is still spinning to
                            // align can't deadlock at zero output — see
                            // circle_demo.cpp for the full derivation.
                            float ffOmega = dirSign * (vTan / ring.radius) * RAD2DEG;
                            float dErr    = clampf(ffOmega - yawRate, -300.f, 300.f);

                            float forward = clampf(vMag, 0.f, MOTOR_MAX) * headingSc;
                            float turnTgt = clampf(K_FF_YAW * ffOmega * headingSc
                                                   + K_ANGLE * angleErr + K_YAW_D * dErr,
                                                   -MAX_TURN, MAX_TURN);
                            float maxStep = MAX_TURN_RATE * controlDt;
                            float turn    = clampf(turnTgt, s.prevTurn - maxStep,
                                                   s.prevTurn + maxStep);
                            s.prevTurn    = turn;

                            l = (int8_t)clampf(forward + turn, -MOTOR_MAX, MOTOR_MAX);
                            r = (int8_t)clampf(forward - turn, -MOTOR_MAX, MOTOR_MAX);
                        } else {
                            s.prevTurn = 0.f;
                        }
                    }

                    if (motors[id][0] != l || motors[id][1] != r) motorsDirty = true;
                    motors[id][0] = l;
                    motors[id][1] = r;
                }
            }
            sendMotors(false);
        }

        // ── Report ───────────────────────────────────────────────────────────
        if (!debug) {
            if (secondsSince(lastStatus) >= 1.0) {
                lastStatus = now;
                printf("[cf] %-7s %-8s loop:%3.0f  robots:%d/%d  %.2gx  hub:%s",
                       run.running()  ? "RUN"   :
                       run.aligning() ? "ALIGN" : (run.pending() ? "CUED" : "SETUP"),
                       cfModelName(model), loopFps.fps(),
                       cfRing.visibleCount(), cfRing.rosterCount(), timeScale,
                       swarm.isConnected() ? "ok" : "--");
                if (buf.on())
                    printf("  buf:%d@B=%.2g%s", buf.id, cfRing.bufferB(),
                           cfRing.bufferActive() ? "" : "(absent)");
                // A '*' marks the buffering vehicle, the way the reference
                // page colours it cyan.
                for (int id : cfRing.order()) {
                    const CfRingCar* c = cfRing.car(id);
                    printf("  %d%s:%.2f/%.2fm/s@%.1fm", id,
                           (cfRing.bufferActive() && id == buf.id) ? "*" : "",
                           c->speed, c->measured, c->gap);
                }
                printf("\n");
                fflush(stdout);
            }
            if (!haveFrame) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // ── Debug view ───────────────────────────────────────────────────────
        cv::Mat disp = tracker.debugFrame().clone();
        if (disp.empty()) {
            // No overlay frame yet — but the window still has to take keys, or
            // there is no way to stop the robots from it. Reused rather than
            // reallocated: at 2048 square that is 12 MB a frame.
            if (blank.empty()) blank.create(tracker.frameSize(), CV_8UC3);
            blank.setTo(cv::Scalar::all(0));
            disp = blank;
        }

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
                         "space = run/stop   s = stop   a = align   click = centre   +/- = radius   "
                         "f = fit   , / . = time scale   q = quit",
                         {12, disp.rows - 16}, 18, {0, 200, 255});

        for (int id : cfRing.order()) {
            const CfRingCar* c = cfRing.car(id);
            auto it = poseById.find(id);
            if (it == poseById.end()) continue;
            bool isBuf = cfRing.bufferActive() && id == buf.id;
            // Cyan for the buffering vehicle, as the reference page draws it.
            tracker.drawText(disp, DemoHud::fmt("%.2fm/s  gap %.1fm%s",
                                                c->speed, c->gap, isBuf ? "  BUF" : ""),
                             {(int)it->second.px + 12, (int)it->second.py - 12},
                             18, isBuf ? cv::Scalar{255, 255, 0} : cv::Scalar{0, 255, 128});
        }

        DemoHud hud;
        hud.title(DemoHud::fmt("loop_fps:%.0f  %s  %s  robots:%d/%d  ring:%.0f  %.2gx%s  HUB:%s%s",
                               loopFps.fps(),
                               run.running()  ? "RUNNING" :
                               run.aligning() ? "ALIGNING" : (run.pending() ? "CUED" : "SETUP"),
                               cfModelName(model),
                               cfRing.visibleCount(), cfRing.rosterCount(),
                               ring.radius, timeScale,
                               buf.on() ? DemoHud::fmt("  buf:%d@B=%.2g",
                                                       buf.id, cfRing.bufferB()).c_str() : "",
                               swarm.isConnected() ? "OK" : "INACTIVE",
                               bridge ? "  BRIDGE" : ""),
                  run.running() && swarm.isConnected()  ? DemoHud::COL_OK :
                  run.aligning() && swarm.isConnected() ? DemoHud::COL_WARN : DemoHud::COL_BAD);
        // Gap/speed/measured are only meaningful mid-run — the model does not
        // step while aligning, so they would otherwise show whatever the last
        // run left behind. Slot error is the aligning equivalent.
        hud.header(run.aligning()
                   ? std::vector<std::string>{"ID", "Slot deg", "Err deg", "-", "Mot-L", "Mot-R", "Battery"}
                   : std::vector<std::string>{"ID", "Gap m", "v m/s", "meas", "Mot-L", "Mot-R", "Battery"});
        for (int id : cfRing.order()) {
            const CfRingCar* c  = cfRing.car(id);
            const auto&      ss = swarm.robotState((uint8_t)id);
            hud.row({DemoHud::fmt("%d%s", id,
                                  (cfRing.bufferActive() && id == buf.id) ? "*" : ""),
                     run.aligning() ? DemoHud::fmt("%.0f", c->alignTargetDeg) : DemoHud::fmt("%.1f", c->gap),
                     run.aligning() ? DemoHud::fmt("%+.0f", c->alignErrorDeg) : DemoHud::fmt("%.2f", c->speed),
                     run.aligning() ? "-" : DemoHud::fmt("%.2f", c->measured),
                     DemoHud::fmt("%+d", (int)motors[id][0]),
                     DemoHud::fmt("%+d", (int)motors[id][1]),
                     ss.known ? DemoHud::formatBattery(ss.battery) : "--"},
                    DemoHud::COL_OK);
        }
        hud.drawTopRight(disp);
        cv::imshow(WIN, disp);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == ' ') run.toggle("key");
        if (key == 's') run.requestStop("key");
        if (key == 'a') run.requestAlign("key");
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
    }

    // Stopping is the one thing that must not be best-effort: a single frame
    // can be lost to a reconnect, so the zero command is repeated. The robots'
    // own watchdog is the backstop if none of them lands.
    allStop();
    for (int i = 0; i < 3; ++i) {
        now = std::chrono::steady_clock::now();
        sendMotors(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (debug) cv::destroyAllWindows();
    printf("[cf] stopped\n");
    return 0;
}
