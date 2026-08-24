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
//                   [--robot-max-speed MM_S] [--bridge] [--port N]
//                   [--debug] [--serial SN] [--ip IP] [--count N]
//
// Debug keys: s = stop, r = re-fit the ring, q/Esc = quit
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
// The heading controller underneath is circle_demo.cpp's orbit controller:
// a yaw feedforward carries the steady turn and a PD corrects the residual
// (see the long comment there for why pure feedback oscillates on a circle).
// The one difference is where the tangential speed comes from — per robot,
// from the car-following model, instead of one global orbit rate.

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
static constexpr float SEND_INTERVAL_S  = 0.01f;
static constexpr float DEFAULT_RADIUS_MM = 300.0f; // fallback when the ring can't be fitted
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

static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

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

// World -> pixel, for drawing only. ArucoTracker keeps its homography private
// and only ever maps the other way, so the debug view reads the same file back
// and caches the inverse once (circle_demo recomputes it per call).
static cv::Mat g_Hinv;

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
    float vCmd     = 0.f;  // simulated m/s, the model's output
    float gap      = 0.f;  // simulated m, clear distance to the predecessor
    float prevTurn = 0.f;  // slew-limited turn output
    bool  haveYaw  = false;
    std::chrono::steady_clock::time_point lastSeen;
};

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);

    std::string serial, ip;
    CfParams    params;
    CfModel     model = CfModel::FVDM;   // the page's default chooser entry
    float  simLengthM  = -1.f;           // <0 = derive from the robot count
    float  radiusMm    = -1.f;
    float  centreX = 0.f, centreY = 0.f;
    bool   haveCentre  = false;
    float  dirSign     = 1.f;            // +1 = counter-clockwise
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
        else if (arg("--radius"))          radiusMm    = (float)atof(argv[++i]);
        else if (arg("--robot-max-speed")) robotMaxMms = (float)atof(argv[++i]);
        else if (arg("--port"))            port        = atoi(argv[++i]);
        else if (arg("--dir"))             dirSign     = strcmp(argv[++i], "cw") == 0 ? -1.f : 1.f;
        else if (strcmp(argv[i], "--centre") == 0 && i + 2 < argc) {
            centreX = (float)atof(argv[++i]);
            centreY = (float)atof(argv[++i]);
            haveCentre = true;
        }
        else if (strcmp(argv[i], "--debug")  == 0) debug  = true;
        else if (strcmp(argv[i], "--bridge") == 0) bridge = true;
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("usage: %s [--model NAME] [--speed-max M/S] [--car-size M] [--time-gap S]\n"
                   "       [--reaction-time S] [--sigma A] [--sim-length M] [--radius MM]\n"
                   "       [--centre X Y] [--dir cw|ccw] [--robot-max-speed MM_S]\n"
                   "       [--bridge] [--port N] [--debug] [--serial SN] [--ip IP] [--count N]\n\n"
                   "models: Reuschel Pipes OVM CF-OVM FVDM ATG IDM\n", argv[0]);
            return 0;
        }
        else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 2; }
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
        if (!H.empty()) g_Hinv = H.inv();
        printf("[vision] homography loaded from %s\n", HOMOGRAPHY_FILE);
    } else {
        printf("[vision] no homography at %s — working in pixels "
               "(run `circle_demo --calibrate` to create one)\n", HOMOGRAPHY_FILE);
    }

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

    bool     ringSet   = false;
    float    ringR     = radiusMm > 0.f ? radiusMm : DEFAULT_RADIUS_MM;
    float    simPerMm  = 0.f;   // simulated metres per world unit
    int      lastCount = -1;
    uint64_t tickNo    = 1;     // model ticks; 0 means "never measured"
    bool     paused    = false; // 's' in --debug; holds every motor at zero

    auto now       = std::chrono::steady_clock::now();
    auto lastModel = now, lastSend = now, lastStatus = now, lastHubRetry = now, lastFrame = now;
    DemoHud::LoopFps loopFps;

    printf("[cf] model=%s  speed-max=%.1f  car-size=%.1f  time-gap=%.2f  "
           "reaction-time=%.2f  sigma=%.2f\n",
           cfModelName(model), params.speedMax, params.carSize,
           params.timeGap, params.reactionTime, params.sigma);

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
            if (!c.haveYaw) { c.yaw = pose.yaw; c.prevYaw = pose.yaw; c.haveYaw = true; }
            else            c.yaw = normAngle(c.yaw + yawAlpha * normAngle(pose.yaw - c.yaw));
        }

        // Brief detection dropouts are normal, so state is held for a second
        // rather than dropped on the first missed frame — but not forever: a
        // stale entry would feed a phantom neighbour into someone's gap.
        for (auto it = cars.begin(); it != cars.end(); )
            it = std::chrono::duration<float>(now - it->second.lastSeen).count() > 1.0f
                     ? cars.erase(it) : std::next(it);

        // ── Fit the ring to wherever the robots are standing ─────────────────
        if (!ringSet && !poseById.empty()) {
            float sx = 0, sy = 0;
            for (auto& [id, p] : poseById) { sx += p.x; sy += p.y; }
            centreX = haveCentre ? centreX : sx / poseById.size();
            centreY = haveCentre ? centreY : sy / poseById.size();

            if (radiusMm > 0.f) {
                ringR = radiusMm;
            } else {
                float sr = 0;
                for (auto& [id, p] : poseById)
                    sr += std::hypot(p.x - centreX, p.y - centreY);
                ringR = sr / poseById.size();
                if (ringR < 50.f) ringR = DEFAULT_RADIUS_MM;   // one robot, or all on top of each other
            }
            ringSet = true;
            printf("[cf] ring: centre=(%.0f, %.0f)  radius=%.0f\n", centreX, centreY, ringR);
        }

        // ── Ring order ───────────────────────────────────────────────────────
        // Sorted counter-clockwise; the predecessor is the next robot in the
        // direction of travel.
        std::vector<std::pair<float,int>> byAngle;   // {angle deg, id}
        if (ringSet) {
            for (auto& [id, p] : poseById) {
                float a = atan2f(p.y - centreY, p.x - centreX) * RAD2DEG;
                byAngle.push_back({a < 0 ? a + 360.f : a, id});
            }
            std::sort(byAngle.begin(), byAngle.end());
        }
        const int M = (int)byAngle.size();

        // Density: the virtual ring grows with the number of robots so the
        // metres-per-vehicle stay at the experiment's value (see the header).
        if (M > 0) {
            float lengthM = simLengthM > 0.f ? simLengthM : M * PAPER_SPACING_M;
            simPerMm = lengthM / (2.f * (float)M_PI * ringR);
            if (M != lastCount) {
                printf("[cf] %d robots → virtual ring %.1f m (%.2f m per vehicle)\n",
                       M, lengthM, lengthM / M);
                lastCount = M;
            }
        }

        // ── Model tick ───────────────────────────────────────────────────────
        float modelDt = std::chrono::duration<float>(now - lastModel).count();
        if (M > 0 && modelDt >= MODEL_DT_S) {
            lastModel = now;
            modelDt   = clampf(modelDt, 0.02f, 0.5f);
            ++tickNo;

            // Pass 1: measure. Every robot's speed and gap must be read before
            // any model runs, or a robot would see its predecessor's
            // already-updated speed instead of this tick's.
            for (int i = 0; i < M; ++i) {
                auto [ang, id] = byAngle[i];
                Car& c = cars[id];

                if (c.tickSeen + 1 == tickNo) {
                    c.speed   = dirSign * normAngle(ang - c.prevAng) * DEG2RAD * ringR
                                / modelDt * simPerMm;
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
                c.gap = gapDeg * DEG2RAD * ringR * simPerMm - params.carSize;
            }

            // Pass 2: step every model off that snapshot.
            for (int i = 0; i < M; ++i) {
                int  id   = byAngle[i].second;
                int  pred = byAngle[(i + (dirSign > 0 ? 1 : M - 1)) % M].second;
                Car& c    = cars[id];

                CfInput in{c.gap, c.speed, cars[pred].speed, cars[pred].gap};
                float v = cfStep(model, in, params, modelDt,
                                 params.sigma > 0.f ? gauss(rng) : 0.f);
                // The robots only ever go forwards around the ring: a model
                // that undershoots into reverse would have them driving into
                // their follower.
                c.vCmd = clampf(v, 0.f, params.speedMax);
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
                if (!ringSet || simPerMm <= 0.f) { motors[id][0] = motors[id][1] = 0; continue; }

                float dx = pose.x - centreX, dy = pose.y - centreY;
                float distC = std::hypot(dx, dy);
                if (distC < 1.f) { motors[id][0] = motors[id][1] = 0; continue; }

                float rx = dx / distC,          ry = dy / distC;
                float tx = dirSign * -ry,       ty = dirSign * rx;

                float vTan = c.vCmd / simPerMm;                       // world units/s
                float vRad = clampf(-K_RAD * (distC - ringR),
                                    -robotMaxMms * 0.5f, robotMaxMms * 0.5f);

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
                // circle_demo.cpp for the full derivation.
                float ffOmega = dirSign * (vTan / ringR) * RAD2DEG;
                float dErr    = clampf(ffOmega - c.yawRate, -300.f, 300.f);

                float forward = clampf(vMag / robotMaxMms * MOTOR_MAX, 0.f, MOTOR_MAX) * headingSc;
                float turnTgt = clampf(K_FF_YAW * ffOmega * headingSc
                                       + K_ANGLE * angleErr + K_YAW_D * dErr,
                                       -MAX_TURN, MAX_TURN);
                float maxStep = MAX_TURN_RATE * sendDt;
                float turn    = clampf(turnTgt, c.prevTurn - maxStep, c.prevTurn + maxStep);
                c.prevTurn    = turn;

                swarm.registerRobot((uint8_t)id);
                motors[id][0] = (int8_t)clampf(forward + turn, -MOTOR_MAX, MOTOR_MAX);
                motors[id][1] = (int8_t)clampf(forward - turn, -MOTOR_MAX, MOTOR_MAX);
            }
            sendMotors();
        }

        // ── Report ───────────────────────────────────────────────────────────
        if (!debug) {
            if (std::chrono::duration<float>(now - lastStatus).count() >= 1.0f) {
                lastStatus = now;
                printf("[cf] %-8s loop:%3.0f  robots:%d  hub:%s",
                       cfModelName(model), loopFps.fps(), M,
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
        if (ringSet) {
            // The ring is defined in world units; project it back through the
            // homography by transforming points on it rather than assuming
            // pixels and world units share a scale.
            std::vector<cv::Point> poly;
            for (int a = 0; a < 360; a += 4)
                poly.push_back(worldToPixel({centreX + ringR * cosf(a * DEG2RAD),
                                             centreY + ringR * sinf(a * DEG2RAD)}));
            cv::polylines(disp, poly, true, {0, 200, 255}, 2, cv::LINE_AA);

            for (auto& [ang, id] : byAngle) {
                const Car& c = cars[id];
                tracker.drawText(disp, DemoHud::fmt("%.1fm/s  gap %.1fm", c.vCmd, c.gap),
                                 {(int)poseById[id].px + 12, (int)poseById[id].py - 12},
                                 18, {0, 255, 128});
            }
        }

        DemoHud hud;
        hud.title(DemoHud::fmt("loop_fps:%.0f  %s  robots:%d  ring:%.0f  HUB:%s%s",
                               loopFps.fps(), cfModelName(model), M, ringR,
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
        if (key == 'r') { ringSet = false; printf("[cf] re-fitting ring\n"); }
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
