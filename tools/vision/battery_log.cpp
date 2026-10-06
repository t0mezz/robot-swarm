// battery_log.cpp — drive one robot round the saved ring at a fixed motor
// command until its battery is flat, logging the camera-measured speed next
// to the battery voltage the robot reports, and plotting both as it goes.
//
// The research question behind it: can a robot's remaining battery be
// estimated from the overhead camera alone? The first step is whether its
// real speed at a fixed command tracks its voltage. That only holds because
// the robots run open-loop (ODOMETRY_ENABLED = False in
// src/robots/uart_controller.py): a command is a fixed PWM duty, so the
// wheels slow down as the battery sags. With the encoder PID on, the robot
// would hold its speed until the motors saturate and this log would be flat
// until the very end. Check that flag before trusting a run.
//
// Headless by default: one status line per second, a CSV row per second and
// a PNG plot rewritten every minute. --debug adds the camera view and a live
// plot window.
//
// Usage:
//   ./battery_log --robot ID [--cmd N] [--dir cw|ccw]
//                 [--rest-every S] [--rest-for S] [--stop-mv MV] [--max-time S]
//                 [--start] [--out DIR] [--debug] [--no-open] [--serial SN] [--ip IP]
//
// ── Lifecycle ────────────────────────────────────────────────────────────────
//   setup  motors held at zero until the run is cued (<enter> on stdin,
//          space in --debug, or --start) and the hub, ring, homography and
//          robot are all there.
//   seek   radial-only drive onto the ring (circle_speed_test's seek).
//   orbit  the fixed command, round the ring.
//   rest   motors at zero every --rest-every s for --rest-for s, so the log
//          also records the resting voltage; ends in a reseek.
//   done   low voltage, a stall, a lost robot, --max-time, or s/q. The final
//          PNG is then opened in the desktop's image viewer (--no-open to
//          skip, e.g. over ssh).
//
// Every log row belongs to exactly one phase: a phase change closes the
// current row early, which is why the CSV carries its own dt_s column.
// Measurement lives in lib/BatteryLog/battery_log.h (unit-tested); this file
// is vision, control, I/O and drawing.
//
// ── The controller ───────────────────────────────────────────────────────────
// circle_demo.cpp's orbit controller, ported from car_following.cpp, which
// carries it with circle_demo's yaw low-pass (YAW_TAU_S) — circle_speed_test
// does not, and a run that lasts hours needs the filtered version. Treat it as
// a port, not a variant: see CLAUDE.md for the two departures that were tried
// on hardware and reverted. Its field is in motor units, so --cmd is fed
// straight to it as the tangential speed, exactly as circle_demo feeds vTan.
// Fixes to that control law should land in circle_demo, car_following,
// circle_speed_test and here.

#include "aruco_tracker.h"
#include "SwarmClient.h"
#include "DemoHud.h"
#include "battery_log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <fcntl.h>
#include <unistd.h>

// ── Tunables ─────────────────────────────────────────────────────────────────

// Heading controller — circle_demo.cpp's orbit mode, as car_following.cpp
// carries it. See the comments there before touching any of these.
static constexpr float K_ANGLE         = 0.45f;
static constexpr float K_YAW_D         = 0.15f;
static constexpr float K_FF_YAW        = 1.00f;
static constexpr float K_RAD           = 0.30f;
static constexpr float MOTOR_MAX       = 100.0f;
static constexpr float MAX_TURN        = 20.0f;    // feedforward + feedback together
static constexpr float MAX_TURN_RATE   = 120.0f;   // turn-units/s
static constexpr float YAW_TAU_S       = 0.50f;
static constexpr float D_TERM_WINDOW_S = 0.01f;

static constexpr float SEEK_ARRIVAL_MM   = 20.f;   // circle_speed_test's
static constexpr float CONTROL_STALL_S   = 0.05f;  // control floor if the camera stalls
static constexpr float MOTOR_KEEPALIVE_S = 0.10f;  // well inside WATCHDOG_TIMEOUT_MS
static constexpr float MOTOR_HOLD_S      = 0.20f;  // unseen this long -> motors to zero
static constexpr float LOST_RESEEK_S     = 1.00f;  // unseen this long in orbit -> reseek on return
static constexpr float REGISTER_DEBOUNCE_S = 0.30f;
// Telemetry older than this is not the battery's current voltage. The robot
// samples every 2 s and telemetry repeats it, so a few seconds is generous.
static constexpr float TELEMETRY_STALE_S = 3.0f;

static constexpr double ROW_S          = 1.0;     // one CSV row per second
static constexpr double PLOT_SAVE_S    = 60.0;    // PNG rewrite interval
static constexpr double PLOT_REDRAW_S  = 1.0;     // --debug live plot
static constexpr double MEDIAN_WINDOW_S = 30.0;   // smoothed speed trace

static constexpr float RAD2DEG = 180.f / (float)M_PI;

static const std::string HOMOGRAPHY_FILE = arucoVisionDataPath("aruco_homography.yml");
static const std::string RING_FILE       = arucoVisionDataPath("car_following_ring.yml");
static const std::string CIRCLE_FILE     = arucoVisionDataPath("circle_demo.yml");

static volatile std::sig_atomic_t g_running = 1;
static void onSignal(int) { g_running = 0; }

// ── Helpers ──────────────────────────────────────────────────────────────────

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static float normAngle(float a) { while (a > 180.f) a -= 360.f; while (a < -180.f) a += 360.f; return a; }

static cv::Mat g_Hinv;
static cv::Point worldToPixel(cv::Point2f w) {
    if (g_Hinv.empty()) return {(int)w.x, (int)w.y};
    std::vector<cv::Point2f> src = {w}, dst;
    cv::perspectiveTransform(src, dst, g_Hinv);
    cv::Point2f q = arucoDistortPixel(dst[0]);   // back into the distorted image we draw on
    return {(int)q.x, (int)q.y};
}

// car_following's ring fixture, read-only here: this tool never edits the
// ring, it runs on whatever car_following or circle_demo last saved.
struct Ring { cv::Point2f centre{0.f, 0.f}; float radius = 0.f; bool centreSet = false; };

static bool loadRing(Ring& ring, const std::string& path) {
    if (!std::ifstream(path).good()) return false;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    float cx = 0.f, cy = 0.f, r = 0.f;
    int   centreSet = 0;
    fs["cx"] >> cx; fs["cy"] >> cy; fs["centre_set"] >> centreSet; fs["radius"] >> r;
    if (centreSet) { ring.centre = {cx, cy}; ring.centreSet = true; }
    if (r > 0.f)     ring.radius = r;
    return true;
}

// circle_demo's yaw-rate estimator: differenced over a window, sample and hold.
struct RateEstimator {
    bool  init = false;
    float baseAngle = 0.f, rate = 0.f;
    std::chrono::steady_clock::time_point baseTime;
};
static float updateRate(RateEstimator& r, float angle,
                        std::chrono::steady_clock::time_point now, float windowS) {
    if (!r.init) { r.baseAngle = angle; r.baseTime = now; r.init = true; return 0.f; }
    float elapsed = std::chrono::duration<float>(now - r.baseTime).count();
    if (elapsed >= windowS) {
        r.rate = normAngle(angle - r.baseAngle) / elapsed;
        r.baseAngle = angle; r.baseTime = now;
    }
    return r.rate;
}

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

static std::string wallTime(const char* fmt) {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[64];
    std::strftime(buf, sizeof(buf), fmt, &tm);
    return buf;
}

static std::string hms(double s) {
    int t = (int)s;
    return DemoHud::fmt("%d:%02d:%02d", t / 3600, (t / 60) % 60, t % 60);
}

// Opens a file in the desktop's default viewer and returns at once. fork+exec
// rather than system() so a path with quotes or spaces in --out needs no shell
// escaping; the child gets its own session so closing this terminal does not
// take the viewer with it. Skipped without a display, where it could only fail.
static void openInViewer(const std::string& path) {
#ifdef __APPLE__
    const char* opener = "open";
#else
    const char* opener = "xdg-open";
    if (!getenv("DISPLAY") && !getenv("WAYLAND_DISPLAY")) {
        printf("[bl] no display — not opening %s\n", path.c_str());
        return;
    }
#endif
    pid_t pid = fork();
    if (pid == 0) {
        setsid();
        int devnull = open("/dev/null", O_RDWR);
        if (devnull >= 0) { dup2(devnull, STDIN_FILENO); dup2(devnull, STDOUT_FILENO); dup2(devnull, STDERR_FILENO); }
        execlp(opener, opener, path.c_str(), (char*)nullptr);
        _exit(127);
    }
    if (pid < 0) fprintf(stderr, "[bl] could not open %s in a viewer\n", path.c_str());
}

// ── Plot ─────────────────────────────────────────────────────────────────────
// Hand-drawn with OpenCV, like circle_speed_test's renderPlot(), so the tool
// needs nothing it does not already link. Three panels:
//   speed vs time   raw orbit rows + a MEDIAN_WINDOW_S rolling median, rests shaded
//   voltage vs time loaded (orbit) and resting points, --stop-mv dashed
//   speed vs volts  orbit rows only, early light -> late dark: the correlation

struct LogPoint {
    double  tS;        // run time at the end of the row
    double  dtS;
    BlPhase phase;
    float   speedMms;  // NAN without a measurement
    int     mv;        // -1 without a valid reading
};

struct Axis {
    cv::Rect r;
    double x0, x1, y0, y1;
    cv::Point map(double x, double y) const {
        double fx = (x - x0) / (x1 - x0), fy = (y - y0) / (y1 - y0);
        fx = std::clamp(fx, 0.0, 1.0); fy = std::clamp(fy, 0.0, 1.0);
        return {r.x + (int)(fx * r.width), r.y + r.height - (int)(fy * r.height)};
    }
};

static double niceStep(double range, int ticks) {
    double raw = range / std::max(1, ticks);
    double mag = std::pow(10.0, std::floor(std::log10(raw)));
    for (double m : {1.0, 2.0, 5.0, 10.0}) if (m * mag >= raw) return m * mag;
    return 10.0 * mag;
}

static const cv::Scalar INK{40, 40, 40}, GRID{228, 228, 228}, REST_FILL{235, 235, 235};
static const cv::Scalar C_RAW{235, 190, 140}, C_MED{160, 70, 0}, C_LOAD{0, 120, 230},
                        C_REST{180, 110, 30}, C_SEEK{170, 170, 170}, C_STOP{40, 40, 200};

static void drawFrame(cv::Mat& img, const Axis& a, const char* xLabel, const char* yLabel,
                      double xScale, const char* xFmt, const char* yFmt) {
    // Ticks are chosen in display units (minutes, not seconds), so they land
    // on round numbers of what is printed. putText is Hershey: ASCII only.
    double xs = niceStep((a.x1 - a.x0) * xScale, 8) / xScale, ys = niceStep(a.y1 - a.y0, 5);
    for (double x = std::ceil(a.x0 / xs) * xs; x <= a.x1 + 1e-9; x += xs) {
        cv::Point p = a.map(x, a.y0);
        cv::line(img, {p.x, a.r.y}, {p.x, a.r.y + a.r.height}, GRID, 1);
        cv::putText(img, DemoHud::fmt(xFmt, x * xScale), {p.x - 12, a.r.y + a.r.height + 18},
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, INK, 1, cv::LINE_AA);
    }
    for (double y = std::ceil(a.y0 / ys) * ys; y <= a.y1 + 1e-9; y += ys) {
        cv::Point p = a.map(a.x0, y);
        cv::line(img, {a.r.x, p.y}, {a.r.x + a.r.width, p.y}, GRID, 1);
        cv::putText(img, DemoHud::fmt(yFmt, y), {a.r.x - 52, p.y + 4},
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, INK, 1, cv::LINE_AA);
    }
    cv::rectangle(img, a.r, INK, 1);
    cv::putText(img, xLabel, {a.r.x + a.r.width - 8 * (int)strlen(xLabel), a.r.y + a.r.height + 36},
                cv::FONT_HERSHEY_SIMPLEX, 0.45, INK, 1, cv::LINE_AA);
    cv::putText(img, yLabel, {a.r.x, a.r.y - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.5, INK, 1, cv::LINE_AA);
}

static void shadeRests(cv::Mat& img, const Axis& a, const std::vector<LogPoint>& pts) {
    for (auto& p : pts) {
        if (p.phase != BlPhase::Rest) continue;
        cv::Point p0 = a.map(p.tS - p.dtS, a.y1), p1 = a.map(p.tS, a.y0);
        cv::rectangle(img, p0, p1, REST_FILL, cv::FILLED);
    }
}

// Rows closer together than this are drawn joined; anything wider is a gap.
static bool joined(const LogPoint& a, const LogPoint& b) { return b.tS - a.tS <= ROW_S * 1.5 + b.dtS; }

static cv::Mat renderPlot(const std::vector<LogPoint>& pts, const std::string& title, int stopMv) {
    const int W = 1100, H = 1380, mL = 80, mR = 30;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(250, 250, 250));
    cv::putText(img, title, {mL, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.55, INK, 1, cv::LINE_AA);

    // Series for the smoothed trace: orbit rows only, everything else NAN so
    // the median never mixes a rest's zero into the orbit speed.
    std::vector<double> t; std::vector<float> v;
    for (auto& p : pts) { t.push_back(p.tS); v.push_back(p.phase == BlPhase::Orbit ? p.speedMms : NAN); }
    std::vector<float> med = blRollingMedian(t, v, MEDIAN_WINDOW_S);
    for (size_t i = 0; i < pts.size(); ++i) if (pts[i].phase != BlPhase::Orbit) med[i] = NAN;

    // Axes. Time in minutes, at least 5 so the first minutes do not stretch
    // across the whole panel. The speed axis is sized off the median's peak,
    // not the raw rows, so one bad detection cannot rescale the plot; the
    // peak comes early (the battery only drains), so in practice it is fixed
    // after the first minute.
    double tEnd = pts.empty() ? 0.0 : pts.back().tS;
    double xMax = std::max(5.0, std::ceil(tEnd / 60.0 / 5.0) * 5.0) * 60.0;
    float medMax = 0.f;
    for (float m : med) if (!std::isnan(m)) medMax = std::max(medMax, m);
    double sMax = medMax > 0.f ? niceStep(medMax * 1.4, 5) * std::ceil(medMax * 1.4 / niceStep(medMax * 1.4, 5)) : 100.0;
    int mvLo = stopMv - 200, mvHi = stopMv + 800;
    for (auto& p : pts) if (p.mv > 0) { mvLo = std::min(mvLo, p.mv - 100); mvHi = std::max(mvHi, p.mv + 100); }
    mvLo = mvLo / 100 * 100; mvHi = (mvHi + 99) / 100 * 100;

    Axis aS{{mL, 70, W - mL - mR, 720}, 0, xMax, 0, sMax};
    Axis aV{{mL, 860, W - mL - mR, 230}, 0, xMax, (double)mvLo, (double)mvHi};
    Axis aC{{mL, 1160, W - mL - mR, 170}, (double)mvLo, (double)mvHi, 0, sMax};

    // Speed vs time.
    shadeRests(img, aS, pts);
    drawFrame(img, aS, "time (min)", "measured speed (mm/s): orbit rows, 30 s median; grey = rest",
              1.0 / 60.0, "%.0f", "%.0f");
    for (size_t i = 0; i < pts.size(); ++i) {
        auto& p = pts[i];
        if (p.phase != BlPhase::Orbit || std::isnan(p.speedMms)) continue;
        cv::circle(img, aS.map(p.tS, p.speedMms), 2, C_RAW, cv::FILLED, cv::LINE_AA);
    }
    for (size_t i = 1; i < pts.size(); ++i)
        if (!std::isnan(med[i]) && !std::isnan(med[i - 1]) && joined(pts[i - 1], pts[i]))
            cv::line(img, aS.map(pts[i - 1].tS, med[i - 1]), aS.map(pts[i].tS, med[i]), C_MED, 2, cv::LINE_AA);

    // Voltage vs time.
    shadeRests(img, aV, pts);
    drawFrame(img, aV, "time (min)", "battery (mV): orange = under load, blue = resting, red = --stop-mv",
              1.0 / 60.0, "%.0f", "%.0f");
    for (double x = 0; x < xMax; x += xMax / 80.0) {
        cv::line(img, aV.map(x, stopMv), aV.map(x + xMax / 160.0, stopMv), C_STOP, 1, cv::LINE_AA);
    }
    for (auto& p : pts) {
        if (p.mv <= 0) continue;
        cv::Scalar c = p.phase == BlPhase::Orbit ? C_LOAD : p.phase == BlPhase::Rest ? C_REST : C_SEEK;
        cv::circle(img, aV.map(p.tS, p.mv), 2, c, cv::FILLED, cv::LINE_AA);
    }

    // Speed vs loaded voltage — the correlation.
    drawFrame(img, aC, "battery under load (mV)", "speed vs voltage: orbit rows, light = early, dark = late",
              1.0, "%.0f", "%.0f");
    for (auto& p : pts) {
        if (p.phase != BlPhase::Orbit || p.mv <= 0 || std::isnan(p.speedMms)) continue;
        double f = tEnd > 0.0 ? p.tS / tEnd : 0.0;
        cv::Scalar c(C_RAW[0] + f * (C_MED[0] - C_RAW[0]), C_RAW[1] + f * (C_MED[1] - C_RAW[1]),
                     C_RAW[2] + f * (C_MED[2] - C_RAW[2]));
        cv::circle(img, aC.map(p.mv, p.speedMms), 2, c, cv::FILLED, cv::LINE_AA);
    }
    return img;
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);

    std::string serial, ip, outDir = "battery_log_results";
    int    robotId   = -1;
    int    cmd       = 60;
    float  dirSign   = 1.f;       // +1 = counter-clockwise
    double maxTimeS  = 0.0;       // 0 = no limit
    bool   autoStart = false;
    bool   debug     = false;
    bool   openPlot  = true;
    BlSchedule   sched;
    BlStopConfig stopCfg;

    for (int i = 1; i < argc; ++i) {
        auto arg = [&](const char* n) { return strcmp(argv[i], n) == 0 && i + 1 < argc; };
        if      (arg("--robot"))      robotId  = atoi(argv[++i]);
        else if (arg("--cmd"))        cmd      = atoi(argv[++i]);
        else if (arg("--rest-every")) sched.restEveryS = atof(argv[++i]);
        else if (arg("--rest-for"))   sched.restForS   = atof(argv[++i]);
        else if (arg("--stop-mv"))    stopCfg.stopMv   = atoi(argv[++i]);
        else if (arg("--max-time"))   maxTimeS = atof(argv[++i]);
        else if (arg("--out"))        outDir   = argv[++i];
        else if (arg("--serial"))     serial   = argv[++i];
        else if (arg("--ip"))         ip       = argv[++i];
        else if (arg("--dir")) {
            const char* d = argv[++i];
            if      (!strcmp(d, "cw"))  dirSign = -1.f;
            else if (!strcmp(d, "ccw")) dirSign =  1.f;
            else { fprintf(stderr, "--dir must be cw or ccw, got: %s\n", d); return 2; }
        }
        else if (!strcmp(argv[i], "--start")) autoStart = true;
        else if (!strcmp(argv[i], "--debug")) debug     = true;
        else if (!strcmp(argv[i], "--no-open")) openPlot = false;
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("usage: %s --robot ID [--cmd N] [--dir cw|ccw]\n"
                   "       [--rest-every S] [--rest-for S] [--stop-mv MV] [--max-time S]\n"
                   "       [--start] [--out DIR] [--debug] [--no-open] [--serial SN] [--ip IP]\n\n"
                   "Orbits one robot on the saved ring at motor command N (default 60) until\n"
                   "its battery reads <= MV (default 4000) for %.0f s, it stalls, or it is\n"
                   "lost; logs vision speed + battery mV to DIR/*.csv and plots them to *.png.\n"
                   "Rests (motors off) every S (default 120, 0 = never) for S (default 12).\n"
                   "Start: <enter> on stdin, space in --debug, or --start. s/q ends the run.\n"
                   "When the run ends the final plot is opened in the image viewer (--no-open).\n",
                   argv[0], stopCfg.lowHoldS);
            return 0;
        } else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 2; }
    }
    if (robotId < 0 || robotId >= SC_MAX_ROBOTS) {
        fprintf(stderr, "--robot ID is required (0..%d)\n", SC_MAX_ROBOTS - 1);
        return 2;
    }
    if (cmd < 1 || cmd > 100) { fprintf(stderr, "--cmd must be 1..100\n"); return 2; }
    if (sched.restEveryS > 0.0 && sched.restForS <= 0.0) {
        fprintf(stderr, "--rest-for must be positive (or --rest-every 0 to disable rests)\n");
        return 2;
    }

    SwarmClient swarm;
    printf(swarm.connect() ? "[hub] connected\n" : "[hub] not available — will retry\n");

    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    cfg.debugOverlay = debug;
    const float renderIntervalS = cfg.renderFps > 0.f ? 1.f / cfg.renderFps : 1.f / 30.f;
    const float dbgScale = (cfg.debugFrameScale > 0.f && cfg.debugFrameScale <= 1.f) ? cfg.debugFrameScale : 1.f;

    ArucoTracker tracker(cfg);
    // Headless needs poses only, so it can ride on whoever already owns the camera
    // (a vision_hub, or another demo) instead of locking them out. --debug draws on
    // the frame, which only the owner has.
    const bool opened = debug ? tracker.open() : tracker.openOrAttach();
    if (!opened) {
        fprintf(stderr, "Could not open Basler camera.%s\n",
                debug ? " (--debug needs the frame itself, so it cannot attach to a vision_hub; "
                        "watch the hub's stream or run headless.)" : "");
        return 1;
    }
    if (tracker.subscribed())
        printf("[vision] attached to the pose publisher at %dx%d (no camera, no frame)\n",
               tracker.frameSize().width, tracker.frameSize().height);
    else
        printf("[vision] camera open at %dx%d\n",
               tracker.frameSize().width, tracker.frameSize().height);

    // Speed has to be in mm for the log to mean anything, so unlike
    // car_following there is no pixel fallback.
    if (!tracker.loadHomography(HOMOGRAPHY_FILE)) {
        fprintf(stderr, "[vision] no usable homography at %s — run tools/build/homography first (it also rejects one made without/with a different lens correction)\n",
                HOMOGRAPHY_FILE.c_str());
        return 1;
    }
    {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        cv::Mat H;
        if (fs.isOpened()) fs["H"] >> H;
        if (!H.empty()) g_Hinv = H.inv();
    }

    // No fitting and no default: the ring is a fixture, and a wrong radius
    // here is a wrong speed in every row. Set it in car_following or circle_demo.
    Ring ring;
    std::string ringSrc = RING_FILE;
    if (!loadRing(ring, RING_FILE)) { ringSrc = CIRCLE_FILE; loadRing(ring, CIRCLE_FILE); }
    if (!ring.centreSet || ring.radius <= 0.f) {
        fprintf(stderr, "[ring] no saved ring in %s or %s — set one with car_following or circle_demo\n",
                RING_FILE.c_str(), CIRCLE_FILE.c_str());
        return 1;
    }
    printf("[ring] centre=(%.0f, %.0f) radius=%.0f mm <- %s\n",
           ring.centre.x, ring.centre.y, ring.radius, ringSrc.c_str());

    const char* WIN  = "Battery Log";
    const char* PWIN = "Battery Log - plot";
    if (debug) {
        cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
        cv::resizeWindow(WIN, (int)(tracker.frameSize().width * dbgScale),
                              (int)(tracker.frameSize().height * dbgScale));
        cv::namedWindow(PWIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
        cv::resizeWindow(PWIN, 1100, 1020);
    }

    // ── State ────────────────────────────────────────────────────────────────
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    auto now = t0;
    auto secondsSince = [&](clock::time_point t) { return std::chrono::duration<double>(now - t).count(); };

    BlPhase phase = BlPhase::Setup;
    double  phaseStartS = 0.0, runStartS = 0.0, orbitS = 0.0;
    bool    startRequested = autoStart, endRequested = false;
    std::string endNote;

    bool   havePose = false, everSeen = false, registered = false;
    double firstSeenS = 0.0, lastSeenS = -1e9;
    RobotPose pose{};
    float  yawF = 0.f; bool yawInit = false;
    RateEstimator yawRate;
    float  prevTurn = 0.f;
    int8_t motorL = 0, motorR = 0;
    bool   motorsDirty = true;

    BlRowAccumulator acc;
    BlStop stop(stopCfg);
    std::vector<LogPoint> points;
    std::ofstream csv;
    std::string csvPath, pngPath, title;

    auto lastFrame = t0, lastControl = t0, lastMotorTx = t0, lastHubRetry = t0;
    double lastStatusS = 0.0, lastPlotSaveS = 0.0, lastPlotDrawS = -1e9, lastRenderS = -1e9;

    auto tNowS = [&]() { return secondsSince(t0); };

    // The battery as the log should see it: valid only if the robot has
    // measured it (BAT_VALID) and the telemetry carrying it is fresh.
    auto battery = [&](int& mv, bool& valid, bool& lowFlag) {
        const auto& st = swarm.robotState((uint8_t)robotId);
        bool fresh = st.hasTelemetry &&
                     std::chrono::duration<float>(now - st.lastSeen).count() <= TELEMETRY_STALE_S;
        valid   = fresh && (st.flags & SC_STATUS_BAT_VALID) && st.batteryMv > 0;
        lowFlag = fresh && (st.flags & SC_STATUS_LOW_BATTERY);
        mv      = valid ? (int)st.batteryMv : -1;
    };

    auto sendMotors = [&](bool force) {
        if (!force && !motorsDirty && secondsSince(lastMotorTx) < MOTOR_KEEPALIVE_S) return;
        swarm.setSpeed((uint8_t)robotId, motorL, motorR);
        swarm.flush();
        lastMotorTx = now;
        motorsDirty = false;
    };
    auto setMotors = [&](int8_t l, int8_t r) {
        if (l != motorL || r != motorR) motorsDirty = true;
        motorL = l; motorR = r;
    };

    auto savePlot = [&]() {
        if (pngPath.empty()) return;
        cv::imwrite(pngPath, renderPlot(points, title, stopCfg.stopMv));
        lastPlotSaveS = tNowS();
    };

    // Closes the current row under the phase it was measured in.
    auto emitRow = [&]() {
        if (phase == BlPhase::Setup || phase == BlPhase::Done || !csv.is_open()) return;
        double t = tNowS();
        BlRow r = acc.take(t);
        if (r.dtS <= 0.0) return;
        if (phase == BlPhase::Orbit) orbitS += r.dtS;
        int mv; bool valid, lowFlag;
        battery(mv, valid, lowFlag);
        stop.row(t, phase, r, orbitS);

        auto num = [](float x, const char* f) { return std::isnan(x) ? std::string() : DemoHud::fmt(f, x); };
        csv << DemoHud::fmt("%.3f", t - runStartS) << ',' << wallTime("%Y-%m-%dT%H:%M:%S") << ','
            << robotId << ',' << blPhaseName(phase) << ',' << DemoHud::fmt("%.3f", r.dtS) << ','
            << cmd << ',' << (int)motorL << ',' << (int)motorR << ','
            << num(r.speedMms, "%.2f") << ',' << num(r.radialErrMm, "%.1f") << ','
            << num(r.absTurn, "%.2f") << ',' << r.nFrames << ',' << (r.visible() ? 1 : 0) << ','
            << (valid ? std::to_string(mv) : std::string()) << ',' << (valid ? 1 : 0) << ','
            << (lowFlag ? 1 : 0) << '\n';
        csv.flush();   // a crash hours in should cost one row, not the run
        points.push_back({t - runStartS, r.dtS, phase, r.speedMms, valid ? mv : -1});
    };

    auto setPhase = [&](BlPhase p, const char* why) {
        if (p == phase) return;
        emitRow();
        printf("[bl] %s -> %s (%s)\n", blPhaseName(phase), blPhaseName(p), why);
        phase = p;
        phaseStartS = tNowS();
        prevTurn = 0.f;
        yawRate = RateEstimator{};
    };

    auto startRun = [&]() {
        std::error_code ec;
        std::filesystem::create_directories(outDir, ec);
        std::string stem = DemoHud::fmt("%s/battery_log_r%d_c%d_%s", outDir.c_str(), robotId, cmd,
                                        wallTime("%Y%m%d_%H%M%S").c_str());
        csvPath = stem + ".csv";
        pngPath = stem + ".png";
        csv.open(csvPath);
        if (!csv) {
            fprintf(stderr, "[bl] could not open %s\n", csvPath.c_str());
            return false;
        }
        csv << "# battery_log run\n"
            << "# started: " << wallTime("%Y-%m-%dT%H:%M:%S") << '\n'
            << "# robot: " << robotId << "\n# cmd: " << cmd << "\n# dir: " << (dirSign > 0 ? "ccw" : "cw") << '\n'
            << DemoHud::fmt("# ring: cx=%.1f cy=%.1f radius=%.1f (%s)\n",
                            ring.centre.x, ring.centre.y, ring.radius, ringSrc.c_str())
            << "# rest_every_s: " << sched.restEveryS << "\n# rest_for_s: " << sched.restForS << '\n'
            << "# stop_mv: " << stopCfg.stopMv << "\n# max_time_s: " << maxTimeS << '\n'
            << "# assumes open-loop robot firmware (ODOMETRY_ENABLED = False)\n"
            << "t_s,wall_iso,robot,phase,dt_s,cmd,motor_l,motor_r,speed_mms,radial_err_mm,"
               "abs_turn,n_frames,visible,bat_mv,bat_valid,low_bat_flag\n";
        csv.flush();
        title = DemoHud::fmt("robot %d   cmd %d   %s   ring R=%.0f mm   started %s",
                             robotId, cmd, dirSign > 0 ? "ccw" : "cw", ring.radius,
                             wallTime("%Y-%m-%d %H:%M").c_str());
        runStartS = tNowS();
        acc.begin(runStartS);
        printf("[bl] logging to %s\n", csvPath.c_str());
        return true;
    };

    printf("[bl] setup — robot %d held still. %s\n", robotId,
           debug ? "Press space to start, s/q to end." : "Press <enter> to start, s/q to end.");

    // ── Loop ─────────────────────────────────────────────────────────────────
    while (g_running && phase != BlPhase::Done) {
        bool haveFrame = tracker.update();
        now = clock::now();
        const double t = tNowS();

        if (!swarm.isConnected() && secondsSince(lastHubRetry) >= 2.0) {
            lastHubRetry = now;
            if (swarm.connect()) printf("[hub] connected\n");
        }
        swarm.poll();

        if (!debug) {
            std::string line;
            if (readStdinLine(line)) {
                if      (line == "q" || line == "quit" || line == "s" || line == "stop") endRequested = true;
                else if (line.empty() || line == "g" || line == "go" || line == "start") startRequested = true;
                else printf("[bl] <enter>/go = start, s/q = end\n");
            }
        }

        // ── Pose ─────────────────────────────────────────────────────────────
        // Yaw low-pass once per frame, as car_following does (see YAW_TAU_S there).
        if (haveFrame) {
            float frameDt = clampf((float)secondsSince(lastFrame), 0.001f, 0.2f);
            lastFrame = now;
            havePose = false;
            for (auto& r : tracker.robots()) if (r.id == robotId) { pose = r; havePose = true; break; }
            if (havePose) {
                if (!everSeen) { everSeen = true; firstSeenS = t; }
                lastSeenS = t;
                float alpha = frameDt / (YAW_TAU_S + frameDt);
                if (!yawInit) { yawF = pose.yaw; yawInit = true; }
                else yawF = normAngle(yawF + alpha * normAngle(pose.yaw - yawF));
                pose.yaw = yawF;
            }
            if (everSeen && !registered && t - firstSeenS >= REGISTER_DEBOUNCE_S) {
                swarm.registerRobot((uint8_t)robotId);
                registered = true;
            }
        }
        const double unseenS = everSeen ? t - lastSeenS : 1e9;
        const bool   visible = unseenS <= MOTOR_HOLD_S;
        float dx = pose.x - ring.centre.x, dy = pose.y - ring.centre.y;
        float distC = std::hypot(dx, dy);

        // ── Phase ────────────────────────────────────────────────────────────
        if (phase == BlPhase::Setup && startRequested && swarm.isConnected() && visible) {
            startRequested = false;
            if (!startRun()) break;
            setPhase(BlPhase::Seek, "start");
        }
        if (phase == BlPhase::Orbit && unseenS > LOST_RESEEK_S) setPhase(BlPhase::Seek, "robot lost");
        // A hub dropout stops the robot (its own watchdog), which would read
        // as a stall ten seconds later and end a run that took hours to get
        // there. Treat it like a lost robot instead: reseek once it is back.
        if (phase == BlPhase::Orbit && !swarm.isConnected()) setPhase(BlPhase::Seek, "hub lost");
        // Only time the robot could actually be driven counts toward the seek
        // timeout; an unseen robot is the lost-robot check's business.
        if (phase == BlPhase::Seek && (!visible || !swarm.isConnected())) phaseStartS = t;
        if (phase == BlPhase::Seek && visible && std::fabs(distC - ring.radius) <= SEEK_ARRIVAL_MM)
            setPhase(BlPhase::Orbit, "on the ring");
        if (phase == BlPhase::Orbit || phase == BlPhase::Rest) {
            BlPhase next = sched.next(phase, t - phaseStartS);
            if (next != phase) setPhase(next, next == BlPhase::Rest ? "rest" : "rest over");
        }

        if (phase != BlPhase::Setup) {
            if (haveFrame) acc.frame(havePose, pose.x, pose.y, ring.centre.x, ring.centre.y,
                                     ring.radius, dirSign, prevTurn, t);
            if (t - acc.startS() >= ROW_S) emitRow();

            int mv; bool valid, lowFlag;
            battery(mv, valid, lowFlag);
            stop.battery(t, mv, valid, lowFlag);
            stop.tick(phase, t - phaseStartS, unseenS);

            if (stop.stopped())                                endNote = blStopReasonName(stop.reason());
            else if (maxTimeS > 0.0 && t - runStartS >= maxTimeS) endNote = "max time";
            else if (endRequested || !g_running)               endNote = "user";
            if (!endNote.empty()) setPhase(BlPhase::Done, endNote.c_str());
        } else if (endRequested) {
            break;
        }

        // ── Control ──────────────────────────────────────────────────────────
        if (haveFrame || secondsSince(lastControl) >= CONTROL_STALL_S) {
            float controlDt = clampf((float)secondsSince(lastControl), 0.001f, 0.2f);
            lastControl = now;
            int8_t l = 0, r = 0;
            bool drive = (phase == BlPhase::Seek || phase == BlPhase::Orbit) && visible;
            if (drive) {
                float yr = updateRate(yawRate, pose.yaw, now, D_TERM_WINDOW_S);
                if (distC >= 1.f) {
                    float rx = dx / distC, ry = dy / distC;
                    float tx = dirSign * -ry, ty = dirSign * rx;
                    // Seek is radial only; orbit feeds --cmd straight in as
                    // the tangential speed, in motor units like circle_demo.
                    float vTan = phase == BlPhase::Orbit ? (float)cmd : 0.f;
                    float vRad = clampf(-K_RAD * (distC - ring.radius), -MOTOR_MAX * 0.5f, MOTOR_MAX * 0.5f);
                    float vx = vTan * tx + vRad * rx, vy = vTan * ty + vRad * ry;
                    float vMag = std::hypot(vx, vy);
                    if (vMag >= 0.5f) {
                        float angleErr  = normAngle(atan2f(vy, vx) * RAD2DEG - pose.yaw);
                        float headingN  = clampf(fabsf(angleErr) / 90.f, 0.f, 1.f);
                        float headingSc = 1.f - headingN * headingN;
                        float ffOmega   = dirSign * (vTan / ring.radius) * RAD2DEG;
                        float dErr      = clampf(ffOmega - yr, -300.f, 300.f);
                        float forward   = clampf(vMag, 0.f, MOTOR_MAX) * headingSc;
                        float turnTgt   = clampf(K_FF_YAW * ffOmega * headingSc
                                                 + K_ANGLE * angleErr + K_YAW_D * dErr,
                                                 -MAX_TURN, MAX_TURN);
                        float maxStep   = MAX_TURN_RATE * controlDt;
                        float turn      = clampf(turnTgt, prevTurn - maxStep, prevTurn + maxStep);
                        prevTurn = turn;
                        l = (int8_t)clampf(forward + turn, -MOTOR_MAX, MOTOR_MAX);
                        r = (int8_t)clampf(forward - turn, -MOTOR_MAX, MOTOR_MAX);
                    } else {
                        prevTurn = 0.f;
                    }
                }
            } else {
                prevTurn = 0.f;
            }
            setMotors(l, r);
            if (registered) sendMotors(false);
        }

        // ── Status, plot ─────────────────────────────────────────────────────
        if (t - lastStatusS >= 1.0) {
            lastStatusS = t;
            int mv; bool valid, lowFlag;
            battery(mv, valid, lowFlag);
            float spd = points.empty() ? NAN : points.back().speedMms;
            printf("[bl] %s %-5s  speed %s  bat %s  radial %+.0f mm  %s  hub:%s\n",
                   phase == BlPhase::Setup ? "--:--:--" : hms(t - runStartS).c_str(),
                   blPhaseName(phase),
                   std::isnan(spd) ? "   -   " : DemoHud::fmt("%5.1f mm/s", spd).c_str(),
                   valid ? DemoHud::fmt("%.2f V", mv / 1000.0).c_str() : "  -  ",
                   visible ? distC - ring.radius : 0.f,
                   visible ? "seen" : "UNSEEN",
                   swarm.isConnected() ? "ok" : "OFFLINE");
            fflush(stdout);
        }
        if (phase != BlPhase::Setup && t - lastPlotSaveS >= PLOT_SAVE_S) savePlot();

        if (debug) {
            if (!points.empty() && t - lastPlotDrawS >= PLOT_REDRAW_S) {
                lastPlotDrawS = t;
                cv::imshow(PWIN, renderPlot(points, title, stopCfg.stopMv));
            }
            if (t - lastRenderS >= renderIntervalS) {
                lastRenderS = t;
                cv::Mat disp = tracker.debugFrame().clone();
                if (!disp.empty()) {
                    cv::Point c = worldToPixel(ring.centre);
                    cv::Point e = worldToPixel({ring.centre.x + ring.radius, ring.centre.y});
                    cv::circle(disp, c, (int)cv::norm(e - c), {0, 200, 255}, 2, cv::LINE_AA);
                    if (visible)
                        cv::circle(disp, {(int)pose.px, (int)pose.py}, 24,
                                   phase == BlPhase::Orbit ? cv::Scalar(0, 140, 255)
                                 : phase == BlPhase::Seek  ? cv::Scalar(0, 220, 0)
                                                           : cv::Scalar(200, 200, 200), 2, cv::LINE_AA);
                    int mv; bool valid, lowFlag;
                    battery(mv, valid, lowFlag);
                    float spd = points.empty() ? NAN : points.back().speedMms;
                    ArucoTracker::drawText(disp, DemoHud::fmt("robot %d  %s  %s  %s", robotId,
                        blPhaseName(phase),
                        std::isnan(spd) ? "-" : DemoHud::fmt("%.0f mm/s", spd).c_str(),
                        valid ? DemoHud::fmt("%.2f V", mv / 1000.0).c_str() : "- V"),
                        {20, 60}, 40, {0, 255, 180});
                    if (dbgScale < 1.f) cv::resize(disp, disp, {}, dbgScale, dbgScale, cv::INTER_AREA);
                    cv::imshow(WIN, disp);
                }
            }
            int key = cv::waitKey(1) & 0xFF;
            if (key == ' ') startRequested = true;
            if (key == 's' || key == 'q' || key == 27) endRequested = true;
        }
    }

    // Ctrl-C leaves the loop without passing through Done; close its row.
    if (phase != BlPhase::Setup && phase != BlPhase::Done) { now = clock::now(); emitRow(); }

    // Zero the motors more than once: a single frame can be lost to a
    // reconnect. The robot's own watchdog is the backstop if none lands.
    setMotors(0, 0);
    for (int i = 0; i < 3; ++i) {
        now = clock::now();
        sendMotors(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (csv.is_open()) {
        if (endNote.empty()) endNote = "user";
        csv << "# ended: " << wallTime("%Y-%m-%dT%H:%M:%S") << " after " << hms(tNowS() - runStartS)
            << " (" << endNote << "), stall baseline " << DemoHud::fmt("%.1f", stop.baseline()) << " mm/s\n";
        csv.close();
        title += "   ended: " + endNote + " after " + hms(tNowS() - runStartS);
        savePlot();
        printf("[bl] ended (%s) after %s — %zu rows\n[bl] wrote %s\n[bl] wrote %s\n",
               endNote.c_str(), hms(tNowS() - runStartS).c_str(), points.size(),
               csvPath.c_str(), pngPath.c_str());
        if (openPlot) openInViewer(pngPath);
    }
    if (debug) cv::destroyAllWindows();
    return 0;
}
