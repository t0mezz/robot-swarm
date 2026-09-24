// measurement_test.cpp — Homography sanity check: click points on the live
// camera feed and read the real-world (mm) distance between them, to verify
// against a tape measure that the stage's calibration is actually correct
// rather than just plausible-looking. No robots or SwarmClient involved —
// this only exercises the camera + homography, like marker_eval.cpp does for
// detection quality.
//
// Usage: ./measurement_test [--serial SN] [--ip IP] [--calibrate]
//
// Controls:
//   left-click    drop a measurement point (builds a polyline ruler)
//   right-click,c clear all points
//   z             undo the last point
//   k             attach a known/expected length to the last segment (typed
//                 on stdin) and report the measured error against it
//   grid          toggle a 100mm world-space grid overlay — a skewed or
//   (key: g)      unevenly-spaced grid is a fast visual tell for a bad fit
//   r             (re)calibrate the homography (click 4 corners)
//   q, Esc        quit

#include "aruco_tracker.h"
#include "DemoHud.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <csignal>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>

static const char* HOMOGRAPHY_FILE = "/tmp/aruco_homography.yml";
static constexpr int   MAX_POINTS = 20;
static constexpr float GRID_STEP_MM = 100.f;

static volatile bool g_running = true;
static void onSignal(int) { g_running = false; }

// ── Homography state + pixel<->world helpers (same shape as drag_drop_demo) ──

static cv::Mat g_H, g_Hinv;
static bool    g_hasH = false;

static void setH(const cv::Mat& H) {
    g_H = H;
    g_hasH = !g_H.empty();
    if (g_hasH) g_Hinv = g_H.inv();
}

static cv::Point2f pixelToWorld(cv::Point2f px) {
    if (!g_hasH) return px;
    std::vector<cv::Point2f> src = {px}, dst;
    cv::perspectiveTransform(src, dst, g_H);
    return dst[0];
}
static cv::Point2f worldToPixel(cv::Point2f w) {
    if (!g_hasH) return w;
    std::vector<cv::Point2f> src = {w}, dst;
    cv::perspectiveTransform(src, dst, g_Hinv);
    return dst[0];
}

// ── Points ────────────────────────────────────────────────────────────────────

struct Segment { float knownMm = -1.f; };  // -1 = no known length entered

static std::vector<cv::Point2f> g_pts;      // pixel-space clicks
static std::vector<Segment>     g_segs;     // one per consecutive pair

static void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        if ((int)g_pts.size() >= MAX_POINTS) {
            printf("[measure] point limit (%d) reached — clear with 'c'.\n", MAX_POINTS);
            return;
        }
        g_pts.push_back({(float)x, (float)y});
        if (g_pts.size() >= 2) g_segs.push_back({});
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        g_pts.clear();
        g_segs.clear();
    }
}

// ── Calibration (click 4 corners, enter arena size) ──────────────────────────

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
    cv::Mat H_ = cv::findHomography(cs.pixPts, worldPts);
    if (H_.empty()) return false;
    tracker.setHomography(cs.pixPts, worldPts);
    tracker.saveHomography(HOMOGRAPHY_FILE);
    setH(H_);
    printf("Saved homography.\n");
    return true;
}

// ── Grid overlay ──────────────────────────────────────────────────────────────
//
// Transforms the frame's 4 corners into world space to get a bounding box,
// then draws every GRID_STEP_MM line inside it, mapped back to pixels. A good
// homography renders this as straight, evenly-spaced, near-square cells; skew,
// bowing or uneven spacing is a bad fit even before any click is measured.
static void drawGrid(cv::Mat& disp, cv::Size frameSz) {
    if (!g_hasH) return;
    std::vector<cv::Point2f> corners = {
        {0,0}, {(float)frameSz.width,0}, {(float)frameSz.width,(float)frameSz.height}, {0,(float)frameSz.height}
    };
    std::vector<cv::Point2f> worldCorners;
    cv::perspectiveTransform(corners, worldCorners, g_H);

    float minX = worldCorners[0].x, maxX = worldCorners[0].x;
    float minY = worldCorners[0].y, maxY = worldCorners[0].y;
    for (auto& p : worldCorners) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }

    cv::Scalar col(80, 80, 80);
    float x0 = std::floor(minX / GRID_STEP_MM) * GRID_STEP_MM;
    for (float x = x0; x <= maxX; x += GRID_STEP_MM) {
        cv::Point2f a = worldToPixel({x, minY}), b = worldToPixel({x, maxY});
        cv::line(disp, a, b, col, 1, cv::LINE_AA);
    }
    float y0 = std::floor(minY / GRID_STEP_MM) * GRID_STEP_MM;
    for (float y = y0; y <= maxY; y += GRID_STEP_MM) {
        cv::Point2f a = worldToPixel({minX, y}), b = worldToPixel({maxX, y});
        cv::line(disp, a, b, col, 1, cv::LINE_AA);
    }
    ArucoTracker::drawText(disp, DemoHud::fmt("grid: %.0fmm", GRID_STEP_MM),
        {10, frameSz.height - 14}, 16, col);
}

int main(int argc, char* argv[]) {
    signal(SIGINT, onSignal); signal(SIGTERM, onSignal); signal(SIGPIPE, SIG_IGN);
    cv::setNumThreads((int)std::thread::hardware_concurrency());
    cv::setUseOptimized(true);

    std::string serial, ip;
    bool doCalib = false;
    bool showGrid = false;

    for (int i = 1; i < argc; i++) {
        auto arg = [&](const char* n) { return strcmp(argv[i], n) == 0 && i + 1 < argc; };
        if      (arg("--serial")) serial = argv[++i];
        else if (arg("--ip"))     ip     = argv[++i];
        else if (!strcmp(argv[i], "--calibrate")) doCalib = true;
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("usage: %s [--serial SN] [--ip IP] [--calibrate]\n", argv[0]);
            return 0;
        } else { fprintf(stderr, "unknown argument: %s\n", argv[i]); return 2; }
    }

    auto cfg = ArucoConfig::fromFile();
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    cfg.debugOverlay = true;
    ArucoTracker tracker(cfg);
    if (!tracker.open()) { fprintf(stderr, "Could not open camera.\n"); return 1; }
    printf("Camera: %dx%d\n", tracker.frameSize().width, tracker.frameSize().height);

    const char* WIN = "Measurement Test";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, tracker.frameSize().width, tracker.frameSize().height);
    cv::setMouseCallback(WIN, onMouse, nullptr);

    if (doCalib) {
        runCalibration(tracker, WIN);
    } else if (tracker.loadHomography(HOMOGRAPHY_FILE)) {
        cv::FileStorage fs(HOMOGRAPHY_FILE, cv::FileStorage::READ);
        cv::Mat H;
        if (fs.isOpened()) fs["H"] >> H;
        setH(H);
        printf("Loaded homography.\n");
    } else {
        printf("No homography loaded — run with --calibrate, or press 'r' now.\n");
    }

    printf("\nleft-click = add point   right-click/c = clear   z = undo\n"
           "k = enter known length for last segment   g = toggle grid   r = recalibrate   q = quit\n\n");

    while (g_running) {
        if (!tracker.update()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }

        cv::Mat disp = tracker.debugFrame().clone();
        cv::Size sz = tracker.frameSize();

        if (showGrid) drawGrid(disp, sz);

        // Points, segments and per-segment mm labels.
        float totalMm = 0.f;
        for (size_t i = 0; i < g_pts.size(); i++) {
            cv::circle(disp, g_pts[i], 6, {0, 0, 255}, -1, cv::LINE_AA);
            ArucoTracker::drawText(disp, std::to_string(i), g_pts[i] + cv::Point2f(10, -10), 16, {0, 0, 255});
            if (i == 0) continue;
            cv::Point2f a = g_pts[i-1], b = g_pts[i];
            cv::line(disp, a, b, {0, 220, 255}, 2, cv::LINE_AA);

            float mm = g_hasH
                ? cv::norm(pixelToWorld(b) - pixelToWorld(a))
                : cv::norm(b - a);
            totalMm += mm;

            std::string label = g_hasH ? DemoHud::fmt("%.1f mm", mm) : DemoHud::fmt("%.0f px (no H)", mm);
            const Segment& seg = g_segs[i-1];
            cv::Scalar col = {0, 220, 255};
            if (seg.knownMm > 0.f) {
                float err = mm - seg.knownMm;
                float pct = 100.f * err / seg.knownMm;
                label += DemoHud::fmt("  (known %.1f, err %+.1f / %+.1f%%)", seg.knownMm, err, pct);
                col = std::fabs(pct) < 2.f ? DemoHud::COL_OK : std::fabs(pct) < 5.f ? DemoHud::COL_WARN : DemoHud::COL_BAD;
            }
            cv::Point2f mid = (a + b) * 0.5f;
            ArucoTracker::drawText(disp, label, mid + cv::Point2f(8, 0), 16, col);
        }

        DemoHud hud;
        hud.title(DemoHud::fmt("MEASUREMENT TEST  H:%s  grid:%s",
                                g_hasH ? "ok" : "MISSING", showGrid ? "on" : "off"),
                  g_hasH ? DemoHud::COL_TEXT : DemoHud::COL_BAD);
        hud.row("Points",       DemoHud::fmt("%d", (int)g_pts.size()));
        hud.row("Segments",     DemoHud::fmt("%d", (int)g_segs.size()));
        hud.row("Path length",  g_hasH ? DemoHud::fmt("%.1f mm", totalMm) : "-- (no homography)");
        hud.row("Keys", "click=point  right/c=clear  z=undo  k=known-len  g=grid  r=calib  q=quit");
        hud.drawTopRight(disp);

        cv::imshow(WIN, disp);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) g_running = false;
        else if (key == 'c') { g_pts.clear(); g_segs.clear(); }
        else if (key == 'z' && !g_pts.empty()) {
            g_pts.pop_back();
            if (!g_segs.empty()) g_segs.pop_back();
        }
        else if (key == 'g') showGrid = !showGrid;
        else if (key == 'r') {
            bool ok = runCalibration(tracker, WIN);
            if (ok) g_hasH = true;
        }
        else if (key == 'k' && !g_segs.empty() && g_hasH) {
            printf("Known length for last segment (mm): ");
            fflush(stdout);
            float known = -1.f;
            if (scanf("%f", &known) == 1 && known > 0.f) {
                g_segs.back().knownMm = known;
                cv::Point2f a = g_pts[g_pts.size()-2], b = g_pts[g_pts.size()-1];
                float mm = (float)cv::norm(pixelToWorld(b) - pixelToWorld(a));
                printf("[measure] measured=%.2f mm  known=%.2f mm  error=%+.2f mm (%+.2f%%)\n",
                       mm, known, mm - known, 100.f * (mm - known) / known);
            }
        }
    }

    return 0;
}
