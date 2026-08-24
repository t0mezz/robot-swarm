// marker_eval.cpp — Camera telemetry display
// Usage: ./marker_eval [--config JSON] [--serial SN] [--ip IP] [--mirror]
//
// Shows live camera feed with a telemetry panel:
//   Resolution, FPS, pipeline latency, sensor temperature, marker count
//
// Run this first to verify camera health and detection quality
// before launching any controller.
//
// Keys: r = reset FPS/latency counters,  q / Esc = quit
//
// BRANCH-ONLY: this tool also opens a second "Color Ramp Debug" window that
// previews ArucoConfig::colorRampEnabled's contrast stretch (see aruco_tracker.h)
// applied to the live frame, so the two windows can be eyeballed side by side
// while judging whether the ramp helps detection consistency. Not meant to
// merge to main — see feature/color-ramp-input.

#include "aruco_tracker.h"
#include "DemoHud.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <algorithm>

// ── panel ─────────────────────────────────────────────────────────────────────
//
// The on-screen telemetry panel is built entirely from the shared DemoHud
// overlay (lib/DemoHud/DemoHud.h) — no bespoke cv::putText/snprintf layout
// code lives here anymore. DemoHud owns sizing, the translucent background,
// and column auto-fit; this file only decides *what* to show.

static cv::Scalar tempColor(float t) {
    if (t < 0.f)  return {110, 110, 110};   // unavailable
    if (t < 60.f) return DemoHud::COL_OK;    // normal
    if (t < 70.f) return DemoHud::COL_WARN;  // warm
    return               DemoHud::COL_BAD;   // hot
}

static void drawPanel(cv::Mat& img, float loopFps, float detFps, float latMs,
                      float tempC, int w, int h, int markerCount)
{
    DemoHud hud;
    hud.title("CAMERA", DemoHud::COL_TEXT);
    hud.row("Resolution", DemoHud::fmt("%d x %d px", w, h));
    hud.row("Det FPS",    DemoHud::fmt("%.1f", detFps));
    hud.row("Loop FPS",   DemoHud::fmt("%.1f", loopFps), DemoHud::COL_OK);
    hud.row("Latency",    DemoHud::fmt("%.1f ms", latMs));
    hud.row("Temp",       tempC >= 0.f ? DemoHud::fmt("%.1f C", tempC) : "n/a",
                          tempColor(tempC));
    hud.row("Markers",    DemoHud::fmt("%d", markerCount),
                          markerCount > 0 ? DemoHud::COL_OK : DemoHud::COL_TEXT);
    hud.row("Keys",       "r=reset  q=quit");

    // Right-anchor the panel: the tracker's debugFrame() burns its own
    // small det_fps/tags HUD + legend into the top-left corner, so anchoring
    // there would put this now-3x panel right on top of that text.
    cv::Size sz = hud.measure();
    int x = std::max(10, img.cols - sz.width - 10);
    hud.draw(img, {x, 10});
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::string configPath = ArucoConfig::defaultConfigPath();
    std::string serial, ip;
    bool cfg_mirror = false;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--config") && i+1<argc) configPath = argv[++i];
        else if (!strcmp(argv[i], "--serial") && i+1<argc) serial     = argv[++i];
        else if (!strcmp(argv[i], "--ip")     && i+1<argc) ip         = argv[++i];
        else if (!strcmp(argv[i], "--mirror"))              cfg_mirror = true;
        else fprintf(stderr, "[eval] unknown arg '%s'\n", argv[i]);
    }

    ArucoConfig cfg = ArucoConfig::fromFile(configPath);
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    if (cfg_mirror)      cfg.mirrorInput  = true;
    cfg.debugOverlay = true;

    ArucoTracker tracker(cfg);
    if (!tracker.open()) {
        fprintf(stderr, "[eval] failed to open camera\n");
        return 1;
    }

    auto sz = tracker.frameSize();
    printf("[eval] %dx%d  r=reset  q=quit\n", sz.width, sz.height);

    // Qt-based HighGUI (Linux build here) adds a toolbar/statusbar around the
    // image by default; WINDOW_GUI_NORMAL suppresses that so the window looks
    // like the plain Cocoa HighGUI window on macOS.
    cv::namedWindow("Camera Eval", cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow("Camera Eval", sz.width, sz.height);

    // BRANCH-ONLY: preview window for cfg.color_ramp_low/high (see the
    // comment block at the top of this file). Independent of whether
    // colorRampEnabled actually feeds the detector — this just visualizes
    // the same LUT against the live debug frame for eyeballing.
    cv::namedWindow("Color Ramp Debug", cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow("Color Ramp Debug", sz.width, sz.height);
    uchar rampLut[256];
    buildColorRampLUT(rampLut, cfg.colorRampLow, cfg.colorRampHigh);
    printf("[eval] color ramp preview: low=%.1f high=%.1f (enabled=%d)\n",
           cfg.colorRampLow, cfg.colorRampHigh, (int)cfg.colorRampEnabled);

    // loop_fps = this display/render loop's rate (distinct from
    // tracker.detectionFps(), the detection thread's rate). Tick every iteration.
    DemoHud::LoopFps loopFps;

    while (true) {
        loopFps.tick();
        if (tracker.update()) {
            cv::Mat frame = tracker.debugFrame();
            drawPanel(frame, loopFps.fps(), tracker.detectionFps(), tracker.latencyMs(),
                      tracker.cameraTemperature(),
                      sz.width, sz.height,
                      (int)tracker.robots().size());
            cv::imshow("Camera Eval", frame);

            cv::Mat gray, ramped;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::LUT(gray, cv::Mat(1, 256, CV_8U, rampLut), ramped);
            ArucoTracker::drawText(ramped, "ramp preview  lo:" + std::to_string((int)cfg.colorRampLow)
                                    + " hi:" + std::to_string((int)cfg.colorRampHigh),
                                    {10, 28}, 24, {255});
            cv::imshow("Color Ramp Debug", ramped);
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 'r') tracker.requestStatsReset();
    }

    cv::destroyAllWindows();
    return 0;
}
