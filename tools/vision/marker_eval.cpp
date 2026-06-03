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

#include "aruco_tracker.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <algorithm>

// ── panel ─────────────────────────────────────────────────────────────────────

static constexpr int kW   = 210;
static constexpr int kPad = 12;

static cv::Scalar tempColor(float t) {
    if (t < 0.f)  return {110, 110, 110};  // unavailable
    if (t < 60.f) return { 50, 220,  70};  // normal
    if (t < 70.f) return { 30, 210, 255};  // warm
    return               { 40,  60, 230};  // hot
}

static void drawPanel(cv::Mat& img, float fps, float latMs,
                      float tempC, int w, int h, int markerCount)
{
    // panel height: title + divider + 5 data rows + divider + hint
    constexpr int kPanelH = 36 + 14 + 5*26 + 14 + 22 + kPad * 2;
    int panelH = std::min(kPanelH, img.rows);

    int px = img.cols - kW;
    cv::Mat roi = img(cv::Rect(px, 0, kW, panelH));
    cv::addWeighted(roi, 0.15, cv::Mat::zeros(roi.size(), roi.type()), 0.85, 0, roi);
    cv::line(img, {px, 0}, {px, panelH}, {70, 70, 70}, 1);

    int x  = px + kPad;
    int bw = kW - kPad * 2;
    int y  = kPad;

    // draw a label on the left and a value right-aligned within bw
    auto kv = [&](const std::string& label, const std::string& val,
                  cv::Scalar valCol = {200, 200, 200}) {
        constexpr float sc = 0.42f;
        int base = 0;
        int th = cv::getTextSize("A", cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base).height;
        y += th;
        // label (dim)
        cv::putText(img, label, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, {0,0,0}, 2, cv::LINE_AA);
        cv::putText(img, label, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, {100,100,100}, 1, cv::LINE_AA);
        // value — right-aligned
        int vw = cv::getTextSize(val, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base).width;
        int vx = px + kW - kPad - vw;
        cv::putText(img, val, {vx, y}, cv::FONT_HERSHEY_SIMPLEX, sc, {0,0,0}, 2, cv::LINE_AA);
        cv::putText(img, val, {vx, y}, cv::FONT_HERSHEY_SIMPLEX, sc, valCol,  1, cv::LINE_AA);
        y += base + 8;
    };

    auto divider = [&](int above = 6, int below = 6) {
        y += above;
        cv::line(img, {x, y}, {x+bw, y}, {65, 65, 65}, 1);
        y += below;
    };

    // title
    {
        constexpr float sc = 0.52f;
        int base = 0;
        int th = cv::getTextSize("A", cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base).height;
        y += th;
        cv::putText(img, "CAMERA", {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, {0,0,0},     2, cv::LINE_AA);
        cv::putText(img, "CAMERA", {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, {220,220,220}, 1, cv::LINE_AA);
        y += base + 4;
    }

    divider(4, 8);

    // data rows
    char buf[48];

    snprintf(buf, sizeof(buf), "%d × %d px", w, h);
    kv("Resolution", buf);

    snprintf(buf, sizeof(buf), "%.1f", fps);
    kv("FPS", buf);

    snprintf(buf, sizeof(buf), "%.1f ms", latMs);
    kv("Latency", buf);

    if (tempC >= 0.f) snprintf(buf, sizeof(buf), "%.1f °C", tempC);
    else              snprintf(buf, sizeof(buf), "n/a");
    kv("Temp", buf, tempColor(tempC));

    snprintf(buf, sizeof(buf), "%d", markerCount);
    kv("Markers", buf, markerCount > 0 ? cv::Scalar{50,220,70} : cv::Scalar{110,110,110});

    divider(6, 6);

    cv::putText(img, "r=reset  q=quit",
                {x, panelH - kPad},
                cv::FONT_HERSHEY_SIMPLEX, 0.34f, {75, 75, 75}, 1, cv::LINE_AA);
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::string configPath = "vision/aruco_tracker_config.json";
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

    cv::namedWindow("Camera Eval", cv::WINDOW_NORMAL);

    while (true) {
        if (tracker.update()) {
            cv::Mat frame = tracker.debugFrame();
            drawPanel(frame, tracker.fps(), tracker.latencyMs(),
                      tracker.cameraTemperature(),
                      sz.width, sz.height,
                      (int)tracker.robots().size());
            cv::imshow("Camera Eval", frame);
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 'r') tracker.requestStatsReset();
    }

    cv::destroyAllWindows();
    return 0;
}
