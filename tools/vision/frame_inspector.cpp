// frame_inspector.cpp — Record N seconds of raw camera frames, then step
// through them one at a time to analyse detection quality.
//
// Frames are held only in memory (a std::vector<cv::Mat>) — nothing is ever
// written to disk, so they vanish the moment the program exits.
//
// Usage: ./frame_inspector [--config JSON] [--serial SN] [--ip IP]
//                           [--seconds N] [--mirror]
//
// Keys (review mode):
//   Right / Left   = step forward / backward one frame
//   Home / End     = jump to first / last frame
//   d              = run the ArUco detector once over every recorded frame,
//                    then toggle the marker overlay on/off
//   q / Esc        = quit

#include "aruco_tracker.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <chrono>

// Arrow/Home/End codes returned by cv::waitKeyEx() — values differ by HighGUI
// backend (Cocoa on macOS vs. GTK/Qt on Linux), so both are checked.
namespace key {
    constexpr int kLeftMac = 63234, kRightMac = 63235, kHomeMac = 63273, kEndMac = 63275;
    constexpr int kLeftLinux = 65361, kRightLinux = 65363, kHomeLinux = 65360, kEndLinux = 65367;
}
static bool isLeft(int k)  { return k == key::kLeftMac  || k == key::kLeftLinux; }
static bool isRight(int k) { return k == key::kRightMac || k == key::kRightLinux; }
static bool isHome(int k)  { return k == key::kHomeMac  || k == key::kHomeLinux; }
static bool isEnd(int k)   { return k == key::kEndMac   || k == key::kEndLinux; }

struct FrameDetections {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
};

static void drawDetections(cv::Mat& img, const FrameDetections& d) {
    for (size_t i = 0; i < d.ids.size(); ++i) {
        const auto& c = d.corners[i];
        for (int k = 0; k < 4; ++k)
            cv::line(img, c[k], c[(k + 1) % 4], {0, 255, 0}, 2, cv::LINE_AA);
        cv::Point2f center = (c[0] + c[1] + c[2] + c[3]) * 0.25f;
        cv::circle(img, center, 4, {0, 0, 255}, -1);
        cv::putText(img, std::to_string(d.ids[i]), {(int)c[0].x, (int)c[0].y - 8},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 0}, 2, cv::LINE_AA);
    }
}

int main(int argc, char* argv[]) {
    setvbuf(stdout, nullptr, _IOLBF, 0);  // flush progress lines even when piped/redirected

    std::string configPath = ArucoConfig::defaultConfigPath();
    std::string serial, ip;
    bool   cfg_mirror = false;
    double seconds    = 1.0;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--config")  && i + 1 < argc) configPath = argv[++i];
        else if (!strcmp(argv[i], "--serial")  && i + 1 < argc) serial     = argv[++i];
        else if (!strcmp(argv[i], "--ip")      && i + 1 < argc) ip         = argv[++i];
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds    = atof(argv[++i]);
        else if (!strcmp(argv[i], "--mirror"))                   cfg_mirror = true;
        else fprintf(stderr, "[inspector] unknown arg '%s'\n", argv[i]);
    }
    if (seconds <= 0.0) {
        fprintf(stderr, "[inspector] --seconds must be > 0\n");
        return 1;
    }

    ArucoConfig cfg = ArucoConfig::fromFile(configPath);
    if (!serial.empty()) cfg.baslerSerial = serial;
    if (!ip.empty())     cfg.baslerIp     = ip;
    if (cfg_mirror)      cfg.mirrorInput  = true;

    // Talk to the camera directly — recording wants every frame the camera
    // can deliver at its currently configured fps, with no detection thread
    // competing for CPU the way the live ArucoTracker's pipeline would.
    BaslerPylonSource source;
    if (!source.open(cfg)) {
        fprintf(stderr, "[inspector] failed to open camera\n");
        return 1;
    }
    cv::Size sz = source.size();

    // ── Record ───────────────────────────────────────────────────────────────
    printf("[inspector] recording %.2fs at the camera's configured fps (cam_fps=%d)...\n",
           seconds, cfg.fps);

    std::vector<cv::Mat> frames;
    auto t0 = std::chrono::steady_clock::now();
    cv::Mat frame;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        if (source.read(frame)) {
            if (cfg.mirrorInput) cv::flip(frame, frame, 1);
            frames.push_back(std::move(frame));
        }
    }
    double elapsed     = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    double achievedFps = frames.empty() ? 0.0 : frames.size() / elapsed;
    printf("[inspector] captured %zu frames in %.2fs (~%.1f fps)\n",
           frames.size(), elapsed, achievedFps);

    if (frames.empty()) {
        fprintf(stderr, "[inspector] no frames captured — aborting\n");
        return 1;
    }

    // ── Review ───────────────────────────────────────────────────────────────
    // Same dictionary/parameters the live tracker uses, but run synchronously
    // and statelessly — each frame is detected on its own, independent of its
    // neighbours, so the overlay shows exactly what a single detector pass
    // finds on that frame (no Kalman smoothing, no carried-over ROI).
    cv::aruco::ArucoDetector detector = buildArucoDetector(cfg);
    cv::Ptr<cv::CLAHE> clahe = cfg.claheClip > 0
        ? cv::createCLAHE(cfg.claheClip, {cfg.claheTile, cfg.claheTile})
        : nullptr;

    std::vector<FrameDetections> detections;  // filled lazily on first 'd' press
    bool haveDetections = false;
    bool showOverlay    = false;

    cv::namedWindow("Frame Inspector", cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow("Frame Inspector", sz.width, sz.height);

    int idx = 0;
    printf("[inspector] %zu frames ready — Left/Right=step  Home/End=jump  "
           "d=detect+toggle overlay  q/Esc=quit\n", frames.size());

    while (true) {
        cv::Mat disp = frames[idx].clone();
        if (showOverlay && haveDetections) drawDetections(disp, detections[idx]);

        char hud[96];
        snprintf(hud, sizeof(hud), "frame %d/%zu", idx + 1, frames.size());
        cv::putText(disp, hud, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2, cv::LINE_AA);
        if (haveDetections) {
            snprintf(hud, sizeof(hud), "tags: %zu%s", detections[idx].ids.size(),
                      showOverlay ? "" : "  (overlay off, press d)");
            cv::putText(disp, hud, {10, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2, cv::LINE_AA);
        }

        cv::imshow("Frame Inspector", disp);
        int k = cv::waitKeyEx(0);

        if (k == 'q' || k == 27) {
            break;
        } else if (isRight(k)) {
            idx = std::min(idx + 1, (int)frames.size() - 1);
        } else if (isLeft(k)) {
            idx = std::max(idx - 1, 0);
        } else if (isHome(k)) {
            idx = 0;
        } else if (isEnd(k)) {
            idx = (int)frames.size() - 1;
        } else if (k == 'd') {
            if (!haveDetections) {
                printf("[inspector] running detector over %zu frames...\n", frames.size());
                detections.resize(frames.size());
                for (size_t i = 0; i < frames.size(); ++i) {
                    cv::Mat gray;
                    cv::cvtColor(frames[i], gray, cv::COLOR_BGR2GRAY);
                    if (clahe) clahe->apply(gray, gray);
                    std::vector<std::vector<cv::Point2f>> rejected;
                    detector.detectMarkers(gray, detections[i].corners, detections[i].ids, rejected);
                }
                haveDetections = true;
                printf("[inspector] done\n");
            }
            showOverlay = !showOverlay;
        }
    }

    cv::destroyAllWindows();
    return 0;  // frames/detections go out of scope here — dropped, never persisted
}
