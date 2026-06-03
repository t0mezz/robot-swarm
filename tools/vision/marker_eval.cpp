// marker_eval.cpp — Live ArUco marker read accuracy evaluator
// Usage: ./marker_eval [--config JSON] [--cam N] [--expected 0,1,2] [--mirror]
//
// Opens the camera, auto-discovers markers as they appear, and starts an
// accuracy rating the moment the first marker is seen.
// When --expected is set, only those IDs are tracked; all others are ignored.
// Keys: r = reset eval,  q / Esc = quit

#include "aruco_tracker.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <chrono>
#include <algorithm>

using Clock = std::chrono::steady_clock;

// ── per-marker tracking ───────────────────────────────────────────────────────

struct MarkerStats {
    int evalFrames    = 0;
    int detectedFrames = 0;
    float rate() const {
        return evalFrames > 0 ? (float)detectedFrames / evalFrames : 0.0f;
    }
};

// ── overlay panel ─────────────────────────────────────────────────────────────

static constexpr int kPanelW = 250;
static constexpr int kPad    = 12;
static constexpr int kBarH   = 6;

static cv::Scalar ratingColor(float rate) {
    if (rate >= 0.90f) return {50, 220,  70};  // green
    if (rate >= 0.70f) return {30, 210, 255};  // yellow
    if (rate >= 0.50f) return {30, 130, 255};  // orange
    return                     {40,  60, 230}; // red
}

static void drawPanel(cv::Mat& img,
                      const std::unordered_map<int, MarkerStats>& stats,
                      const std::set<int>& expected,
                      int evalFrames, double evalSecs, bool started)
{
    // ── measure content height ────────────────────────────────────────────────
    // rows: title(1) + divider + timer(1) + divider + N*(label+bar+pct) + divider + overall(1+bar)
    int nRows = (int)stats.size();
    int contentH = 30          // title
                 + 18          // divider
                 + 20          // timer
                 + 14          // divider
                 + nRows * 54  // per-marker: label 22 + bar 14 + pct 18
                 + (nRows > 0 ? 18 + 26 + 14 + 26 : 0)  // divider + overall label + bar + pct
                 + 20;         // hint at bottom
    int panelH = std::max(contentH, 80) + kPad * 2;
    panelH = std::min(panelH, img.rows);

    // ── dark overlay on right edge ────────────────────────────────────────────
    int px = img.cols - kPanelW;
    cv::Mat roi = img(cv::Rect(px, 0, kPanelW, panelH));
    cv::Mat dark = cv::Mat::zeros(roi.size(), roi.type());
    cv::addWeighted(roi, 0.18, dark, 0.82, 0, roi);

    // thin left border
    cv::line(img, {px, 0}, {px, panelH}, {80, 80, 80}, 1);

    // ── drawing helpers ───────────────────────────────────────────────────────
    int x  = px + kPad;
    int bw = kPanelW - kPad * 2;  // bar width
    int y  = kPad;

    // text: draws with black outline for legibility over any background
    auto text = [&](const std::string& s, float sc, cv::Scalar col, int extraBelow = 0) {
        int baseline = 0;
        cv::Size sz = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &baseline);
        y += sz.height;
        cv::putText(img, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, {0,0,0}, 3, cv::LINE_AA);
        cv::putText(img, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, col,   1, cv::LINE_AA);
        y += baseline + extraBelow;
    };

    auto divider = [&](int above = 8, int below = 8) {
        y += above;
        cv::line(img, {x, y}, {x+bw, y}, {75, 75, 75}, 1);
        y += below;
    };

    auto bar = [&](float rate, cv::Scalar col) {
        y += 4;
        cv::rectangle(img, {x, y}, {x+bw, y+kBarH}, {50,50,50}, -1);
        int fill = (int)(bw * std::clamp(rate, 0.0f, 1.0f));
        if (fill > 0) cv::rectangle(img, {x, y}, {x+fill, y+kBarH}, col, -1);
        y += kBarH + 4;
    };

    // ── content ───────────────────────────────────────────────────────────────
    text("MARKER EVAL", 0.55f, {240, 240, 240}, 2);
    divider(4, 6);

    if (!started) {
        text("Waiting for first marker", 0.40f, {120, 120, 120}, 4);
        divider(6, 6);
        text("r=reset  q=quit", 0.36f, {75, 75, 75});
        return;
    }

    char tbuf[48];
    snprintf(tbuf, sizeof(tbuf), "%.1fs   %d frames", evalSecs, evalFrames);
    text(tbuf, 0.40f, {130, 130, 130}, 2);
    divider();

    std::vector<int> ids;
    for (auto& [id, _] : stats) ids.push_back(id);
    std::sort(ids.begin(), ids.end());

    for (int id : ids) {
        auto& st   = stats.at(id);
        float rate = st.rate();
        cv::Scalar col = ratingColor(rate);

        char label[32], pct[32];
        snprintf(label, sizeof(label), "ID %d", id);
        snprintf(pct,   sizeof(pct),   "%.0f%%  (%d/%d fr)",
                 rate * 100, st.detectedFrames, st.evalFrames);

        text(label, 0.46f, col, 0);
        bar(rate, col);
        text(pct, 0.38f, col, 6);
    }

    if (!ids.empty()) {
        float sum = 0;
        for (auto& [_, st] : stats) sum += st.rate();
        float overall = sum / (float)stats.size();
        cv::Scalar col = ratingColor(overall);

        divider();
        text("Overall", 0.50f, col, 0);
        bar(overall, col);
        char obuf[16];
        snprintf(obuf, sizeof(obuf), "%.0f%%", overall * 100);
        text(obuf, 0.50f, col, 4);
    }

    // hint pinned to bottom of panel
    cv::putText(img, "r=reset  q=quit",
                {x, panelH - 6},
                cv::FONT_HERSHEY_SIMPLEX, 0.36f, {75, 75, 75}, 1, cv::LINE_AA);
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::string configPath = "aruco_tracker_config.json";
    std::set<int> expectedIds;
    int  camOverride = -1;
    bool cfg_mirror  = false;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--config")   && i+1<argc) configPath  = argv[++i];
        else if (!strcmp(argv[i], "--cam")      && i+1<argc) camOverride = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--mirror"))              cfg_mirror = true;
        else if (!strcmp(argv[i], "--expected") && i+1<argc) {
            char buf[512]; strncpy(buf, argv[++i], 511); buf[511] = 0;
            char* tok = strtok(buf, ",");
            while (tok) { expectedIds.insert(atoi(tok)); tok = strtok(nullptr, ","); }
        } else {
            fprintf(stderr, "[eval] unknown arg '%s'\n", argv[i]);
        }
    }

    ArucoConfig cfg = ArucoConfig::fromFile(configPath);
    if (camOverride >= 0) cfg.camIndex = camOverride;
    if (cfg_mirror) cfg.mirrorInput = true;
    cfg.debugOverlay = true;

    ArucoTracker tracker(cfg);
    if (!tracker.open()) {
        fprintf(stderr, "[eval] failed to open camera %d\n", cfg.camIndex);
        return 1;
    }
    printf("[eval] camera %d  (%dx%d @ %d fps)\n",
           cfg.camIndex, cfg.width, cfg.height, cfg.fps);
    if (!expectedIds.empty()) {
        printf("[eval] tracking only IDs: ");
        for (int id : expectedIds) printf("%d ", id);
        printf("\n");
    }
    printf("[eval] r=reset  q=quit\n");

    cv::namedWindow("Marker Eval", cv::WINDOW_NORMAL);

    std::unordered_map<int, MarkerStats> stats;
    bool  started    = false;
    int   evalFrames = 0;
    Clock::time_point evalStart;
    bool hasExpected = !expectedIds.empty();

    auto reset = [&]() {
        stats.clear();
        started    = false;
        evalFrames = 0;
        printf("[eval] reset\n");
    };

    while (true) {
        if (!tracker.update()) { cv::waitKey(1); continue; }

        const auto& robots = tracker.robots();

        // filter to expected IDs when set
        std::set<int> detectedNow;
        for (auto& r : robots) {
            if (!hasExpected || expectedIds.count(r.id))
                detectedNow.insert(r.id);
        }

        // start eval on first relevant detection
        if (!started && !detectedNow.empty()) {
            started   = true;
            evalStart = Clock::now();
            printf("[eval] first marker seen — eval started\n");
        }

        if (started) {
            evalFrames++;
            double evalSecs = std::chrono::duration<double>(Clock::now()-evalStart).count();

            for (auto& [id, st] : stats) st.evalFrames++;
            for (int id : detectedNow) {
                if (!stats.count(id)) {
                    stats[id] = {};
                    printf("[eval] discovered ID %d\n", id);
                } else {
                    stats[id].detectedFrames++;
                }
            }

            cv::Mat frame = tracker.debugFrame().clone();
            drawPanel(frame, stats, expectedIds, evalFrames, evalSecs, true);
            cv::imshow("Marker Eval", frame);
        } else {
            cv::Mat frame = tracker.debugFrame().clone();
            drawPanel(frame, stats, expectedIds, 0, 0.0, false);
            cv::imshow("Marker Eval", frame);
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        if (key == 'r') reset();
    }

    printf("\n════ FINAL SUMMARY ════\n");
    if (!started) {
        printf("No markers detected.\n");
    } else {
        printf("Eval frames: %d\n", evalFrames);
        std::vector<int> ids;
        for (auto& [id, _] : stats) ids.push_back(id);
        std::sort(ids.begin(), ids.end());
        float sum = 0;
        for (int id : ids) {
            auto& st = stats[id];
            printf("  ID %-2d  %.1f%%  (%d / %d frames)\n",
                   id, st.rate()*100, st.detectedFrames, st.evalFrames);
            sum += st.rate();
        }
        if (!ids.empty())
            printf("Overall: %.1f%%\n", sum / ids.size() * 100);
    }

    cv::destroyAllWindows();
    return 0;
}
