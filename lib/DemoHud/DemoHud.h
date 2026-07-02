// DemoHud.h — header-only on-screen debug overlay for the PC vision *demo*
// tools (vision_controller, circle_demo, wingman, shape_demo, drag_drop_demo,
// demo_hud_preview). Not a general-purpose UI/HUD library — it exists purely
// so these interactive demos can show loop_fps, cam_fps, latency, battery,
// hub status, and per-robot tables without each one rolling its own ad-hoc
// cv::putText/snprintf HUD code. Production/headless tools (swarm_hub,
// swarm_terminal, latency_plot, swarm_controller) have no use for this.
//
// Usage:
//   DemoHud hud;
//   hud.title(fmt("loop_fps:%.0f  Robots:%d  HUB:%s", fps, n, ok ? "OK":"--"));
//   hud.header({"ID", "Vision", "Battery", "Latency", "Mot-L", "Mot-R", "Status"});
//   for (auto& r : robots) hud.row({id, vis, battery, latency, l, r, st}, color);
//   hud.drawTopRight(frame);   // canonical: docked to the top-right corner
//   hud.clear();
//
// Every demo follows this same shape — a summary title line + the uniform
// per-robot table, docked top-right via drawTopRight — so the HUDs all look
// and sit the same across vision_controller, circle_demo, wingman, shape_demo
// and drag_drop_demo. Use draw(img, origin) directly only if you need a
// non-corner placement.
//
// LoopFps is a small helper for the "frameCount/lastFpsT, recomputed once a
// second" pattern that was duplicated identically in every tool's main loop.

#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

class DemoHud {
public:
    inline static const cv::Scalar COL_TEXT = {230, 230, 230};
    inline static const cv::Scalar COL_OK   = {100, 220, 100};
    inline static const cv::Scalar COL_WARN = {60, 200, 230};
    inline static const cv::Scalar COL_BAD  = {60, 60, 220};

    static constexpr int    FONT       = cv::FONT_HERSHEY_SIMPLEX;
    // Sizing knobs tuned for a corner overlay: every demo now docks this panel
    // to the top-right of the live video (see drawTopRight), so the panel has
    // to be readable without swallowing the frame. These are ~1.4x the original
    // 0.5/1/20/16/8 values — big enough to read across a room, small enough
    // that the per-robot table still leaves most of the video visible.
    static constexpr double FONT_SCALE = 0.7;
    static constexpr int    THICKNESS  = 2;
    static constexpr int    ROW_H      = 34;
    static constexpr int    COL_GAP    = 28;
    static constexpr int    PAD        = 14;

    static std::string fmt(const char* f, ...) {
        char buf[128];
        va_list ap;
        va_start(ap, f);
        vsnprintf(buf, sizeof(buf), f, ap);
        va_end(ap);
        return buf;
    }

    // Single-value row, e.g. hud.row("loop_fps", "60");
    void row(const std::string& label, const std::string& value,
             cv::Scalar color = COL_TEXT) {
        rows_.push_back({{label + ":", value}, color});
    }

    // Multi-column row for tabular panels (per-robot lists, ...).
    void row(const std::vector<std::string>& cells, cv::Scalar color = COL_TEXT) {
        rows_.push_back({cells, color});
    }

    void header(const std::vector<std::string>& cells) {
        header_ = cells;
    }

    // Optional full-width title line drawn above header/rows (e.g. a
    // "loop_fps:60 Robots:3/4 HUB:OK" summary), not counted into column
    // width measurement.
    void title(const std::string& text, cv::Scalar color = COL_TEXT) {
        title_ = text;
        titleColor_ = color;
    }

    void clear() {
        rows_.clear();
        header_.clear();
        title_.clear();
    }

    // Overall pixel size of the panel for the current rows/header/title.
    // Lets callers right-/bottom-anchor the panel (draw() auto-sizes, so
    // without this they couldn't know where the right edge would land).
    cv::Size measure(int minWidth = 0) const {
        std::vector<int> colWidth;
        return layout(colWidth, minWidth);
    }

    // Draws the accumulated rows as one panel anchored at `origin`
    // (top-left corner), with a semi-transparent background.
    void draw(cv::Mat& img, cv::Point origin, int minWidth = 0) const {
        if (img.empty() || (rows_.empty() && header_.empty() && title_.empty())) return;

        std::vector<int> colWidth;
        cv::Size size = layout(colWidth, minWidth);

        cv::Rect panel(origin.x, origin.y, size.width, size.height);
        panel &= cv::Rect(0, 0, img.cols, img.rows);
        if (panel.width <= 0 || panel.height <= 0) return;

        cv::Mat roi = img(panel);
        cv::Mat overlay;
        roi.copyTo(overlay);
        overlay.setTo(cv::Scalar(20, 20, 20));
        cv::addWeighted(overlay, 0.55, roi, 0.45, 0, roi);

        int y = origin.y + PAD + ROW_H - 10;  // text baseline within the first row
        if (!title_.empty()) {
            cv::putText(img, title_, {origin.x + PAD, y}, FONT, FONT_SCALE, titleColor_, THICKNESS, cv::LINE_AA);
            y += ROW_H;
        }
        if (!header_.empty()) {
            drawRow(img, origin.x + PAD, y, header_, COL_WARN, colWidth);
            y += ROW_H;
        }
        for (auto& r : rows_) {
            drawRow(img, origin.x + PAD, y, r.cells, r.color, colWidth);
            y += ROW_H;
        }
    }

    // Docks the panel to the top-right corner of `img`, `margin` px in from the
    // top and right edges. This is the canonical placement for every demo HUD
    // so they all sit in the same spot — callers just build rows and call this
    // instead of hand-computing an origin from measure().
    void drawTopRight(cv::Mat& img, int margin = 16, int minWidth = 0) const {
        if (img.empty()) return;
        cv::Size size = measure(minWidth);
        draw(img, {img.cols - size.width - margin, margin}, minWidth);
    }

    // Helper for the loop_fps pattern duplicated across every tool's main
    // loop: call tick() once per iteration, read fps() whenever needed.
    class LoopFps {
    public:
        void tick() {
            ++count_;
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_).count();
            if (elapsed >= 1.0) {
                fps_ = count_ / elapsed;
                count_ = 0;
                last_ = now;
            }
        }
        float fps() const { return fps_; }

    private:
        int count_ = 0;
        float fps_ = 0.f;
        std::chrono::steady_clock::time_point last_ = std::chrono::steady_clock::now();
    };

    // Battery telemetry byte is 40mV/LSB (0-255 -> 0-10.2V). 0 means the robot
    // has not reported a measurement yet, not an empty battery.
    static std::string formatBattery(uint8_t raw) {
        return raw > 0 ? fmt("%.2fV", raw * 0.04f) : "--";
    }

    static std::string formatLatency(uint16_t latencyUs) {
        return latencyUs > 0 ? fmt("%.1fms", latencyUs / 1000.f) : "--";
    }

private:
    struct Row {
        std::vector<std::string> cells;
        cv::Scalar color;
    };

    // Measures per-column widths and the overall panel size. Shared by
    // measure() and draw() so both agree on geometry.
    cv::Size layout(std::vector<int>& colWidth, int minWidth) const {
        size_t numCols = header_.size();
        for (auto& r : rows_) numCols = std::max(numCols, r.cells.size());

        colWidth.assign(numCols, 0);
        auto measureRow = [&](const std::vector<std::string>& cells) {
            for (size_t c = 0; c < cells.size(); ++c) {
                int w = cv::getTextSize(cells[c], FONT, FONT_SCALE, THICKNESS, nullptr).width;
                colWidth[c] = std::max(colWidth[c], w);
            }
        };
        if (!header_.empty()) measureRow(header_);
        for (auto& r : rows_) measureRow(r.cells);

        int totalW = 2 * PAD;
        for (int w : colWidth) totalW += w + COL_GAP;
        if (!title_.empty()) {
            int titleW = cv::getTextSize(title_, FONT, FONT_SCALE, THICKNESS, nullptr).width + 2 * PAD;
            totalW = std::max(totalW, titleW);
        }
        totalW = std::max(totalW, minWidth);

        int numRows = (int)rows_.size() + (header_.empty() ? 0 : 1) + (title_.empty() ? 0 : 1);
        int totalH = 2 * PAD + numRows * ROW_H;
        return {totalW, totalH};
    }

    static void drawRow(cv::Mat& img, int x, int y, const std::vector<std::string>& cells,
                         cv::Scalar color, const std::vector<int>& colWidth) {
        int cx = x;
        for (size_t c = 0; c < cells.size(); ++c) {
            cv::putText(img, cells[c], {cx, y}, FONT, FONT_SCALE, color, THICKNESS, cv::LINE_AA);
            cx += colWidth[c] + COL_GAP;
        }
    }

    std::vector<Row> rows_;
    std::vector<std::string> header_;
    std::string title_;
    cv::Scalar titleColor_ = COL_TEXT;
};
