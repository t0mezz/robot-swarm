// DebugHud.h — header-only shared debug overlay for PC vision tools.
//
// One consistent panel style (stacked rows, optional columns) for everything
// every tool currently rolls its own HUD for: loop_fps, cam_fps, latency,
// battery, hub status, per-robot tables, etc. Replaces the ad-hoc
// cv::putText/snprintf HUD code duplicated across vision_controller.cpp,
// wingman.cpp, circle_demo.cpp, shape_demo.cpp, drag_drop_demo.cpp.
//
// Usage:
//   DebugHud hud;
//   hud.row("loop_fps", fmt("%.0f", fps));
//   hud.row("Robots", fmt("%d", swarm.knownCount()), DebugHud::COL_OK);
//   hud.draw(frame, {10, 10});
//   hud.clear();
//
//   // tabular (per-robot) panel — same visual style, multiple columns/row
//   DebugHud table;
//   table.header({"ID", "Battery", "Latency"});
//   for (auto& r : robots) table.row({id, battery, latency}, colorForRow);
//   table.draw(frame, {10, 10});
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

class DebugHud {
public:
    inline static const cv::Scalar COL_TEXT = {230, 230, 230};
    inline static const cv::Scalar COL_OK   = {100, 220, 100};
    inline static const cv::Scalar COL_WARN = {60, 200, 230};
    inline static const cv::Scalar COL_BAD  = {60, 60, 220};

    static constexpr int    FONT       = cv::FONT_HERSHEY_SIMPLEX;
    static constexpr double FONT_SCALE = 0.5;
    static constexpr int    THICKNESS  = 1;
    static constexpr int    ROW_H      = 20;
    static constexpr int    COL_GAP    = 16;
    static constexpr int    PAD        = 8;

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

    // Draws the accumulated rows as one panel anchored at `origin`
    // (top-left corner), with a semi-transparent background.
    void draw(cv::Mat& img, cv::Point origin, int minWidth = 0) const {
        if (img.empty() || (rows_.empty() && header_.empty() && title_.empty())) return;

        size_t numCols = header_.size();
        for (auto& r : rows_) numCols = std::max(numCols, r.cells.size());

        std::vector<int> colWidth(numCols, 0);
        auto measure = [&](const std::vector<std::string>& cells) {
            for (size_t c = 0; c < cells.size(); ++c) {
                int w = cv::getTextSize(cells[c], FONT, FONT_SCALE, THICKNESS, nullptr).width;
                colWidth[c] = std::max(colWidth[c], w);
            }
        };
        if (!header_.empty()) measure(header_);
        for (auto& r : rows_) measure(r.cells);

        int totalW = 2 * PAD;
        for (int w : colWidth) totalW += w + COL_GAP;
        if (!title_.empty()) {
            int titleW = cv::getTextSize(title_, FONT, FONT_SCALE, THICKNESS, nullptr).width + 2 * PAD;
            totalW = std::max(totalW, titleW);
        }
        totalW = std::max(totalW, minWidth);

        int numRows = (int)rows_.size() + (header_.empty() ? 0 : 1) + (title_.empty() ? 0 : 1);
        int totalH = 2 * PAD + numRows * ROW_H;

        cv::Rect panel(origin.x, origin.y, totalW, totalH);
        panel &= cv::Rect(0, 0, img.cols, img.rows);
        if (panel.width <= 0 || panel.height <= 0) return;

        cv::Mat roi = img(panel);
        cv::Mat overlay;
        roi.copyTo(overlay);
        overlay.setTo(cv::Scalar(20, 20, 20));
        cv::addWeighted(overlay, 0.55, roi, 0.45, 0, roi);

        int y = origin.y + PAD + ROW_H - 6;
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

    static std::string formatBattery(uint8_t raw) {
        return fmt("%d%%", (int)(raw * 100 / 255));
    }

    static std::string formatLatency(uint16_t latencyUs) {
        return latencyUs > 0 ? fmt("%.1fms", latencyUs / 1000.f) : "--";
    }

private:
    struct Row {
        std::vector<std::string> cells;
        cv::Scalar color;
    };

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
