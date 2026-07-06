// swarm_dashboard.cpp
// ═══════════════════════════════════════════════════════════════
// Swarm Telemetry Dashboard — pure debugging/observability view
// ═══════════════════════════════════════════════════════════════
//
// Shows live graphs and statistics for every known robot: latency
// sparkline, battery peak-meter, motor L/R peak-meters, debug log.
// Not a demo — sends no motor commands, doesn't depend on vision.
//
// Voraussetzung:
//   ./swarm_hub /dev/tty.usbmodem* muss laufen
//
// Aufruf:
//   ./swarm_dashboard

#include "SwarmClient.h"
#include "telemetry_history.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
#include <algorithm>
#include <sys/ioctl.h>
#include <unistd.h>

static volatile bool running = true;
void signal_handler(int) { running = false; }

// ── Terminal sizing ──────────────────────────────────────────────

static int terminalWidth() {
    struct winsize w{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0 && w.ws_col > 0) return w.ws_col;
    return 80;
}

// ── Peak meter / sparkline rendering ─────────────────────────────

static const char* BLOCKS[] = { " ", "▁", "▂", "▃", "▄",
                                 "▅", "▆", "▇", "█" };

// Renders `value` in [0, max] as a colored horizontal peak-meter bar of
// `width` cells, green/yellow/red banded like an audio level meter.
static std::string peakMeter(float value, float maxVal, int width) {
    if (maxVal <= 0) maxVal = 1.0f;
    float frac = std::clamp(value / maxVal, 0.0f, 1.0f);
    int   filled = (int)(frac * width + 0.5f);

    std::string out;
    for (int i = 0; i < width; i++) {
        if (i >= filled) { out += "\033[90m░\033[0m"; continue; }
        float cellFrac = (float)(i + 1) / width;
        const char* color = cellFrac < 0.6f ? "\033[32m" : cellFrac < 0.85f ? "\033[33m" : "\033[31m";
        out += color;
        out += "█";
        out += "\033[0m";
    }
    return out;
}

// Bipolar peak meter centered at 0, range [-maxVal, +maxVal]. Used for motor
// power so forward/reverse are visually distinguishable. The center column is a
// fixed '|' divider; magnitude grows outward from it (right = forward/cyan,
// left = reverse/magenta).
static std::string bipolarMeter(float value, float maxVal, int width) {
    if (maxVal <= 0) maxVal = 1.0f;
    int   mid  = width / 2;
    float frac = std::clamp(value / maxVal, -1.0f, 1.0f);
    float af   = frac < 0 ? -frac : frac;
    // Round to nearest cell (not truncate), and guarantee any nonzero command
    // lights at least one cell — otherwise small values (e.g. ±15) floored to
    // 0 cells and the bar looked identical to a stopped motor.
    int mag = (int)(af * mid + 0.5f);
    if (value != 0 && mag == 0) mag = 1;

    std::string out;
    for (int i = 0; i < width; i++) {
        if (i == mid) { out += "\033[90m|\033[0m"; continue; }
        int  rel = i - mid;  // >0: right of divider (forward), <0: left (reverse)
        bool on  = (value >= 0) ? (rel >= 1 && rel <= mag)
                                : (rel <= -1 && rel >= -mag);
        if (!on) { out += "\033[90m░\033[0m"; continue; }
        out += (value >= 0) ? "\033[36m█\033[0m" : "\033[35m█\033[0m";
    }
    return out;
}

// Renders a metric history window as a block-character sparkline, scaled to
// [0, maxVal] over `width` columns (most recent sample on the right).
static std::string sparkline(const TelemetryHistory::Buffer& buf, float maxVal, int width) {
    if (maxVal <= 0) maxVal = 1.0f;
    int n = (int)buf.size();
    std::string out;
    int pad = std::max(0, width - n);
    for (int i = 0; i < pad; i++) out += ' ';
    int start = std::max(0, n - width);
    for (int i = start; i < n; i++) {
        float frac = std::clamp(buf.at((size_t)i) / maxVal, 0.0f, 1.0f);
        int   idx  = (int)(frac * 8.0f + 0.5f);
        out += BLOCKS[std::clamp(idx, 0, 8)];
    }
    return out;
}

// ── Layout ────────────────────────────────────────────────────────

struct RobotRowLayout {
    int sparkWidth;
    int meterWidth;
};

// Scales bar/sparkline widths down as the number of robots grows, so the
// dashboard stays within the terminal without horizontal overflow.
static RobotRowLayout computeLayout(int termWidth, int robotCount) {
    // Fixed columns: "ID│ lat(meter) val │ bat(meter) val │ L(meter) R(meter) │"
    // roughly: id(3) + 3*sep + labels(~30) + 4 bars
    int fixedOverhead = 3 + 30 + 8;
    int available = std::max(20, termWidth - fixedOverhead);
    int perBar = available / 4;

    int sparkWidth = std::clamp(perBar, 8, 30);
    int meterWidth = std::clamp(perBar, 8, 24);

    // With many robots on screen at once, shrink further so vertical density
    // doesn't push rows beyond the terminal height either.
    if (robotCount > 8) { sparkWidth = std::max(8, sparkWidth / 2); meterWidth = std::max(8, meterWidth / 2); }

    return { sparkWidth, meterWidth };
}

// ── Draw ──────────────────────────────────────────────────────────

static constexpr float LATENCY_MAX_US = 5000.0f;
static constexpr float BATTERY_MAX_V  = 6.5f;   // meter full-scale; fresh 4xAAA pack ≈ 6.4V
static constexpr float MOTOR_MAX      = 127.0f;

// Synchronized snapshot+redraw tick: all displayed values (and one history
// sample per robot) are taken at this cadence, from one coherent copy of the
// swarm state — not whenever a telemetry/pong frame happens to arrive.
static constexpr int UPDATE_INTERVAL_MS = 250;

// All robot states copied at a single instant; drawUI renders exclusively
// from this so a frame can't mix values that mutate mid-draw.
struct Snapshot {
    std::chrono::steady_clock::time_point at;
    std::vector<std::pair<uint8_t, SwarmClient::RobotState>> robots;
};

static void appendf(std::string& out, const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    out += buf;
}

static void drawUI(const Snapshot& snap, const TelemetryHistory& hist,
                   const std::vector<SwarmClient::DebugEntry>& log) {
    int termWidth = terminalWidth();

    // One frame = one atomic write(), wrapped in DEC synchronized-update
    // (\033[?2026h/l, ignored where unsupported). No leading full-screen
    // erase — GNOME Terminal/VTE repaints between stdout's line-buffered
    // flushes and kept catching the just-erased blank screen (flicker on
    // Ubuntu); instead every line ends with \033[K and the frame ends with
    // \033[J to clear leftovers from the previous, possibly taller frame.
    std::string f;
    f.reserve(16384);
    f += "\033[?2026h\033[H";

    appendf(f, "\033[1;36m%s\033[0m\033[K\n", std::string((size_t)termWidth, '=').c_str());
    f += "\033[1;37m  SWARM TELEMETRY DASHBOARD\033[0m\033[K\n";
    appendf(f, "\033[1;36m%s\033[0m\033[K\n\033[K\n", std::string((size_t)termWidth, '=').c_str());

    auto layout = computeLayout(termWidth, (int)snap.robots.size());

    if (snap.robots.empty()) {
        f += "\033[90m  Waiting for robots to announce...\033[0m\033[K\n";
    }

    for (const auto& [id, r] : snap.robots) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(snap.at - r.lastSeen).count();
        bool lost = elapsed > 5000;

        const auto& latBuf = hist.get(id, Metric::Latency);

        const char* idColor = lost ? "\033[31m" : "\033[1;33m";
        appendf(f, "%s R%-2d\033[0m ", idColor, id);

        appendf(f, "lat \033[36m%s\033[0m %5uus  ",
                sparkline(latBuf, LATENCY_MAX_US, layout.sparkWidth).c_str(), r.latencyUs);

        // Battery byte is 40mV/LSB; without STATUS_BAT_VALID the robot's ESP32 has
        // never received a metrics frame from the RP2040 — show "--", not 0.0V.
        if (r.flags & SC_STATUS_BAT_VALID) {
            appendf(f, "bat %s %4.2fV  ",
                    peakMeter(scBatteryVolts(r.battery), BATTERY_MAX_V, layout.meterWidth).c_str(),
                    scBatteryVolts(r.battery));
        } else {
            appendf(f, "bat %s %5s  ", peakMeter(0.0f, BATTERY_MAX_V, layout.meterWidth).c_str(), "--");
        }

        appendf(f, "L %s%+4d\033[0m  ", bipolarMeter((float)r.motorL, MOTOR_MAX, layout.meterWidth).c_str(), r.motorL);
        appendf(f, "R %s%+4d\033[0m\033[K\n", bipolarMeter((float)r.motorR, MOTOR_MAX, layout.meterWidth).c_str(), r.motorR);

        if (lost) {
            appendf(f, "     \033[31mLOST — last seen %.1fs ago\033[0m\033[K\n", elapsed / 1000.0);
        } else {
            appendf(f, "     \033[90mavg lat %.0fus  min %.0fus  max %.0fus  uptime %us\033[0m\033[K\n",
                    latBuf.avg(), latBuf.min(), latBuf.max(), r.uptime);
        }
        f += "\033[K\n";
    }

    appendf(f, "\033[90m%s\033[0m\033[K\n", std::string((size_t)termWidth, '-').c_str());
    appendf(f, "\033[37m  Known: %zu    via swarm_hub (/tmp/swarm_hub.sock)    Ctrl+C to exit\033[0m\033[K\n", snap.robots.size());

    if (!log.empty()) {
        f += "\033[K\n\033[1;37m  DEBUG LOG\033[0m\033[K\n";
        const int shown = 8;
        int start = (int)log.size() > shown ? (int)log.size() - shown : 0;
        for (int i = start; i < (int)log.size(); i++) {
            const auto& e = log[i];
            double age = std::chrono::duration_cast<std::chrono::milliseconds>(snap.at - e.at).count() / 1000.0;
            appendf(f, "\033[90m  -%5.1fs \033[36mR%-3d\033[0m %s\033[K\n", age, e.robotId, e.text.c_str());
        }
    }

    f += "\033[J\033[?2026l";
    (void)!::write(STDOUT_FILENO, f.data(), f.size());
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main() {
    std::cout << "Swarm Telemetry Dashboard\n";

    SwarmClient swarm;
    if (!swarm.connect()) return 1;
    std::cout << "Connected to swarm_hub (/tmp/swarm_hub.sock)\n\n";

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);  // restore the cursor on `kill`, too

    (void)!::write(STDOUT_FILENO, "\033[?25l", 6);  // hide cursor while drawing

    TelemetryHistory hist;
    auto lastTick = std::chrono::steady_clock::now();

    while (running) {
        swarm.poll();

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTick).count() >= UPDATE_INTERVAL_MS) {
            Snapshot snap;
            snap.at = now;
            for (uint8_t id : swarm.knownIds())
                snap.robots.emplace_back(id, swarm.robotState(id));
            for (const auto& [id, st] : snap.robots)
                hist.sample(id, st);
            drawUI(snap, hist, swarm.debugLog());
            lastTick = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "\033[2J\033[H\033[0m\033[?25h";
    return 0;
}
