// latency_plot.cpp
// ═══════════════════════════════════════════════════════════════
// Latency Plotter — Live ASCII-Plot der ESP-NOW Latenz
// ═══════════════════════════════════════════════════════════════
//
// Verbindet sich mit swarm_hub und zeichnet die Roundtrip-Latenz
// eines bestimmten Roboters als rollenden ASCII-Balkendiagramm.
//
// Voraussetzung:
//   ./swarm_hub /dev/tty.usbserial-0001 muss laufen
//
// Aufruf:
//   ./latency_plot <robot_id>

#include "SwarmClient.h"

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>
#include <deque>

// ═══════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════

static volatile bool running = true;
void signal_handler(int) { running = false; }

// Hub pings at 200ms round-robin; window = PLOT_WIDTH samples
static constexpr int  PLOT_WIDTH  = 60;
static constexpr int  PLOT_HEIGHT = 12;
static constexpr int  PING_MS     = 200;   // hub ping interval (for window annotation)

static std::deque<uint16_t> samples;

static uint16_t latestUs    =     0;
static uint16_t minUs       = 65535;
static uint16_t maxUs       =     0;
static uint64_t sumUs       =     0;
static uint32_t sampleCount =     0;

// Throttle graph samples to ~1 per second (hub pings at 200ms round-robin,
// so each robot gets a pong every PING_MS * MAX_ROBOTS ≈ 200ms, but the
// graph should scroll at a human-readable rate)
static constexpr int  SAMPLE_INTERVAL_MS = 1000;
static std::chrono::steady_clock::time_point lastSampleAt;

static void onPong(uint16_t us) {
    latestUs = us;
    if (us < minUs) minUs = us;
    if (us > maxUs) maxUs = us;
    sumUs += us;
    sampleCount++;

    // Throttle graph points so the chart scrolls at ~1 sample/second
    auto now = std::chrono::steady_clock::now();
    auto msSinceLast = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastSampleAt).count();
    if (sampleCount == 1 || msSinceLast >= SAMPLE_INTERVAL_MS) {
        samples.push_back(us);
        if ((int)samples.size() > PLOT_WIDTH) samples.pop_front();
        lastSampleAt = now;
    }
}

// ═══════════════════════════════════════════════════════════════
// Plot
// ═══════════════════════════════════════════════════════════════

static const char* BLOCKS[] = { " ", "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█" };

static void drawPlot(uint8_t robotId) {
    std::cout << "\033[2J\033[H";

    std::cout << "\033[1;37mLatency Plot\033[0m";
    std::cout << "  Robot \033[1;33m" << (int)robotId << "\033[0m";
    std::cout << "  \033[90mvia swarm_hub\033[0m\n";

    if (sampleCount == 0) {
        std::cout << "\n  \033[90mWaiting for pong from robot " << (int)robotId << "...\033[0m\n";
        std::cout.flush();
        return;
    }

    uint16_t avgUs = (uint16_t)(sumUs / sampleCount);

    std::cout << "  Now: \033[1;32m" << latestUs << " µs\033[0m"
              << "   Min: \033[32m"  << minUs    << " µs\033[0m"
              << "   Max: \033[31m"  << maxUs    << " µs\033[0m"
              << "   Avg: \033[36m"  << avgUs    << " µs\033[0m"
              << "   n=" << sampleCount << '\n';

    uint16_t lo = minUs;
    uint16_t hi = maxUs;
    if (hi == lo) hi = lo + 1;
    hi = (uint16_t)(hi * 1.10f);

    int N       = (int)samples.size();
    int padCols = PLOT_WIDTH - N;

    char rowLabel[64];
    for (int row = PLOT_HEIGHT - 1; row >= 0; row--) {
        float rowHi = lo + (float)(row + 1) * (hi - lo) / PLOT_HEIGHT;
        if (row == PLOT_HEIGHT - 1 || row == 0 || row == PLOT_HEIGHT / 2) {
            snprintf(rowLabel, sizeof(rowLabel), "\033[90m%5.0f us\033[0m |", rowHi);
        } else {
            snprintf(rowLabel, sizeof(rowLabel), "          |");
        }
        std::cout << rowLabel;

        for (int c = 0; c < padCols; c++) std::cout << ' ';

        for (int c = 0; c < N; c++) {
            float v     = (float)samples[c];
            float rowLo = lo + (float)row * (hi - lo) / PLOT_HEIGHT;
            if (v <= rowLo) {
                std::cout << ' ';
            } else if (v >= rowHi) {
                uint16_t s = samples[c];
                if      (s <= avgUs)          std::cout << "\033[32m█\033[0m";
                else if (s <= avgUs * 3 / 2)  std::cout << "\033[33m█\033[0m";
                else                          std::cout << "\033[31m█\033[0m";
            } else {
                float fill   = (v - rowLo) / (rowHi - rowLo);
                int   eighth = (int)(fill * 8.0f + 0.5f);
                if (eighth < 1) eighth = 1;
                if (eighth > 8) eighth = 8;
                uint16_t s = samples[c];
                if      (s <= avgUs)          std::cout << "\033[32m" << BLOCKS[eighth] << "\033[0m";
                else if (s <= avgUs * 3 / 2)  std::cout << "\033[33m" << BLOCKS[eighth] << "\033[0m";
                else                          std::cout << "\033[31m" << BLOCKS[eighth] << "\033[0m";
            }
        }
        std::cout << '\n';
    }

    std::cout << "         └";
    for (int c = 0; c < PLOT_WIDTH; c++) std::cout << '-';
    std::cout << '\n';
    std::cout << "\033[90m           ← " << PLOT_WIDTH << " samples ("
              << (PLOT_WIDTH * PING_MS / 1000) << "s window, hub ping=" << PING_MS << "ms)\033[0m\n";
    std::cout.flush();
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <robot_id>\n";
        std::cerr << "  e.g. " << argv[0] << " 0\n";
        std::cerr << "Make sure swarm_hub is running: ./swarm_hub /dev/tty.usbserial-0001\n";
        return 1;
    }

    int targetArg = std::stoi(argv[1]);
    if (targetArg < 0 || targetArg >= SC_MAX_ROBOTS) {
        std::cerr << "Error: robot_id must be in [0, " << (SC_MAX_ROBOTS - 1) << "]\n";
        return 1;
    }
    uint8_t target = (uint8_t)targetArg;

    SwarmClient swarm;
    if (!swarm.connect()) return 1;

    signal(SIGINT, signal_handler);

    using Clock = std::chrono::steady_clock;
    auto lastDraw   = Clock::now();
    auto lastPongAt = Clock::time_point{};

    while (running) {
        swarm.poll();

        const auto& st = swarm.robotState(target);
        if (st.lastPongAt != Clock::time_point{} && st.lastPongAt != lastPongAt) {
            lastPongAt = st.lastPongAt;
            onPong(st.latencyUs);
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - lastDraw).count() >= 100) {
            drawPlot(target);
            lastDraw = Clock::now();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::cout << "\033[2J\033[H\033[0m";
    return 0;
}
