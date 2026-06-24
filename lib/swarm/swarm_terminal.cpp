// swarm_terminal.cpp
// ═══════════════════════════════════════════════════════════════
// Swarm Terminal Monitor — Terminal-UI für Roboter-Schwarm
// ═══════════════════════════════════════════════════════════════
//
// Zeigt alle registrierten Roboter mit Live-Status im Terminal.
// Verbindet sich mit swarm_hub statt direkt mit dem Dongle.
//
// Voraussetzung:
//   ./swarm_hub /dev/tty.usbmodem* muss laufen
//
// Aufruf:
//   ./swarm_terminal

#include "SwarmClient.h"

#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>

static constexpr int MAX_ROBOTS = SC_MAX_ROBOTS;

static volatile bool running = true;

void signal_handler(int sig) { (void)sig; running = false; }

// ═══════════════════════════════════════════════════════════════
// Terminal UI
// ═══════════════════════════════════════════════════════════════

static void statusParts(uint8_t flags, int8_t motorL, int8_t motorR,
                        const char*& color, const char*& text) {
    if (flags & SC_STATUS_ANNOUNCING)  { color = "\033[33m"; text = "ANNOUNCE"; return; }
    if (flags & SC_STATUS_LOW_BATTERY) { color = "\033[31m"; text = "LOW BAT "; return; }
    if (motorL != 0 || motorR != 0) { color = "\033[32m"; text = "RUNNING "; return; }
    color = "\033[36m"; text = "IDLE    ";
}

static void drawUI(const SwarmClient& swarm) {
    auto now = std::chrono::steady_clock::now();

    std::cout << "\033[H\033[J";

    std::cout << "\033[1;36m═══════════════════════════════════════════════════════════════════════════\033[0m\n";
    std::cout << "\033[1;37m  ROBOT SWARM MONITOR                                                     \033[0m\n";
    std::cout << "\033[1;36m═══════════════════════════════════════════════════════════════════════════\033[0m\n";
    std::cout << "\n";

    std::cout << "\033[1;37m ID │ MAC               │ Latency │ Status   │ Motors    │ Uptime\033[0m\n";
    std::cout << "\033[90m────┼───────────────────┼─────────┼──────────┼───────────┼────────\033[0m\n";

    int activeCount = 0;
    int totalCount  = 0;

    for (int i = 0; i < MAX_ROBOTS; i++) {
        const auto& r = swarm.robotState((uint8_t)i);
        if (!r.known) continue;

        totalCount++;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - r.lastSeen).count();
        bool lost = elapsed > 5000;

        if (!lost) activeCount++;

        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 r.mac[0], r.mac[1], r.mac[2], r.mac[3], r.mac[4], r.mac[5]);

        char latStr[12];
        if (r.latencyUs > 0)
            snprintf(latStr, sizeof(latStr), "%5uus", r.latencyUs);
        else
            snprintf(latStr, sizeof(latStr), "   --- ");

        char motorStr[12];
        snprintf(motorStr, sizeof(motorStr), "%+4d %+4d", r.motorL, r.motorR);

        char uptimeStr[10];
        snprintf(uptimeStr, sizeof(uptimeStr), "%4ds", r.uptime);

        if (lost) {
            printf("\033[31m %2d │ %s │ %s │ LOST     │ %s │ %s\033[0m\n",
                   i, macStr, latStr, motorStr, uptimeStr);
        } else {
            const char* sc; const char* st;
            statusParts(r.flags, r.motorL, r.motorR, sc, st);
            // Color the motor cell too: green when driving, dim when idle.
            const char* motorCol = (r.motorL != 0 || r.motorR != 0) ? "\033[32m" : "\033[90m";
            printf(" %2d │ %s │ %s │ %s%s\033[0m │ %s%s\033[0m │ %s\n",
                   i, macStr, latStr, sc, st, motorCol, motorStr, uptimeStr);
        }
    }

    if (totalCount == 0) {
        std::cout << "\033[90m  Waiting for robots to announce...\033[0m\n";
    }

    std::cout << "\n";
    std::cout << "\033[90m────────────────────────────────────────────────────────────────────────────\033[0m\n";
    printf("\033[37m  Active: %d/%d    Registered: %d/%d\033[0m\n",
           activeCount, MAX_ROBOTS, totalCount, MAX_ROBOTS);
    std::cout << "\033[90m  via swarm_hub (/tmp/swarm_hub.sock)   Ctrl+C to exit\033[0m\n";

    // Debug log — only shown once a robot has sent at least one MSG_DEBUG line.
    const auto& log = swarm.debugLog();
    if (!log.empty()) {
        std::cout << "\n\033[1;37m  DEBUG LOG\033[0m\n";
        std::cout << "\033[90m  age     robot  message\033[0m\n";
        std::cout << "\033[90m────────────────────────────────────────────────────────────────────────────\033[0m\n";

        const int shown = 12;
        int start = (int)log.size() > shown ? (int)log.size() - shown : 0;
        for (int i = start; i < (int)log.size(); i++) {
            const auto& e = log[i];
            double age = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - e.at).count() / 1000.0;
            printf("\033[90m  -%5.1fs \033[36mR%-3d\033[0m %s\n",
                   age, e.robotId, e.text.c_str());
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int /*argc*/, char* argv[]) {
    std::cout << "Swarm Terminal Monitor\n";

    SwarmClient swarm;
    if (!swarm.connect()) return 1;
    std::cout << "Connected to swarm_hub (/tmp/swarm_hub.sock)\n\n";

    signal(SIGINT, signal_handler);

    auto lastDraw = std::chrono::steady_clock::now();

    while (running) {
        swarm.poll();

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastDraw).count() >= 500) {
            drawUI(swarm);
            lastDraw = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    swarm.disconnect();
    std::cout << "\n\033[0mStopped.\n";
    return 0;
}
