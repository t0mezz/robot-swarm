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

#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <glob.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>

// ═══════════════════════════════════════════════════════════════
// Protokoll
// ═══════════════════════════════════════════════════════════════

#define MAGIC_0           0xAA
#define MAGIC_1           0x55
#define MSG_ANNOUNCE      0x20
#define MSG_PONG          0x23
#define MSG_TELEMETRY     0x30

#define MAX_ROBOTS        32

#define STATUS_LOW_BATTERY    0x04
#define STATUS_ANNOUNCING     0x08

static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════
// Hub Connection
// ═══════════════════════════════════════════════════════════════

static int hub_connect() {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, "/tmp/swarm_hub.sock", sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

static std::string find_serial_port() {
    const char* patterns[] = {
        "/dev/tty.usbmodem*", "/dev/tty.usbserial*",
        "/dev/ttyUSB*",       "/dev/ttyACM*",
        nullptr
    };
    for (int p = 0; patterns[p]; p++) {
        glob_t g{};
        if (glob(patterns[p], 0, nullptr, &g) == 0 && g.gl_pathc > 0) {
            std::string port = g.gl_pathv[0];
            globfree(&g);
            return port;
        }
        globfree(&g);
    }
    return "";
}

static bool hub_is_running() {
    FILE* f = fopen("/tmp/swarm_hub.pid", "r");
    if (!f) return false;
    pid_t pid = 0;
    fscanf(f, "%d", &pid);
    fclose(f);
    return pid > 0 && kill(pid, 0) == 0;
}

static int hub_connect_or_start() {
    int fd = hub_connect();
    if (fd >= 0) return fd;

    // If hub process is alive but socket not ready yet, wait for it
    if (hub_is_running()) {
        std::cerr << "[auto] Hub is running, waiting for socket...\n";
        for (int i = 0; i < 20; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            fd = hub_connect();
            if (fd >= 0) { std::cerr << "[auto] Connected to swarm_hub.\n"; return fd; }
        }
        std::cerr << "Error: hub is running but socket not ready.\n";
        return -1;
    }

    std::string port = find_serial_port();
    if (port.empty()) {
        std::cerr << "Error: swarm_hub not running and no serial port found.\n"
                  << "Start manually: ./swarm_hub /dev/tty.usbmodem*\n";
        return -1;
    }

    std::cerr << "[auto] Starting swarm_hub on " << port << "...\n";
    pid_t pid = fork();
    if (pid == 0) {
        execlp("./swarm_hub", "./swarm_hub", "--daemon", port.c_str(), nullptr);
        execlp("swarm_hub",   "swarm_hub",   "--daemon", port.c_str(), nullptr);
        _exit(1);
    } else if (pid < 0) {
        std::cerr << "Error: fork failed: " << strerror(errno) << "\n";
        return -1;
    }
    int status; waitpid(pid, &status, 0);

    // Poll up to 5s for the socket to appear
    for (int i = 0; i < 50; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        fd = hub_connect();
        if (fd >= 0) { std::cerr << "[auto] Connected to swarm_hub.\n"; return fd; }
    }
    std::cerr << "Error: swarm_hub did not become ready within 5s.\n";
    return -1;
}

// ═══════════════════════════════════════════════════════════════
// Roboter-Status
// ═══════════════════════════════════════════════════════════════

struct RobotStatus {
    bool     registered = false;
    uint8_t  mac[6]     = {};
    uint8_t  battery    = 0;
    uint8_t  flags      = 0;
    int8_t   motorL     = 0;
    int8_t   motorR     = 0;
    uint16_t uptime     = 0;
    uint16_t latencyUs  = 0;

    std::chrono::steady_clock::time_point lastSeen;
    std::chrono::steady_clock::time_point lastTelemetry;
    bool     hasTelemetry = false;
};

static RobotStatus robots[MAX_ROBOTS];
static volatile bool running = true;

void signal_handler(int sig) { (void)sig; running = false; }

// ═══════════════════════════════════════════════════════════════
// Pakete parsen
// ═══════════════════════════════════════════════════════════════

static void parsePacket(const uint8_t* data, int len) {
    if (len < 5) return;
    if (data[0] != MAGIC_0 || data[1] != MAGIC_1) return;

    uint8_t type = data[2];
    uint8_t plen = data[3];

    if (4 + plen + 1 > len) return;
    uint8_t calcCRC = crc8(&data[2], plen + 2);
    if (calcCRC != data[4 + plen]) return;

    const uint8_t* payload = &data[4];
    auto now = std::chrono::steady_clock::now();

    switch (type) {
        case MSG_ANNOUNCE: {
            if (plen < 7) break;
            uint8_t id = payload[0];
            if (id >= MAX_ROBOTS) break;

            robots[id].registered = true;
            memcpy(robots[id].mac, &payload[1], 6);
            robots[id].lastSeen = now;
            if (!robots[id].hasTelemetry)
                robots[id].flags |= STATUS_ANNOUNCING;
            break;
        }

        case MSG_TELEMETRY: {
            if (plen < 7) break;
            uint8_t id = payload[0];
            if (id >= MAX_ROBOTS) break;

            robots[id].registered = true;
            robots[id].battery = payload[1];
            robots[id].flags   = payload[2];
            robots[id].motorL  = (int8_t)payload[3];
            robots[id].motorR  = (int8_t)payload[4];
            robots[id].uptime  = payload[5] | (payload[6] << 8);
            robots[id].lastSeen = now;
            robots[id].lastTelemetry = now;
            robots[id].hasTelemetry = true;
            break;
        }

        case MSG_PONG: {
            if (plen < 3) break;
            uint8_t id = payload[0];
            if (id >= MAX_ROBOTS) break;

            robots[id].latencyUs = payload[1] | (payload[2] << 8);
            robots[id].lastSeen = now;
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Serial-Buffer & Parser
// ═══════════════════════════════════════════════════════════════

static uint8_t rxBuf[1024];
static int     rxLen = 0;

static void readAndParse(int fd) {
    uint8_t tmp[512];
    ssize_t n = read(fd, tmp, sizeof(tmp));
    if (n <= 0) return;

    int space = sizeof(rxBuf) - rxLen;
    int copy = (n < space) ? n : space;
    memcpy(&rxBuf[rxLen], tmp, copy);
    rxLen += copy;

    while (rxLen >= 5) {
        int idx = -1;
        for (int i = 0; i < rxLen - 1; i++) {
            if (rxBuf[i] == MAGIC_0 && rxBuf[i+1] == MAGIC_1) { idx = i; break; }
        }
        if (idx < 0) { rxLen = 0; return; }
        if (idx > 0) {
            memmove(rxBuf, &rxBuf[idx], rxLen - idx);
            rxLen -= idx;
        }
        if (rxLen < 4) return;

        uint8_t plen = rxBuf[3];
        int frameLen = 4 + plen + 1;
        if (rxLen < frameLen) return;

        parsePacket(rxBuf, frameLen);

        memmove(rxBuf, &rxBuf[frameLen], rxLen - frameLen);
        rxLen -= frameLen;
    }
}

// ═══════════════════════════════════════════════════════════════
// Terminal UI
// ═══════════════════════════════════════════════════════════════

static void statusParts(uint8_t flags, int8_t motorL, int8_t motorR,
                        const char*& color, const char*& text) {
    if (flags & STATUS_ANNOUNCING)  { color = "\033[33m"; text = "ANNOUNCE"; return; }
    if (flags & STATUS_LOW_BATTERY) { color = "\033[31m"; text = "LOW BAT "; return; }
    if (motorL != 0 || motorR != 0) { color = "\033[32m"; text = "RUNNING "; return; }
    color = "\033[36m"; text = "IDLE    ";
}

static void drawUI() {
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
        auto& r = robots[i];
        if (!r.registered) continue;

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
            printf(" %2d │ %s │ %s │ %s%s\033[0m │ %s │ %s\n",
                   i, macStr, latStr, sc, st, motorStr, uptimeStr);
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
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int /*argc*/, char* argv[]) {
    std::cout << "Swarm Terminal Monitor\n";

    int fd = hub_connect_or_start();
    if (fd < 0) return 1;
    std::cout << "Connected to swarm_hub (/tmp/swarm_hub.sock)\n\n";

    signal(SIGINT, signal_handler);

    auto lastDraw = std::chrono::steady_clock::now();

    while (running) {
        readAndParse(fd);

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastDraw).count() >= 500) {
            drawUI();
            lastDraw = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    close(fd);
    std::cout << "\n\033[0mStopped.\n";
    return 0;
}
