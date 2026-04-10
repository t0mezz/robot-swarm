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

#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>
#include <deque>
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

#define MAGIC_0       0xAA
#define MAGIC_1       0x55
#define MSG_PONG      0x23
#define MAX_ROBOTS    64

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

    for (int i = 0; i < 50; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        fd = hub_connect();
        if (fd >= 0) { std::cerr << "[auto] Connected to swarm_hub.\n"; return fd; }
    }
    std::cerr << "Error: swarm_hub did not become ready within 5s.\n";
    return -1;
}

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

// ═══════════════════════════════════════════════════════════════
// Serial Parser
// ═══════════════════════════════════════════════════════════════

static uint8_t rxBuf[1024];
static int     rxLen = 0;

static void onPong(uint8_t id, uint16_t us, uint8_t targetId) {
    if (id != targetId) return;
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

static void parsePacket(const uint8_t* data, int len, uint8_t targetId) {
    if (len < 5) return;
    if (data[0] != MAGIC_0 || data[1] != MAGIC_1) return;
    uint8_t type = data[2];
    uint8_t plen = data[3];
    if (4 + plen + 1 > len) return;
    if (crc8(&data[2], plen + 2) != data[4 + plen]) return;

    if (type == MSG_PONG && plen >= 3) {
        uint8_t  id = data[4];
        uint16_t us = data[5] | (data[6] << 8);
        onPong(id, us, targetId);
    }
}

static void readAndParse(int fd, uint8_t targetId) {
    uint8_t tmp[512];
    ssize_t n = read(fd, tmp, sizeof(tmp));
    if (n <= 0) return;
    int space = (int)sizeof(rxBuf) - rxLen;
    int copy  = (n < space) ? (int)n : space;
    memcpy(&rxBuf[rxLen], tmp, copy);
    rxLen += copy;

    while (rxLen >= 5) {
        int idx = -1;
        for (int i = 0; i < rxLen - 1; i++) {
            if (rxBuf[i] == MAGIC_0 && rxBuf[i+1] == MAGIC_1) { idx = i; break; }
        }
        if (idx < 0) { rxLen = 0; return; }
        if (idx > 0) { memmove(rxBuf, &rxBuf[idx], rxLen - idx); rxLen -= idx; }
        if (rxLen < 4) return;
        uint8_t plen     = rxBuf[3];
        int     frameLen = 4 + plen + 1;
        if (rxLen < frameLen) return;
        parsePacket(rxBuf, frameLen, targetId);
        memmove(rxBuf, &rxBuf[frameLen], rxLen - frameLen);
        rxLen -= frameLen;
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

    uint8_t target = (uint8_t)std::stoi(argv[1]);

    int fd = hub_connect_or_start();
    if (fd < 0) return 1;

    signal(SIGINT, signal_handler);

    using Clock = std::chrono::steady_clock;
    auto lastDraw = Clock::now();

    while (running) {
        readAndParse(fd, target);

        if (std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - lastDraw).count() >= 100) {
            drawPlot(target);
            lastDraw = Clock::now();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    close(fd);
    std::cout << "\033[2J\033[H\033[0m";
    return 0;
}
