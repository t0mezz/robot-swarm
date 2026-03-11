// latency_plot.cpp
// ═══════════════════════════════════════════════════════════════
// Latency Plotter — Live ASCII-Plot der ESP-NOW Latenz
// ═══════════════════════════════════════════════════════════════
//
// Sendet Pings an einen bestimmten Roboter und zeichnet die
// Roundtrip-Latenz als rollenden ASCII-Balkendiagramm.
//
// Kompilieren (macOS):
//   g++ latency_plot.cpp -o latency_plot -std=c++17 -framework IOKit
//
// Kompilieren (Linux):
//   g++ latency_plot.cpp -o latency_plot -std=c++17
//
// Aufruf:
//   ./latency_plot /dev/tty.usbserial-0001 <robot_id>

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
#include <termios.h>
#include <errno.h>
#ifdef __APPLE__
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>
#endif

// ═══════════════════════════════════════════════════════════════
// Protokoll
// ═══════════════════════════════════════════════════════════════

#define MAGIC_0       0xAA
#define MAGIC_1       0x55
#define MSG_SWARM     0x10
#define MSG_PING      0x22
#define MSG_PONG      0x23
#define MAX_ROBOTS    20

static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

static void buildFrame(uint8_t* buf, uint8_t type, const uint8_t* payload, uint8_t plen) {
    buf[0] = MAGIC_0;
    buf[1] = MAGIC_1;
    buf[2] = type;
    buf[3] = plen;
    memcpy(&buf[4], payload, plen);
    buf[4 + plen] = crc8(&buf[2], plen + 2);
}

// ═══════════════════════════════════════════════════════════════
// Serial Port
// ═══════════════════════════════════════════════════════════════

static int serial_open(const std::string& path, int baud) {
    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Cannot open " << path << ": " << strerror(errno) << "\n";
        return -1;
    }
    {
        int pins = 0;
        ioctl(fd, TIOCMGET, &pins);
        pins &= ~(TIOCM_DTR | TIOCM_RTS);
        ioctl(fd, TIOCMSET, &pins);
    }
    struct termios tty{};
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | HUPCL);
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
#ifdef __APPLE__
    {
        speed_t speed = (speed_t)baud;
        ioctl(fd, IOSSIOSPEED, &speed);
    }
#else
    if (baud > 230400) {
        speed_t h = (baud == 460800) ? B460800 : B921600;
        cfsetispeed(&tty, h); cfsetospeed(&tty, h);
        tcsetattr(fd, TCSANOW, &tty);
    }
#endif
    return fd;
}

// ═══════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════

static volatile bool running = true;
void signal_handler(int) { running = false; }

static constexpr int  PLOT_WIDTH   =   60;   // number of samples shown
static constexpr int  PLOT_HEIGHT  =   12;   // rows in the chart
static constexpr int  PING_MS      = 1000;
static constexpr int  SWARM_MS     = 1000;

static std::deque<uint16_t> samples;       // rolling window

static uint16_t latestUs    =     0;
static uint16_t minUs       = 65535;
static uint16_t maxUs       =     0;
static uint64_t sumUs       =     0;
static uint32_t sampleCount =     0;

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

    samples.push_back(us);
    if ((int)samples.size() > PLOT_WIDTH) samples.pop_front();
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
        uint8_t plen    = rxBuf[3];
        int     frameLen = 4 + plen + 1;
        if (rxLen < frameLen) return;
        parsePacket(rxBuf, frameLen, targetId);
        memmove(rxBuf, &rxBuf[frameLen], rxLen - frameLen);
        rxLen -= frameLen;
    }
}

// ═══════════════════════════════════════════════════════════════
// Senders
// ═══════════════════════════════════════════════════════════════

static void sendPing(int fd, uint8_t robotId) {
    uint32_t ts = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);
    uint8_t payload[5] = { robotId,
        (uint8_t)(ts), (uint8_t)(ts >> 8),
        (uint8_t)(ts >> 16), (uint8_t)(ts >> 24) };
    uint8_t frame[10];
    buildFrame(frame, MSG_PING, payload, 5);
    write(fd, frame, 10);
}

static void sendSwarmIdle(int fd) {
    uint8_t payload[MAX_ROBOTS * 3];
    memset(payload, 0, sizeof(payload));
    for (int i = 0; i < MAX_ROBOTS; i++) payload[i * 3] = (uint8_t)i;
    uint8_t frame[5 + MAX_ROBOTS * 3];
    buildFrame(frame, MSG_SWARM, payload, MAX_ROBOTS * 3);
    write(fd, frame, sizeof(frame));
}

// ═══════════════════════════════════════════════════════════════
// Plot
// ═══════════════════════════════════════════════════════════════

// Unicode block chars for sub-row precision: ▁▂▃▄▅▆▇█
static const char* BLOCKS[] = { " ", "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█" };

static void drawPlot(uint8_t robotId, const std::string& port) {
    // Clear screen and move to top-left
    std::cout << "\033[2J\033[H";

    // ── Header ────────────────────────────────────────────────
    std::cout << "\033[1;37mLatency Plot\033[0m";
    std::cout << "  Robot \033[1;33m" << (int)robotId << "\033[0m";
    std::cout << "  Port: \033[90m" << port << "\033[0m\n";

    if (sampleCount == 0) {
        std::cout << "\n  \033[90mWaiting for pings...\033[0m\n";
        std::cout.flush();
        return;
    }

    uint16_t avgUs = (uint16_t)(sumUs / sampleCount);

    std::cout << "  Now: \033[1;32m" << latestUs << " µs\033[0m"
              << "   Min: \033[32m"  << minUs    << " µs\033[0m"
              << "   Max: \033[31m"  << maxUs    << " µs\033[0m"
              << "   Avg: \033[36m"  << avgUs    << " µs\033[0m"
              << "   n=" << sampleCount << '\n';

    // ── Determine y-axis range ─────────────────────────────────
    uint16_t lo = minUs;
    uint16_t hi = maxUs;
    if (hi == lo) hi = lo + 1;                 // avoid divide-by-zero
    // Pad 10% above max so the tallest bar doesn't touch the ceiling
    hi = (uint16_t)(hi * 1.10f);

    // ── Draw grid + bars ──────────────────────────────────────
    // Each row covers (hi-lo)/PLOT_HEIGHT µs
    // We work top-to-bottom, row 0 = highest µs value.
    // For each column (sample), compute filled rows using 8 sub-steps.

    int N = (int)samples.size();

    // Print blank columns on the left when buffer isn't full yet
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

        // Padding
        for (int c = 0; c < padCols; c++) std::cout << ' ';

        // Bars
        for (int c = 0; c < N; c++) {
            float v = (float)samples[c];
            // How many 1/8-rows does this sample fill in this row?
            // Row covers [rowLo, rowHi). Value above rowLo = filled portion.
            float rowLo = lo + (float)row * (hi - lo) / PLOT_HEIGHT;
            if (v <= rowLo) {
                std::cout << ' ';
            } else if (v >= rowHi) {
                // Pick color: green ≤ avg, yellow ≤ 1.5×avg, red otherwise
                uint16_t s = samples[c];
                if      (s <= avgUs)          std::cout << "\033[32m█\033[0m";
                else if (s <= avgUs * 3 / 2)  std::cout << "\033[33m█\033[0m";
                else                          std::cout << "\033[31m█\033[0m";
            } else {
                // Partial: determine sub-block
                float fill = (v - rowLo) / (rowHi - rowLo);  // 0..1
                int eighth = (int)(fill * 8.0f + 0.5f);
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

    // ── X-axis ────────────────────────────────────────────────
    std::cout << "         └";
    for (int c = 0; c < PLOT_WIDTH; c++) std::cout << '-';
    std::cout << '\n';
    std::cout << "\033[90m           ← " << PLOT_WIDTH << " samples ("
              << (PLOT_WIDTH * PING_MS / 1000) << "s window)\033[0m\n";
    std::cout.flush();
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <port> <robot_id>\n";
        std::cerr << "  e.g. " << argv[0] << " /dev/tty.usbserial-0001 0\n";
        return 1;
    }

    std::string port   = argv[1];
    uint8_t     target = (uint8_t)std::stoi(argv[2]);

    int fd = serial_open(port, 115200);
    if (fd < 0) return 1;

    signal(SIGINT, signal_handler);

    using Clock = std::chrono::steady_clock;
    auto lastPing  = Clock::now();
    auto lastSwarm = Clock::now();
    auto lastDraw  = Clock::now();

    while (running) {
        auto now = Clock::now();

        readAndParse(fd, target);

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSwarm).count() >= SWARM_MS) {
            sendSwarmIdle(fd);
            lastSwarm = now;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPing).count() >= PING_MS) {
            sendPing(fd, target);
            lastPing = now;
        }

        // Redraw at ~10 fps
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastDraw).count() >= 100) {
            drawPlot(target, port);
            lastDraw = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    close(fd);
    std::cout << "\033[2J\033[H\033[0m";
    return 0;
}
