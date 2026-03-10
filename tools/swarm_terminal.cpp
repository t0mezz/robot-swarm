// swarm_terminal.cpp
// ═══════════════════════════════════════════════════════════════
// Swarm Terminal Monitor — Terminal-UI für Roboter-Schwarm
// ═══════════════════════════════════════════════════════════════
//
// Zeigt alle registrierten Roboter mit Live-Status im Terminal.
// Sendet periodische Pings für Latenz-Messung.
// Sendet Swarm-Pakete (alle Motoren 0 = idle, oder Test-Pattern).
//
// Kompilieren (macOS):
//   g++ swarm_terminal.cpp -o swarm_terminal -std=c++17 -framework IOKit
//
// Kompilieren (Linux):
//   g++ swarm_terminal.cpp -o swarm_terminal -std=c++17
//
// Aufruf:
//   ./swarm_terminal /dev/tty.usbmodem* [send_ms]

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
#include <termios.h>
#include <errno.h>
#ifdef __APPLE__
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>
#endif

// ═══════════════════════════════════════════════════════════════
// Protokoll
// ═══════════════════════════════════════════════════════════════

#define MAGIC_0           0xAA
#define MAGIC_1           0x55
#define MSG_SWARM         0x10
#define MSG_ANNOUNCE      0x20
#define MSG_PING          0x22
#define MSG_PONG          0x23
#define MSG_TELEMETRY     0x30

#define MAX_ROBOTS        20

#define STATUS_MOTOR_ACTIVE   0x01
#define STATUS_SENSOR_ERROR   0x02
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
        std::cerr << "Cannot open " << path << ": " << strerror(errno) << std::endl;
        return -1;
    }
    struct termios tty{};
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
#ifdef __APPLE__
    if (baud > 230400) { speed_t c = (speed_t)baud; ioctl(fd, IOSSIOSPEED, &c); }
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
// Roboter-Status
// ═══════════════════════════════════════════════════════════════

struct RobotStatus {
    bool     registered = false;
    uint8_t  mac[6]     = {};
    int8_t   rssi       = 0;
    uint8_t  battery    = 0;
    uint8_t  flags      = 0;
    int8_t   motorL     = 0;
    int8_t   motorR     = 0;
    uint16_t uptime     = 0;
    uint16_t latencyUs  = 0;

    // Timing
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
            // Robot registriert sich: [id][mac x6][rssi]
            if (plen < 8) break;
            uint8_t id = payload[0];
            if (id >= MAX_ROBOTS) break;

            robots[id].registered = true;
            memcpy(robots[id].mac, &payload[1], 6);
            robots[id].rssi = (int8_t)payload[7];
            robots[id].lastSeen = now;
            robots[id].flags |= STATUS_ANNOUNCING;
            break;
        }

        case MSG_TELEMETRY: {
            // [id][rssi][battery][flags][motorL][motorR][uptime_lo][uptime_hi]
            if (plen < 8) break;
            uint8_t id = payload[0];
            if (id >= MAX_ROBOTS) break;

            robots[id].registered = true;
            robots[id].rssi    = (int8_t)payload[1];
            robots[id].battery = payload[2];
            robots[id].flags   = payload[3];
            robots[id].motorL  = (int8_t)payload[4];
            robots[id].motorR  = (int8_t)payload[5];
            robots[id].uptime  = payload[6] | (payload[7] << 8);
            robots[id].lastSeen = now;
            robots[id].lastTelemetry = now;
            robots[id].hasTelemetry = true;
            break;
        }

        case MSG_PONG: {
            // [id][roundtrip_us_lo][roundtrip_us_hi]
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

    // An Buffer anfügen
    int space = sizeof(rxBuf) - rxLen;
    int copy = (n < space) ? n : space;
    memcpy(&rxBuf[rxLen], tmp, copy);
    rxLen += copy;

    // Pakete extrahieren
    while (rxLen >= 5) {
        // Magic suchen
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
// Swarm-Paket senden (alle Motoren idle)
// ═══════════════════════════════════════════════════════════════

static void sendSwarmIdle(int fd) {
    uint8_t payload[MAX_ROBOTS * 3];
    memset(payload, 0, sizeof(payload));
    for (int i = 0; i < MAX_ROBOTS; i++) {
        payload[i * 3] = (uint8_t)i;
    }
    uint8_t frame[5 + MAX_ROBOTS * 3];
    buildFrame(frame, MSG_SWARM, payload, MAX_ROBOTS * 3);
    write(fd, frame, sizeof(frame));
}

// ═══════════════════════════════════════════════════════════════
// Ping senden
// ═══════════════════════════════════════════════════════════════

static void sendPing(int fd, uint8_t robotId) {
    uint32_t ts = (uint32_t)(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() & 0xFFFFFFFF);

    uint8_t payload[5];
    payload[0] = robotId;
    payload[1] = ts & 0xFF;
    payload[2] = (ts >> 8) & 0xFF;
    payload[3] = (ts >> 16) & 0xFF;
    payload[4] = (ts >> 24) & 0xFF;

    uint8_t frame[10];
    buildFrame(frame, MSG_PING, payload, 5);
    write(fd, frame, 10);
}

// ═══════════════════════════════════════════════════════════════
// Terminal UI
// ═══════════════════════════════════════════════════════════════

static const char* statusStr(uint8_t flags) {
    if (flags & STATUS_ANNOUNCING) return "\033[33mANNOUNCE\033[0m";
    if (flags & STATUS_SENSOR_ERROR) return "\033[31mERROR\033[0m";
    if (flags & STATUS_LOW_BATTERY) return "\033[31mLOW BAT\033[0m";
    if (flags & STATUS_MOTOR_ACTIVE) return "\033[32mRUNNING\033[0m";
    return "\033[36mIDLE\033[0m";
}

static const char* rssiBar(int8_t rssi) {
    if (rssi > -40) return "\033[32m████\033[0m";
    if (rssi > -55) return "\033[32m███\033[0m ";
    if (rssi > -70) return "\033[33m██\033[0m  ";
    if (rssi > -80) return "\033[31m█\033[0m   ";
    return "\033[31m·\033[0m   ";
}

static void drawUI() {
    auto now = std::chrono::steady_clock::now();

    // Cursor nach oben, Bildschirm löschen
    std::cout << "\033[H\033[J";

    // Header
    std::cout << "\033[1;36m═══════════════════════════════════════════════════════════════════════════\033[0m\n";
    std::cout << "\033[1;37m  ROBOT SWARM MONITOR                                                     \033[0m\n";
    std::cout << "\033[1;36m═══════════════════════════════════════════════════════════════════════════\033[0m\n";
    std::cout << "\n";

    // Tabellen-Header
    std::cout << "\033[1;37m ID │ MAC               │ RSSI │ Signal │ Latency │ Status   │ Motors  │ Uptime\033[0m\n";
    std::cout << "\033[90m────┼───────────────────┼──────┼────────┼─────────┼──────────┼─────────┼────────\033[0m\n";

    int activeCount = 0;
    int totalCount = 0;

    for (int i = 0; i < MAX_ROBOTS; i++) {
        auto& r = robots[i];
        if (!r.registered) continue;

        totalCount++;

        // Timeout check: 5s ohne Telemetrie = lost
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

        char motorStr[10];
        snprintf(motorStr, sizeof(motorStr), "%+4d %+4d", r.motorL, r.motorR);

        char uptimeStr[10];
        snprintf(uptimeStr, sizeof(uptimeStr), "%4ds", r.uptime);

        if (lost) {
            std::cout << "\033[31m";
            printf(" %2d │ %s │ %4d │ ·    │ %s │ LOST     │ %s │ %s",
                   i, macStr, r.rssi, latStr, motorStr, uptimeStr);
            std::cout << "\033[0m\n";
        } else {
            printf(" %2d │ %s │ %4d │ %s│ %s │ %-8s │ %s │ %s\n",
                   i, macStr, r.rssi, rssiBar(r.rssi), latStr,
                   "", motorStr, uptimeStr);
            // Status mit Farbe nochmal drüber (cursor back)
            std::cout << "\033[1A\033[59G" << statusStr(r.flags) << "\033[1B\033[0G";
        }
    }

    if (totalCount == 0) {
        std::cout << "\033[90m  Waiting for robots to announce...\033[0m\n";
    }

    std::cout << "\n";
    std::cout << "\033[90m────────────────────────────────────────────────────────────────────────────\033[0m\n";
    printf("\033[37m  Active: %d/%d    Registered: %d/%d\033[0m\n",
           activeCount, MAX_ROBOTS, totalCount, MAX_ROBOTS);
    std::cout << "\033[90m  Ctrl+C to exit\033[0m\n";
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    std::string port_path = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    int         send_ms   = (argc > 2) ? std::stoi(argv[2]) : 50;

    std::cout << "Swarm Terminal Monitor\n";
    std::cout << "Port: " << port_path << "  Interval: " << send_ms << "ms\n\n";

    int fd = serial_open(port_path, 921600);
    if (fd < 0) return 1;

    signal(SIGINT, signal_handler);

    auto lastSwarm   = std::chrono::steady_clock::now();
    auto lastPing    = std::chrono::steady_clock::now();
    auto lastDraw    = std::chrono::steady_clock::now();
    int  pingRobot   = 0;   // Round-Robin Ping

    while (running) {
        auto now = std::chrono::steady_clock::now();

        // Serial lesen und parsen
        readAndParse(fd);

        // Swarm-Paket senden (idle = alle 0)
        auto swarmElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSwarm).count();
        if (swarmElapsed >= send_ms) {
            sendSwarmIdle(fd);
            lastSwarm = now;
        }

        // Ping: alle 200ms den nächsten registrierten Robot pingen (Round-Robin)
        auto pingElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPing).count();
        if (pingElapsed >= 200) {
            // Nächsten aktiven Robot finden
            for (int attempt = 0; attempt < MAX_ROBOTS; attempt++) {
                pingRobot = (pingRobot + 1) % MAX_ROBOTS;
                if (robots[pingRobot].registered) {
                    sendPing(fd, pingRobot);
                    break;
                }
            }
            lastPing = now;
        }

        // UI zeichnen alle 500ms
        auto drawElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastDraw).count();
        if (drawElapsed >= 500) {
            drawUI();
            lastDraw = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Stop: alle Motoren 0
    sendSwarmIdle(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sendSwarmIdle(fd);

    close(fd);
    std::cout << "\n\033[0mStopped.\n";
    return 0;
}
