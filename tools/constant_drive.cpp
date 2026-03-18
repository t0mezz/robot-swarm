// constant_drive.cpp
// Lightweight controller that continuously sends a constant drive command via
// swarm_hub Unix socket.
//
// Usage:
//   ./constant_drive [-i <robot_id>|-a] [-s <speed>] [-r <rate_ms>]
//
// Examples:
//   ./constant_drive -a         # drive ALL robots forward at default speed
//   ./constant_drive -i 0 -s 80 # drive robot 0 forward at speed 80

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <chrono>
#include <thread>

static const char* HUB_SOCK_PATH = "/tmp/swarm_hub.sock";
static const uint8_t MAGIC_0 = 0xAA;
static const uint8_t MAGIC_1 = 0x55;
static const uint8_t MSG_SWARM = 0x10;
static const int MAX_ROBOTS = 5;

static volatile bool g_running = true;

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

static int connectHub() {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, HUB_SOCK_PATH, sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void signalHandler(int) {
    g_running = false;
}

int main(int argc, char* argv[]) {
    int targetId = -1;  // -1 = all
    int8_t speed = 60;
    int rateMs = 50;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-i") && i + 1 < argc) {
            targetId = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-a")) {
            targetId = -1;
        } else if (!strcmp(argv[i], "-s") && i + 1 < argc) {
            speed = (int8_t)atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-r") && i + 1 < argc) {
            rateMs = atoi(argv[++i]);
        } else {
            fprintf(stderr, "Usage: %s [-i <robot_id>|-a] [-s <speed>] [-r <rate_ms>]\n", argv[0]);
            return 1;
        }
    }

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    int fd = -1;
    for (int attempt = 0; attempt < 50 && fd < 0; attempt++) {
        fd = connectHub();
        if (fd >= 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (fd < 0) {
        fprintf(stderr, "Failed to connect to swarm_hub at %s\n", HUB_SOCK_PATH);
        return 1;
    }

    uint8_t payload[MAX_ROBOTS * 3];
    uint8_t frame[5 + MAX_ROBOTS * 3];

    while (g_running) {
        // Build payload (broadcast every robot if targetId == -1)
        for (int i = 0; i < MAX_ROBOTS; i++) {
            payload[i * 3]     = (uint8_t)i;
            if (targetId < 0 || targetId == i) {
                payload[i * 3 + 1] = (uint8_t)speed;
                payload[i * 3 + 2] = (uint8_t)speed;
            } else {
                payload[i * 3 + 1] = 0;
                payload[i * 3 + 2] = 0;
            }
        }
        buildFrame(frame, MSG_SWARM, payload, MAX_ROBOTS * 3);

        ssize_t n = write(fd, frame, sizeof(frame));
        if (n < 0) break;

        std::this_thread::sleep_for(std::chrono::milliseconds(rateMs));
    }

    // Send a final stop packet (best effort)
    if (fd >= 0) {
        for (int i = 0; i < MAX_ROBOTS; i++) {
            payload[i * 3]     = (uint8_t)i;
            payload[i * 3 + 1] = 0;
            payload[i * 3 + 2] = 0;
        }
        buildFrame(frame, MSG_SWARM, payload, MAX_ROBOTS * 3);
        write(fd, frame, sizeof(frame));
        close(fd);
    }

    return 0;
}
