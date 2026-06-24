// swarm_hub_simulation.cpp
// ═══════════════════════════════════════════════════════════════
// Swarm Hub Simulation — synthesizes robot telemetry, no hardware
// ═══════════════════════════════════════════════════════════════
//
// Drop-in stand-in for swarm_hub's socket side: binds the same
// /tmp/swarm_hub.sock Unix socket and speaks the same wire protocol,
// but instead of bridging to a real serial dongle it synthesizes
// MSG_ANNOUNCE/MSG_TELEMETRY for a handful of fake robots and replies
// to MSG_PING with a jittered MSG_PONG. Lets PC tools (swarm_terminal,
// latency_plot, swarm_dashboard, ...) be exercised without any
// hardware attached.
//
// MSG_SWARM frames from clients are accepted (so swarm_controller
// doesn't error out) but otherwise ignored — there are no real motors
// to drive.
//
// Aufruf:
//   ./swarm_hub_simulation [num_robots]

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <string>
#include <csignal>
#include <random>
#include <chrono>
#include <algorithm>

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>

#define HUB_SOCK_PATH "/tmp/swarm_hub.sock"
#define MAX_CLIENTS   16
#define MAX_SIM_ROBOTS 32

#define MAGIC_0        0xAA
#define MAGIC_1        0x55
#define MSG_SWARM      0x10
#define MSG_ANNOUNCE   0x20
#define MSG_PING       0x22
#define MSG_PONG       0x23
#define MSG_TELEMETRY  0x30

static constexpr uint8_t STATUS_LOW_BATTERY = 0x04;
static constexpr uint8_t STATUS_ANNOUNCING  = 0x08;

// ── Frame helpers ────────────────────────────────────────────────

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

static int frameSize(uint8_t plen) { return 4 + plen + 1; }

// ── Client management ─────────────────────────────────────────────

struct ClientConn {
    int     fd     = -1;
    bool    active = false;
    uint8_t rxBuf[256];
    int     rxLen  = 0;
};

static ClientConn    clients[MAX_CLIENTS];
static volatile bool g_running = true;

void signal_handler(int) { g_running = false; }

static void client_close(int slot) {
    if (clients[slot].active) {
        close(clients[slot].fd);
        clients[slot].active = false;
        clients[slot].fd     = -1;
        clients[slot].rxLen  = 0;
        std::cout << "[sim] Client disconnected (slot " << slot << ")\n";
    }
}

static void send_to_client(int slot, const uint8_t* frame, int len) {
    ssize_t n = write(clients[slot].fd, frame, len);
    if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) client_close(slot);
}

static void broadcast(const uint8_t* frame, int len) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].active) send_to_client(i, frame, len);
    }
}

static void client_accept(int serverFd) {
    int cfd = accept(serverFd, nullptr, nullptr);
    if (cfd < 0) return;

    int slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) if (!clients[i].active) { slot = i; break; }
    if (slot < 0) { std::cerr << "[sim] Max clients reached, rejecting\n"; close(cfd); return; }

    fcntl(cfd, F_SETFL, fcntl(cfd, F_GETFL, 0) | O_NONBLOCK);
    clients[slot].fd     = cfd;
    clients[slot].active = true;
    std::cout << "[sim] Client connected (slot " << slot << ")\n";
}

// ── Synthetic robot model ─────────────────────────────────────────

struct SimRobot {
    uint8_t  id;
    uint8_t  mac[6];
    float    batteryF;     // 0..255, drains slowly with a slight ripple
    float    motorPhase;   // drives a sine-wave motor pattern
    uint16_t baseLatencyUs;
    uint16_t uptime = 0;
};

static std::mt19937 rng{std::random_device{}()};

static SimRobot makeSimRobot(uint8_t id) {
    SimRobot r;
    r.id = id;
    for (int i = 0; i < 6; i++) r.mac[i] = (uint8_t)(0xC0 + id * 7 + i);
    std::uniform_real_distribution<float> battDist(180.0f, 255.0f);
    std::uniform_real_distribution<float> phaseDist(0.0f, 6.28f);
    std::uniform_int_distribution<int>    latDist(800, 3000);
    r.batteryF      = battDist(rng);
    r.motorPhase    = phaseDist(rng);
    r.baseLatencyUs = (uint16_t)latDist(rng);
    return r;
}

static void sendAnnounce(const SimRobot& r) {
    uint8_t payload[7] = { r.id, r.mac[0], r.mac[1], r.mac[2], r.mac[3], r.mac[4], r.mac[5] };
    uint8_t frame[4 + 7 + 1];
    buildFrame(frame, MSG_ANNOUNCE, payload, 7);
    broadcast(frame, frameSize(7));
}

static void sendTelemetry(SimRobot& r, double t) {
    // Slow drain with a small ripple so the battery meter isn't static.
    r.batteryF -= 0.01f;
    if (r.batteryF < 20.0f) r.batteryF = 255.0f;  // simulate a "swap" once drained
    uint8_t battery = (uint8_t)std::clamp(r.batteryF, 0.0f, 255.0f);

    float   motor = 100.0f * (float)std::sin(t * 0.5 + r.motorPhase);
    int8_t  motorL = (int8_t)std::clamp(motor, -127.0f, 127.0f);
    int8_t  motorR = (int8_t)std::clamp(motor * 0.9f, -127.0f, 127.0f);

    uint8_t flags = (battery < 40) ? STATUS_LOW_BATTERY : 0;
    r.uptime++;

    uint8_t payload[7] = {
        r.id, battery, flags,
        (uint8_t)motorL, (uint8_t)motorR,
        (uint8_t)r.uptime, (uint8_t)(r.uptime >> 8)
    };
    uint8_t frame[4 + 7 + 1];
    buildFrame(frame, MSG_TELEMETRY, payload, 7);
    broadcast(frame, frameSize(7));
}

static void sendPong(uint8_t robotId, uint16_t baseLatencyUs) {
    std::uniform_int_distribution<int> jitter(-150, 400);
    uint16_t lat = (uint16_t)std::clamp((int)baseLatencyUs + jitter(rng), 100, 65000);
    uint8_t payload[3] = { robotId, (uint8_t)lat, (uint8_t)(lat >> 8) };
    uint8_t frame[4 + 3 + 1];
    buildFrame(frame, MSG_PONG, payload, 3);
    broadcast(frame, frameSize(3));
}

// ── Client -> hub frame parsing (only MSG_PING matters here) ──────

static SimRobot* g_robots = nullptr;
static int       g_numRobots = 0;

static SimRobot* findRobot(uint8_t id) {
    for (int i = 0; i < g_numRobots; i++) if (g_robots[i].id == id) return &g_robots[i];
    return nullptr;
}

static void process_client_data(int slot) {
    ClientConn& c = clients[slot];
    uint8_t tmp[256];
    ssize_t n = read(c.fd, tmp, sizeof(tmp));
    if (n <= 0) { client_close(slot); return; }

    int space = (int)sizeof(c.rxBuf) - c.rxLen;
    int copy  = (n < space) ? (int)n : space;
    memcpy(&c.rxBuf[c.rxLen], tmp, copy);
    c.rxLen += copy;

    while (c.rxLen >= 4) {
        int idx = -1;
        for (int i = 0; i < c.rxLen - 1; i++) {
            if (c.rxBuf[i] == MAGIC_0 && c.rxBuf[i+1] == MAGIC_1) { idx = i; break; }
        }
        if (idx < 0) {
            c.rxLen = (c.rxBuf[c.rxLen - 1] == MAGIC_0) ? 1 : 0;
            if (c.rxLen == 1) c.rxBuf[0] = MAGIC_0;
            break;
        }
        if (idx > 0) { memmove(c.rxBuf, c.rxBuf + idx, c.rxLen - idx); c.rxLen -= idx; }
        if (c.rxLen < 4) break;

        int flen = frameSize(c.rxBuf[3]);
        if (flen > (int)sizeof(c.rxBuf)) { c.rxLen = 0; break; }
        if (c.rxLen < flen) break;

        if (c.rxBuf[2] == MSG_PING) {
            uint8_t plen = c.rxBuf[3];
            if (plen >= 1) {
                uint8_t targetId = c.rxBuf[4];
                if (SimRobot* r = findRobot(targetId)) sendPong(r->id, r->baseLatencyUs);
            }
        }
        // MSG_SWARM and anything else from clients: accepted, no-op (no real motors).

        memmove(c.rxBuf, c.rxBuf + flen, c.rxLen - flen);
        c.rxLen -= flen;
    }
}

// ── Server socket ──────────────────────────────────────────────────

static int hub_server_create() {
    unlink(HUB_SOCK_PATH);
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { std::cerr << "socket() failed: " << strerror(errno) << "\n"; return -1; }

    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, HUB_SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "bind() failed: " << strerror(errno) << "\n"; close(fd); return -1;
    }
    if (listen(fd, MAX_CLIENTS) < 0) {
        std::cerr << "listen() failed: " << strerror(errno) << "\n"; close(fd); return -1;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    return fd;
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    int numRobots = 5;
    if (argc > 1) numRobots = std::clamp(std::atoi(argv[1]), 1, MAX_SIM_ROBOTS);

    SimRobot robots[MAX_SIM_ROBOTS];
    for (int i = 0; i < numRobots; i++) robots[i] = makeSimRobot((uint8_t)i);
    g_robots    = robots;
    g_numRobots = numRobots;

    int serverFd = hub_server_create();
    if (serverFd < 0) return 1;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);

    std::cout << "=== Swarm Hub Simulation ===\n";
    std::cout << "Socket : " << HUB_SOCK_PATH << "\n";
    std::cout << "Robots : " << numRobots << " (synthetic, no hardware)\n";
    std::cout << "Ready.\n\n";

    auto start         = std::chrono::steady_clock::now();
    auto lastTelemetry = start;
    constexpr int TELEMETRY_INTERVAL_MS = 300;

    while (g_running) {
        struct pollfd fds[1 + MAX_CLIENTS];
        int           slotOf[1 + MAX_CLIENTS];
        int           nfds = 0;

        fds[nfds] = { serverFd, POLLIN, 0 }; slotOf[nfds] = -2; nfds++;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].active) { fds[nfds] = { clients[i].fd, POLLIN, 0 }; slotOf[nfds] = i; nfds++; }
        }

        poll(fds, nfds, TELEMETRY_INTERVAL_MS);

        for (int i = 0; i < nfds; i++) {
            if (!(fds[i].revents & POLLIN)) continue;
            if (slotOf[i] == -2) client_accept(serverFd);
            else                 process_client_data(slotOf[i]);
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTelemetry).count()
                >= TELEMETRY_INTERVAL_MS) {
            double t = std::chrono::duration<double>(now - start).count();
            for (int i = 0; i < numRobots; i++) {
                if (robots[i].uptime == 0) sendAnnounce(robots[i]);
                sendTelemetry(robots[i], t);
            }
            lastTelemetry = now;
        }
    }

    for (int i = 0; i < MAX_CLIENTS; i++) client_close(i);
    close(serverFd);
    unlink(HUB_SOCK_PATH);

    std::cout << "\n[sim] Stopped.\n";
    return 0;
}
