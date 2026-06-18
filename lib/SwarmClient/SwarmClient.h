// SwarmClient.h — header-only library for robot swarm control
//
// Manages the Unix socket connection to swarm_hub, builds MSG_SWARM frames,
// and parses incoming telemetry/announce packets into per-robot state.
//
// Usage:
//   #include "SwarmClient.h"
//
//   SwarmClient swarm;
//   if (!swarm.connect()) { /* hub not available */ }
//
//   swarm.setSpeed(0, 80, 80);   // robot 0: forward
//   swarm.setAll(0, 0);          // all known: stop
//   swarm.flush();               // send MSG_SWARM to hub
//
//   swarm.poll();                // read incoming packets (call regularly)
//   auto& s = swarm.robotState(0);  // battery, flags, latency, ...

#pragma once

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <algorithm>

#include <unistd.h>
#include <fcntl.h>
#include <glob.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>

// ── Protocol constants (mirrors lib/SwarmProtocol/protocol.h) ────────────────

static constexpr uint8_t SC_MAGIC_0       = 0xAA;
static constexpr uint8_t SC_MAGIC_1       = 0x55;
static constexpr uint8_t SC_MSG_SWARM     = 0x10;
static constexpr uint8_t SC_MSG_ANNOUNCE  = 0x20;
static constexpr uint8_t SC_MSG_PING      = 0x22;
static constexpr uint8_t SC_MSG_PONG      = 0x23;
static constexpr uint8_t SC_MSG_TELEMETRY = 0x30;
static constexpr int     SC_MAX_ROBOTS    = 32;

static constexpr uint8_t SC_STATUS_LOW_BATTERY = 0x04;
static constexpr uint8_t SC_STATUS_ANNOUNCING  = 0x08;

// ── SwarmClient ───────────────────────────────────────────────────────────────

class SwarmClient {
public:
    struct RobotState {
        bool     known     = false;
        uint8_t  mac[6]    = {};
        uint8_t  battery   = 0;
        uint8_t  flags     = 0;
        int8_t   motorL    = 0;
        int8_t   motorR    = 0;
        uint16_t uptime    = 0;
        uint16_t latencyUs = 0;
        bool     hasTelemetry = false;
        std::chrono::steady_clock::time_point lastSeen;
        std::chrono::steady_clock::time_point lastPongAt;
    };

    SwarmClient() {
        memset(m_speeds, 0, sizeof(m_speeds));
    }

    ~SwarmClient() { disconnect(); }

    // ── Connection ────────────────────────────────────────────────

    // Connect to swarm_hub. If autoStartHub is true and the hub is not
    // running, finds the serial port and launches swarm_hub as a daemon.
    bool connect(bool autoStartHub = true) {
        m_fd = openSocket();
        if (m_fd >= 0) return true;
        if (!autoStartHub) return false;

        // Check if hub process is alive but socket not ready yet
        if (hubIsRunning()) {
            fprintf(stderr, "[SwarmClient] hub running, waiting for socket...\n");
            for (int i = 0; i < 20; i++) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                m_fd = openSocket();
                if (m_fd >= 0) return true;
            }
            fprintf(stderr, "[SwarmClient] hub running but socket not ready\n");
            return false;
        }

        std::string port = findSerialPort();
        if (port.empty()) {
            fprintf(stderr, "[SwarmClient] no serial port found; "
                            "start swarm_hub manually\n");
            return false;
        }

        fprintf(stderr, "[SwarmClient] starting swarm_hub on %s\n", port.c_str());
        pid_t pid = fork();
        if (pid == 0) {
            execlp("./swarm_hub", "./swarm_hub", "--daemon", port.c_str(), nullptr);
            execlp("swarm_hub",   "swarm_hub",   "--daemon", port.c_str(), nullptr);
            _exit(1);
        } else if (pid < 0) {
            fprintf(stderr, "[SwarmClient] fork failed: %s\n", strerror(errno));
            return false;
        }
        int status; waitpid(pid, &status, 0);

        for (int i = 0; i < 50; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            m_fd = openSocket();
            if (m_fd >= 0) return true;
        }
        fprintf(stderr, "[SwarmClient] swarm_hub did not become ready within 5s\n");
        return false;
    }

    void disconnect() {
        if (m_fd >= 0) { close(m_fd); m_fd = -1; }
    }

    bool isConnected() const { return m_fd >= 0; }

    // ── Motor control ─────────────────────────────────────────────

    // Set commanded speed for a single robot. Call flush() to transmit.
    void setSpeed(uint8_t id, int8_t left, int8_t right) {
        if (id >= SC_MAX_ROBOTS) return;
        m_speeds[id][0] = left;
        m_speeds[id][1] = right;
    }

    // Set the same speed on all currently known robots.
    void setAll(int8_t left, int8_t right) {
        for (int i = 0; i < SC_MAX_ROBOTS; i++) {
            if (m_robots[i].known) setSpeed(i, left, right);
        }
    }

    // Zero the commanded speed for one robot.
    void stop(uint8_t id) { setSpeed(id, 0, 0); }

    // Zero commanded speeds for all known robots.
    void stopAll() { setAll(0, 0); }

    // Mark a robot as known so flush() will include it.
    // Useful when a robot is detected by vision before it has sent an announce.
    void registerRobot(uint8_t id) {
        if (id < SC_MAX_ROBOTS) m_robots[id].known = true;
    }

    // ── Transmit ──────────────────────────────────────────────────

    // Build an optimised MSG_SWARM containing only known robots and write
    // it to the hub socket. Returns false if the socket is not connected.
    bool flush() {
        if (m_fd < 0) return false;

        uint8_t payload[SC_MAX_ROBOTS * 3];
        int count = 0;
        for (int i = 0; i < SC_MAX_ROBOTS; i++) {
            if (!m_robots[i].known) continue;
            payload[count * 3 + 0] = static_cast<uint8_t>(i);
            payload[count * 3 + 1] = static_cast<uint8_t>(m_speeds[i][0]);
            payload[count * 3 + 2] = static_cast<uint8_t>(m_speeds[i][1]);
            count++;
        }
        if (count == 0) return true;

        uint8_t plen = static_cast<uint8_t>(count * 3);
        uint8_t frame[4 + SC_MAX_ROBOTS * 3 + 1];
        buildFrame(frame, SC_MSG_SWARM, payload, plen);

        std::lock_guard<std::mutex> lock(m_writeMutex);
        ssize_t written = write(m_fd, frame, frameSize(plen));
        if (written < 0) {
            perror("[SwarmClient] write");
            close(m_fd);
            m_fd = -1;
            return false;
        }
        return true;
    }

    // Send a MSG_PING to the given robot, carrying the current timestamp
    // (low 32 bits of microseconds since epoch). The hub relays this to the
    // robot, which echoes a MSG_PONG; poll() then updates
    // robotState(id).latencyUs and lastPongAt.
    bool sendPing(uint8_t targetId) {
        if (m_fd < 0) return false;

        using namespace std::chrono;
        uint32_t ts = static_cast<uint32_t>(
            duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
        uint8_t payload[5] = {
            targetId,
            static_cast<uint8_t>(ts), static_cast<uint8_t>(ts >> 8),
            static_cast<uint8_t>(ts >> 16), static_cast<uint8_t>(ts >> 24)
        };
        uint8_t frame[10];
        buildFrame(frame, SC_MSG_PING, payload, 5);

        std::lock_guard<std::mutex> lock(m_writeMutex);
        ssize_t written = write(m_fd, frame, frameSize(5));
        if (written < 0) {
            perror("[SwarmClient] write");
            close(m_fd);
            m_fd = -1;
            return false;
        }
        return true;
    }

    // ── Receive ───────────────────────────────────────────────────

    // Read incoming bytes from the hub and update robot state.
    // Call this regularly (e.g. every loop iteration) to process
    // telemetry and announce packets. Also drives the round-robin
    // auto-ping (see autoPing()), so latency tracking works for any
    // caller regardless of which program is driving the robots.
    void poll() {
        if (m_fd < 0) return;

        autoPing();

        uint8_t tmp[512];
        ssize_t n = read(m_fd, tmp, sizeof(tmp));
        if (n <= 0) {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("[SwarmClient] read");
                close(m_fd);
                m_fd = -1;
            }
            return;
        }

        int space = static_cast<int>(sizeof(m_rxBuf)) - m_rxLen;
        int copy  = std::min(static_cast<int>(n), space);
        memcpy(&m_rxBuf[m_rxLen], tmp, copy);
        m_rxLen += copy;

        while (m_rxLen >= 5) {
            // Scan for magic bytes
            int idx = -1;
            for (int i = 0; i < m_rxLen - 1; i++) {
                if (m_rxBuf[i] == SC_MAGIC_0 && m_rxBuf[i+1] == SC_MAGIC_1) {
                    idx = i; break;
                }
            }
            if (idx < 0) {
                // No header pair found. If the buffer ends in a lone MAGIC_0, keep
                // it — it may be the start of the next frame with MAGIC_1 still in
                // flight. Dropping it here would desync and lose that frame.
                m_rxLen = (m_rxBuf[m_rxLen - 1] == SC_MAGIC_0) ? 1 : 0;
                if (m_rxLen == 1) m_rxBuf[0] = SC_MAGIC_0;
                return;
            }
            if (idx > 0) {
                memmove(m_rxBuf, m_rxBuf + idx, m_rxLen - idx);
                m_rxLen -= idx;
            }
            if (m_rxLen < 4) return;

            uint8_t plen = m_rxBuf[3];
            int     flen = 4 + plen + 1;
            if (m_rxLen < flen) return;

            parseFrame(m_rxBuf, flen);
            memmove(m_rxBuf, m_rxBuf + flen, m_rxLen - flen);
            m_rxLen -= flen;
        }
    }

    // ── Robot state ───────────────────────────────────────────────

    bool isKnown(uint8_t id) const {
        return id < SC_MAX_ROBOTS && m_robots[id].known;
    }

    std::vector<uint8_t> knownIds() const {
        std::vector<uint8_t> ids;
        for (int i = 0; i < SC_MAX_ROBOTS; i++)
            if (m_robots[i].known) ids.push_back(static_cast<uint8_t>(i));
        return ids;
    }

    const RobotState& robotState(uint8_t id) const {
        static const RobotState empty{};
        return id < SC_MAX_ROBOTS ? m_robots[id] : empty;
    }

private:
    int         m_fd    = -1;
    int8_t      m_speeds[SC_MAX_ROBOTS][2] = {};
    RobotState  m_robots[SC_MAX_ROBOTS];
    uint8_t     m_rxBuf[1024];
    int         m_rxLen = 0;
    std::mutex  m_writeMutex;

    // Round-robin auto-ping: every SC_PING_INTERVAL_MS, ping the next known
    // robot. Runs from poll() so latency tracking doesn't depend on any
    // particular client (e.g. swarm_controller) also being active.
    static constexpr int SC_PING_INTERVAL_MS = 200;
    std::chrono::steady_clock::time_point m_lastAutoPingAt{};
    int m_autoPingIdx = -1;

    void autoPing() {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                now - m_lastAutoPingAt).count() < SC_PING_INTERVAL_MS) return;
        m_lastAutoPingAt = now;

        for (int attempt = 0; attempt < SC_MAX_ROBOTS; attempt++) {
            m_autoPingIdx = (m_autoPingIdx + 1) % SC_MAX_ROBOTS;
            if (m_robots[m_autoPingIdx].known) {
                sendPing((uint8_t)m_autoPingIdx);
                break;
            }
        }
    }

    // ── Protocol helpers ──────────────────────────────────────────

    static uint8_t crc8(const uint8_t* data, uint8_t len) {
        uint8_t crc = 0x00;
        for (uint8_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t b = 0; b < 8; b++)
                crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
        return crc;
    }

    static void buildFrame(uint8_t* buf, uint8_t type,
                           const uint8_t* payload, uint8_t plen) {
        buf[0] = SC_MAGIC_0;
        buf[1] = SC_MAGIC_1;
        buf[2] = type;
        buf[3] = plen;
        memcpy(&buf[4], payload, plen);
        buf[4 + plen] = crc8(&buf[2], plen + 2);
    }

    static int frameSize(uint8_t plen) { return 4 + plen + 1; }

    void parseFrame(const uint8_t* data, int len) {
        if (len < 5) return;
        uint8_t type = data[2];
        uint8_t plen = data[3];
        if (4 + plen + 1 > len) return;
        if (crc8(&data[2], plen + 2) != data[4 + plen]) return;

        const uint8_t* p   = &data[4];
        auto           now = std::chrono::steady_clock::now();

        switch (type) {
            case SC_MSG_ANNOUNCE:
                if (plen >= 7 && p[0] < SC_MAX_ROBOTS) {
                    uint8_t id = p[0];
                    m_robots[id].known = true;
                    memcpy(m_robots[id].mac, &p[1], 6);
                    m_robots[id].lastSeen = now;
                    if (!m_robots[id].hasTelemetry) m_robots[id].flags |= SC_STATUS_ANNOUNCING;
                }
                break;
            case SC_MSG_TELEMETRY:
                if (plen >= 7 && p[0] < SC_MAX_ROBOTS) {
                    uint8_t id = p[0];
                    m_robots[id].known    = true;
                    m_robots[id].battery  = p[1];
                    m_robots[id].flags    = p[2];
                    m_robots[id].motorL   = static_cast<int8_t>(p[3]);
                    m_robots[id].motorR   = static_cast<int8_t>(p[4]);
                    m_robots[id].uptime   = static_cast<uint16_t>(p[5] | (p[6] << 8));
                    m_robots[id].hasTelemetry = true;
                    m_robots[id].lastSeen = now;
                }
                break;
            case SC_MSG_PONG:
                if (plen >= 3 && p[0] < SC_MAX_ROBOTS) {
                    uint8_t id = p[0];
                    m_robots[id].latencyUs = static_cast<uint16_t>(p[1] | (p[2] << 8));
                    m_robots[id].lastSeen  = now;
                    m_robots[id].lastPongAt = now;
                }
                break;
        }
    }

    // ── Hub discovery helpers ─────────────────────────────────────

    static int openSocket() {
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) return -1;
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, "/tmp/swarm_hub.sock", sizeof(addr.sun_path) - 1);
        if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(fd); return -1;
        }
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        return fd;
    }

    static bool hubIsRunning() {
        FILE* f = fopen("/tmp/swarm_hub.pid", "r");
        if (!f) return false;
        pid_t pid = 0;
        if (fscanf(f, "%d", &pid) != 1) pid = 0;
        fclose(f);
        return pid > 0 && kill(pid, 0) == 0;
    }

    static std::string findSerialPort() {
        const char* patterns[] = {
            "/dev/tty.usbmodem*", "/dev/tty.usbserial*",
            "/dev/ttyUSB*",       "/dev/ttyACM*",
            nullptr
        };
        for (int i = 0; patterns[i]; i++) {
            glob_t g{};
            if (glob(patterns[i], 0, nullptr, &g) == 0 && g.gl_pathc > 0) {
                std::string port = g.gl_pathv[0];
                globfree(&g);
                return port;
            }
            globfree(&g);
        }
        return {};
    }
};
