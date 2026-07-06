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

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

// ── Protocol constants (mirrors lib/SwarmProtocol/protocol.h) ────────────────

static constexpr uint8_t SC_MAGIC_0       = 0xAA;
static constexpr uint8_t SC_MAGIC_1       = 0x55;
static constexpr uint8_t SC_MSG_DEBUG     = 0x02;
static constexpr uint8_t SC_MSG_SWARM     = 0x10;
static constexpr uint8_t SC_MSG_ANNOUNCE  = 0x20;
static constexpr uint8_t SC_MSG_PING      = 0x22;
static constexpr uint8_t SC_MSG_PONG      = 0x23;
static constexpr uint8_t SC_MSG_TELEMETRY = 0x30;
static constexpr int     SC_MAX_ROBOTS    = 32;

// Debug value types (mirrors lib/SwarmProtocol/debug_protocol.h), used by the
// MSG_DEBUG robot->PC debug-log channel.
static constexpr uint8_t SC_DBG_FLOAT32 = 0x01;
static constexpr uint8_t SC_DBG_INT8    = 0x02;
static constexpr uint8_t SC_DBG_INT16   = 0x03;
static constexpr uint8_t SC_DBG_STRING  = 0x04;

static constexpr uint8_t SC_STATUS_LOW_BATTERY = 0x04;
static constexpr uint8_t SC_STATUS_ANNOUNCING  = 0x08;
static constexpr uint8_t SC_STATUS_BAT_VALID   = 0x10;  // battery byte is real data, not "never measured"

// Battery telemetry byte -> volts (40mV/LSB). Only meaningful when the robot's
// flags carry SC_STATUS_BAT_VALID.
static inline float scBatteryVolts(uint8_t raw) { return raw * 0.04f; }

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

    // One robot->PC debug-log line (MSG_DEBUG). Robots emit these via
    // UARTProtocol.send_debug(); the receiver prepends robotId.
    struct DebugEntry {
        uint8_t     robotId = 0;
        uint8_t     fieldId = 0;
        std::string text;
        std::chrono::steady_clock::time_point at;
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
        std::string hubPath = siblingHubPath();  // resolve before fork()
        pid_t pid = fork();
        if (pid == 0) {
            if (!hubPath.empty())
                execl(hubPath.c_str(), hubPath.c_str(), "--daemon", port.c_str(), nullptr);
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
    // telemetry, announce and pong packets. Latency (latencyUs) is
    // populated from MSG_PONG frames; the round-robin pinging that
    // produces them is driven centrally by swarm_hub, not per-client,
    // so exactly one ping is ever in flight regardless of how many
    // tools are connected.
    void poll() {
        if (m_fd < 0) return;

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

    // Rolling buffer of robot->PC debug-log lines (oldest first, capped at
    // SC_DEBUG_LOG_MAX). Empty until a robot sends its first MSG_DEBUG.
    const std::vector<DebugEntry>& debugLog() const { return m_debugLog; }
    void clearDebugLog() { m_debugLog.clear(); }

private:
    int         m_fd    = -1;
    int8_t      m_speeds[SC_MAX_ROBOTS][2] = {};
    RobotState  m_robots[SC_MAX_ROBOTS];
    uint8_t     m_rxBuf[1024];
    int         m_rxLen = 0;
    std::mutex  m_writeMutex;

    static constexpr size_t SC_DEBUG_LOG_MAX = 200;
    std::vector<DebugEntry> m_debugLog;

    // Note: round-robin pinging is driven centrally by swarm_hub (it snoops
    // announce/telemetry/pong frames to learn live robot IDs and emits one
    // MSG_PING per interval). It used to live here, per-client, but with N
    // tools connected that produced N independent ping streams that overran
    // the dongle's single-slot ping tracker (corrupting latency) and added
    // serial/airtime load. sendPing() remains public for manual/diagnostic use.

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

    // Render a MSG_DEBUG value payload to text per its value_type.
    static std::string formatDebugValue(uint8_t vtype, const uint8_t* d, int len) {
        if (len < 0) len = 0;
        char buf[64];
        switch (vtype) {
            case SC_DBG_FLOAT32:
                if (len >= 4) { float f; memcpy(&f, d, 4);
                    snprintf(buf, sizeof(buf), "%.2f", f); return buf; }
                break;
            case SC_DBG_INT8:
                if (len >= 1) {
                    snprintf(buf, sizeof(buf), "%d", (int)(int8_t)d[0]); return buf; }
                break;
            case SC_DBG_INT16:
                if (len >= 2) { int16_t v = (int16_t)(d[0] | (d[1] << 8));
                    snprintf(buf, sizeof(buf), "%d", (int)v); return buf; }
                break;
            case SC_DBG_STRING:
                return std::string(reinterpret_cast<const char*>(d), len);
        }
        return std::string();
    }

    void parseFrame(const uint8_t* data, int len) {
        if (len < 5) return;
        uint8_t type = data[2];
        uint8_t plen = data[3];
        if (4 + plen + 1 > len) return;
        if (crc8(&data[2], plen + 2) != data[4 + plen]) return;

        const uint8_t* p   = &data[4];
        auto           now = std::chrono::steady_clock::now();

        switch (type) {
            case SC_MSG_DEBUG:
                // [robot_id][field_id][value_type][data...]
                if (plen >= 3 && p[0] < SC_MAX_ROBOTS) {
                    uint8_t id = p[0];
                    DebugEntry e;
                    e.robotId = id;
                    e.fieldId = p[1];
                    e.text    = formatDebugValue(p[2], &p[3], plen - 3);
                    e.at      = now;
                    m_debugLog.push_back(std::move(e));
                    if (m_debugLog.size() > SC_DEBUG_LOG_MAX)
                        m_debugLog.erase(m_debugLog.begin(),
                                         m_debugLog.begin() +
                                             (m_debugLog.size() - SC_DEBUG_LOG_MAX));
                    m_robots[id].lastSeen = now;
                }
                break;
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

    // Path to a swarm_hub binary next to the running executable, or "" if
    // the executable's own path can't be determined. Keeps auto-start working
    // regardless of the caller's cwd, since the Makefile puts all tools into
    // the same build directory.
    static std::string siblingHubPath() {
        char buf[4096];
#ifdef __APPLE__
        uint32_t size = sizeof(buf);
        if (_NSGetExecutablePath(buf, &size) != 0) return {};
#else
        ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
        if (n <= 0) return {};
        buf[n] = '\0';
#endif
        std::string path(buf);
        size_t slash = path.rfind('/');
        if (slash == std::string::npos) return {};
        return path.substr(0, slash + 1) + "swarm_hub";
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
