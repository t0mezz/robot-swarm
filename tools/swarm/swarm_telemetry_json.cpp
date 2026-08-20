// swarm_telemetry_json.cpp
// ═══════════════════════════════════════════════════════════════
// Swarm telemetry → newline-delimited JSON on stdout
// ═══════════════════════════════════════════════════════════════
//
// Headless producer for UIs that aren't C++. Owns exactly the two data
// sources swarm_dashboard owns — the dongle (via swarm_hub/SwarmClient) and
// the Basler camera (via a headless ArucoTracker) — and writes one JSON
// object per tick to stdout, so a renderer in any language can consume the
// swarm without reimplementing the wire protocol (which is already
// hand-mirrored in three places, see lib/SwarmProtocol/protocol.h).
//
// Deliberately stateless beyond the current tick: no rolling history, no
// derived statistics. Consumers keep their own windows, which is why the
// tick interval is reported in the hello line.
//
// Sends no motor commands — pure observer, safe to run alongside a demo.
//
// Usage:
//   ./swarm_telemetry_json [--interval MS] [--no-vision]
//
// stdout is NDJSON, one object per line; all diagnostics go to stderr.
//   {"type":"hello","intervalMs":250,"maxRobots":32,...}
//   {"type":"tick","t":...,"hub":true,"robots":[...],"vision":{...},"log":[...]}

#include "SwarmClient.h"
#include "aruco_tracker.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

static volatile sig_atomic_t running = 1;
static void signal_handler(int) { running = 0; }

// ── JSON helpers ─────────────────────────────────────────────────
// Minimal by design: every value written here is a number, a bool, or a
// short robot-authored debug string, so a full JSON library would be all
// dependency and no benefit.

// Escapes per RFC 8259. Debug text arrives from the robots as raw bytes and
// may legitimately contain quotes or control characters, so this also
// escapes anything below 0x20 and drops bytes >= 0x7F (which would otherwise
// emit invalid UTF-8 and break the consumer's line parse).
static std::string jsonEscape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (unsigned char c : in) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else if (c < 0x7F) {
                    out += (char)c;
                }
                break;
        }
    }
    return out;
}

static void appendf(std::string& out, const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    out += buf;
}

static std::string macToString(const uint8_t* mac) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return buf;
}

// One atomic write per line: a partially written line would desync the
// consumer's newline-delimited parse, and stdout may be a pipe.
static void emitLine(std::string& line) {
    line += '\n';
    (void)!::write(STDOUT_FILENO, line.data(), line.size());
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    int  intervalMs = 250;
    bool wantVision = true;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--interval" && i + 1 < argc) {
            intervalMs = std::max(20, atoi(argv[++i]));
        } else if (a == "--no-vision") {
            wantVision = false;
        } else if (a == "--help" || a == "-h") {
            fprintf(stderr, "usage: %s [--interval MS] [--no-vision]\n", argv[0]);
            return 0;
        } else {
            fprintf(stderr, "unknown argument: %s\n", a.c_str());
            return 2;
        }
    }

    SwarmClient swarm;
    bool hubConnected = swarm.connect();
    fprintf(stderr, hubConnected ? "[hub] connected\n"
                                 : "[hub] unavailable, retrying in background\n");

    ArucoTracker tracker(ArucoConfig::fromFile());
    bool visionOk = wantVision && tracker.open();
    fprintf(stderr, visionOk ? "[vision] camera open\n" : "[vision] no camera\n");

    // Unlike swarm_dashboard this does NOT bail out when both sources are
    // missing: the consumer is a long-lived UI that should render an honest
    // "waiting" state and pick both up as they appear, rather than die at
    // startup because the dongle wasn't plugged in yet.
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);  // consumer exiting first must not kill us mid-write

    {
        std::string hello;
        appendf(hello, "{\"type\":\"hello\",\"intervalMs\":%d,\"maxRobots\":%d,"
                       "\"visionRequested\":%s,\"latencyMaxUs\":5000,"
                       "\"batteryMaxV\":6.5,\"motorMax\":127}",
                intervalMs, SC_MAX_ROBOTS, wantVision ? "true" : "false");
        emitLine(hello);
    }

    auto lastTick     = std::chrono::steady_clock::now();
    auto lastHubRetry = lastTick - std::chrono::seconds(10);

    while (running) {
        if (!swarm.isConnected()) {
            auto t = std::chrono::steady_clock::now();
            if (std::chrono::duration<float>(t - lastHubRetry).count() >= 2.0f) {
                lastHubRetry = t;
                // false = adopt an already-running hub only. The auto-launch
                // path scans serial ports and logs on failure; retrying that
                // every 2s would spam stderr while no dongle is plugged in.
                swarm.connect(false);
            }
        }
        swarm.poll();
        if (visionOk) tracker.update();

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTick).count() < intervalMs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        lastTick = now;

        std::string line;
        line.reserve(4096);

        auto wallMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch()).count();
        appendf(line, "{\"type\":\"tick\",\"t\":%lld,\"hub\":%s,\"robots\":[",
                (long long)wallMs, swarm.isConnected() ? "true" : "false");

        bool first = true;
        for (uint8_t id : swarm.knownIds()) {
            const auto& r = swarm.robotState(id);
            auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - r.lastSeen).count();
            if (!first) line += ',';
            first = false;
            appendf(line, "{\"id\":%u,\"latencyUs\":%u,\"motorL\":%d,\"motorR\":%d,"
                          "\"uptime\":%u,\"ageMs\":%lld,\"flags\":%u,"
                          "\"hasTelemetry\":%s,\"mac\":\"%s\",",
                    id, r.latencyUs, (int)r.motorL, (int)r.motorR,
                    r.uptime, (long long)ageMs, r.flags,
                    r.hasTelemetry ? "true" : "false", macToString(r.mac).c_str());
            // Without STATUS_BAT_VALID the robot's ESP32 has never received a
            // metrics frame from the RP2040, so the battery byte is not a
            // reading of 0V — it's the absence of one. Emit null, never 0.
            if (r.flags & SC_STATUS_BAT_VALID)
                appendf(line, "\"batteryV\":%.3f}", scBatteryVolts(r.battery));
            else
                line += "\"batteryV\":null}";
        }

        appendf(line, "],\"vision\":{\"ok\":%s", visionOk ? "true" : "false");
        if (visionOk) {
            auto sz = tracker.frameSize();
            appendf(line, ",\"fps\":%.1f,\"w\":%d,\"h\":%d,\"robots\":[",
                    tracker.detectionFps(), sz.width, sz.height);
            first = true;
            for (const auto& p : tracker.robots()) {
                if (!first) line += ',';
                first = false;
                appendf(line, "{\"id\":%d,\"x\":%.1f,\"y\":%.1f,\"yaw\":%.1f,"
                              "\"px\":%.1f,\"py\":%.1f}",
                        p.id, p.x, p.y, p.yaw, p.px, p.py);
            }
            line += ']';
        }
        line += "},\"log\":[";

        // Drained rather than replayed: this process is the log's only
        // consumer, so emitting an entry once and clearing keeps each tick
        // carrying strictly the lines that arrived since the last one.
        first = true;
        for (const auto& e : swarm.debugLog()) {
            auto ageMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - e.at).count();
            if (!first) line += ',';
            first = false;
            appendf(line, "{\"id\":%u,\"field\":%u,\"ageMs\":%lld,\"text\":\"%s\"}",
                    e.robotId, e.fieldId, (long long)ageMs,
                    jsonEscape(e.text).c_str());
        }
        swarm.clearDebugLog();
        line += "]}";

        emitLine(line);
    }

    fprintf(stderr, "[exit] stopped\n");
    return 0;
}
