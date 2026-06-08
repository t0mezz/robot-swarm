// swarm_controller.cpp
// ═══════════════════════════════════════════════════════════════
// Swarm Controller — Interactive Robot Control + Test Suite
// ═══════════════════════════════════════════════════════════════
//
// Connects to swarm_hub and provides:
//   - Manual control of individual or all robots
//   - Automated test suite (forward, reverse, spins, round-robin)
//
// Voraussetzung:
//   ./swarm_hub /dev/tty.usbmodem* muss laufen
//
// Aufruf:
//   ./swarm_controller
//
// Steuerung:
//   0-9        Select robot by ID (. = all)
//   Arrows     Throttle / Steer
//   SPACE      Stop selected robot(s)
//   s          Stop ALL robots
//   t          Open test menu
//   q / Ctrl+C Quit

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <glob.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>

#ifdef __APPLE__
#include <CoreGraphics/CoreGraphics.h>
using KeyHandle = CGKeyCode;
static constexpr KeyHandle kKey_A = 0x00;
static constexpr KeyHandle kKey_S = 0x01;
static constexpr KeyHandle kKey_D = 0x02;
static constexpr KeyHandle kKey_W = 0x0D;
static inline bool keyDown(KeyHandle code) {
    return CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, code);
}
#else
#include <X11/Xlib.h>
#include <X11/keysym.h>
using KeyHandle = KeySym;
static constexpr KeyHandle kKey_A = XK_a;
static constexpr KeyHandle kKey_S = XK_s;
static constexpr KeyHandle kKey_D = XK_d;
static constexpr KeyHandle kKey_W = XK_w;
static Display* g_xDisplay = nullptr;
static inline bool keyDown(KeyHandle sym) {
    if (!g_xDisplay) return false;
    KeyCode kc = XKeysymToKeycode(g_xDisplay, sym);
    if (!kc) return false;
    char keys[32] = {};
    XQueryKeymap(g_xDisplay, keys);
    return (keys[kc / 8] >> (kc % 8)) & 1;
}
#endif

// ═══════════════════════════════════════════════════════════════
// Protocol
// ═══════════════════════════════════════════════════════════════

#define MAGIC_0       0xAA
#define MAGIC_1       0x55
#define MSG_SWARM     0x10
#define MSG_ANNOUNCE  0x20
#define MSG_PING      0x22
#define MSG_PONG      0x23
#define MSG_TELEMETRY 0x30
#define MAX_ROBOTS    32

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
        fprintf(stderr, "[auto] Hub is running, waiting for socket...\n");
        for (int i = 0; i < 20; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            fd = hub_connect();
            if (fd >= 0) { fprintf(stderr, "[auto] Connected to swarm_hub.\n"); return fd; }
        }
        fprintf(stderr, "Error: hub is running but socket not ready.\n");
        return -1;
    }

    std::string port = find_serial_port();
    if (port.empty()) {
        fprintf(stderr, "Error: swarm_hub not running and no serial port found.\n"
                        "Start manually: ./swarm_hub /dev/tty.usbmodem*\n");
        return -1;
    }

    fprintf(stderr, "[auto] Starting swarm_hub on %s...\n", port.c_str());
    pid_t pid = fork();
    if (pid == 0) {
        execlp("./swarm_hub", "./swarm_hub", "--daemon", port.c_str(), nullptr);
        execlp("swarm_hub",   "swarm_hub",   "--daemon", port.c_str(), nullptr);
        _exit(1);
    } else if (pid < 0) {
        fprintf(stderr, "Error: fork failed: %s\n", strerror(errno));
        return -1;
    }
    int status; waitpid(pid, &status, 0);

    for (int i = 0; i < 50; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        fd = hub_connect();
        if (fd >= 0) { fprintf(stderr, "[auto] Connected to swarm_hub.\n"); return fd; }
    }
    fprintf(stderr, "Error: swarm_hub did not become ready within 5s.\n");
    return -1;
}

// ═══════════════════════════════════════════════════════════════
// Robot State
// ═══════════════════════════════════════════════════════════════

struct RobotInfo {
    bool    known     = false;
    uint8_t mac[6]    = {};
    uint8_t battery   = 0;
    uint8_t flags     = 0;
    int8_t  motorL    = 0;
    int8_t  motorR    = 0;
    uint16_t latencyUs = 0;
    std::chrono::steady_clock::time_point lastSeen;
};

static RobotInfo     robots[MAX_ROBOTS];
static int8_t        speeds[MAX_ROBOTS][2] = {};   // commanded L/R
static volatile bool g_running = true;

// ═══════════════════════════════════════════════════════════════
// Terminal Raw Mode
// ═══════════════════════════════════════════════════════════════

static struct termios g_oldTermios;

static void rawMode() {
    tcgetattr(STDIN_FILENO, &g_oldTermios);
    struct termios raw = g_oldTermios;
    raw.c_lflag &= ~(ECHO | ICANON | ISIG);
    raw.c_iflag &= ~(IXON | ICRNL);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void normalMode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_oldTermios);
}

// ═══════════════════════════════════════════════════════════════
// Swarm Send
// ═══════════════════════════════════════════════════════════════

static void sendSwarm(int fd) {
    // Only include known robots so the frame stays small.
    // With N active robots the payload is N*3 bytes instead of always MAX_ROBOTS*3.
    // At 115200 baud a full 32-robot frame takes 8.77 ms on the serial link;
    // a single-robot frame takes 0.69 ms — a 12× reduction that cuts both
    // fixed latency and jitter amplification from frame bursts.
    uint8_t payload[MAX_ROBOTS * 3];
    int count = 0;
    for (int i = 0; i < MAX_ROBOTS; i++) {
        if (!robots[i].known) continue;
        payload[count * 3]     = (uint8_t)i;
        payload[count * 3 + 1] = (uint8_t)speeds[i][0];
        payload[count * 3 + 2] = (uint8_t)speeds[i][1];
        count++;
    }
    if (count == 0) return;
    int plen = count * 3;
    uint8_t frame[5 + MAX_ROBOTS * 3];
    buildFrame(frame, MSG_SWARM, payload, (uint8_t)plen);
    write(fd, frame, 5 + plen);
}

static void stopAll() {
    memset(speeds, 0, sizeof(speeds));
}

// ═══════════════════════════════════════════════════════════════
// Incoming Packet Parser
// ═══════════════════════════════════════════════════════════════

static uint8_t rxBuf[1024];
static int     rxLen = 0;

static void parsePacket(const uint8_t* data, int len) {
    if (len < 5) return;
    if (data[0] != MAGIC_0 || data[1] != MAGIC_1) return;
    uint8_t type = data[2];
    uint8_t plen = data[3];
    if (4 + plen + 1 > len) return;
    if (crc8(&data[2], plen + 2) != data[4 + plen]) return;

    const uint8_t* payload = &data[4];
    auto now = std::chrono::steady_clock::now();

    switch (type) {
        case MSG_ANNOUNCE:
            if (plen >= 7) {
                uint8_t id = payload[0];
                if (id < MAX_ROBOTS) {
                    robots[id].known = true;
                    memcpy(robots[id].mac, &payload[1], 6);
                    robots[id].lastSeen = now;
                }
            }
            break;
        case MSG_TELEMETRY:
            if (plen >= 7) {
                uint8_t id = payload[0];
                if (id < MAX_ROBOTS) {
                    robots[id].known   = true;
                    robots[id].battery = payload[1];
                    robots[id].flags   = payload[2];
                    robots[id].motorL  = (int8_t)payload[3];
                    robots[id].motorR  = (int8_t)payload[4];
                    robots[id].lastSeen = now;
                }
            }
            break;
        case MSG_PONG:
            if (plen >= 3) {
                uint8_t id = payload[0];
                if (id < MAX_ROBOTS) {
                    robots[id].latencyUs = payload[1] | (payload[2] << 8);
                    robots[id].lastSeen  = now;
                }
            }
            break;
    }
}

static void readFromHub(int fd) {
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
        if (idx > 0) { memmove(rxBuf, rxBuf + idx, rxLen - idx); rxLen -= idx; }
        if (rxLen < 4) return;
        uint8_t plen = rxBuf[3];
        int     flen = 4 + plen + 1;
        if (rxLen < flen) return;
        parsePacket(rxBuf, flen);
        memmove(rxBuf, rxBuf + flen, rxLen - flen);
        rxLen -= flen;
    }
}

// ═══════════════════════════════════════════════════════════════
// Test Suite
// ═══════════════════════════════════════════════════════════════

struct TestStep {
    const char* name;
    int8_t  motorL;
    int8_t  motorR;
    int     durationMs;
};

static const TestStep TEST_FORWARD    = { "Forward",    80,   80,  3000 };
static const TestStep TEST_REVERSE    = { "Reverse",   -80,  -80,  3000 };
static const TestStep TEST_SPIN_LEFT  = { "Spin Left", -70,   70,  3000 };
static const TestStep TEST_SPIN_RIGHT = { "Spin Right", 70,  -70,  3000 };
static const TestStep TEST_STOP_STEP  = { "Stop",        0,    0,   500 };

struct TestSuite {
    const char*             name;
    std::vector<TestStep>   steps;
};

static const TestSuite SUITES[] = {
    { "All Forward 3s",   { TEST_FORWARD,    TEST_STOP_STEP } },
    { "All Reverse 3s",   { TEST_REVERSE,    TEST_STOP_STEP } },
    { "Spin Left 3s",     { TEST_SPIN_LEFT,  TEST_STOP_STEP } },
    { "Spin Right 3s",    { TEST_SPIN_RIGHT, TEST_STOP_STEP } },
    { "Full Suite",       { TEST_FORWARD, TEST_STOP_STEP,
                            TEST_REVERSE, TEST_STOP_STEP,
                            TEST_SPIN_LEFT, TEST_STOP_STEP,
                            TEST_SPIN_RIGHT, TEST_STOP_STEP } },
};
static const int NUM_SUITES = (int)(sizeof(SUITES) / sizeof(SUITES[0]));

struct TestState {
    bool   running    = false;
    int    suiteIdx   = -1;
    int    stepIdx    = 0;
    std::chrono::steady_clock::time_point stepStart;
    char   status[64] = "Idle";
};

static TestState g_test;

static void applyTestStep(const TestStep& step) {
    for (int i = 0; i < MAX_ROBOTS; i++) {
        if (!robots[i].known) continue;
        speeds[i][0] = step.motorL;
        speeds[i][1] = step.motorR;
    }
}

static void advanceTest() {
    if (!g_test.running) return;
    const TestSuite& suite = SUITES[g_test.suiteIdx];
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - g_test.stepStart).count();

    if (elapsed >= suite.steps[g_test.stepIdx].durationMs) {
        g_test.stepIdx++;
        if (g_test.stepIdx >= (int)suite.steps.size()) {
            // Done
            stopAll();
            g_test.running = false;
            snprintf(g_test.status, sizeof(g_test.status),
                     "Done: %s", suite.name);
            return;
        }
        applyTestStep(suite.steps[g_test.stepIdx]);
        g_test.stepStart = now;
    }

    // Update status
    int remaining = (int)(suite.steps[g_test.stepIdx].durationMs - elapsed);
    snprintf(g_test.status, sizeof(g_test.status),
             "%s — Step %d/%d: %s (%dms left)",
             suite.name,
             g_test.stepIdx + 1, (int)suite.steps.size(),
             suite.steps[g_test.stepIdx].name,
             remaining);
}

static void startTest(int suiteIdx) {
    if (suiteIdx < 0 || suiteIdx >= NUM_SUITES) return;

    // Need at least one known robot
    bool anyKnown = false;
    for (int i = 0; i < MAX_ROBOTS; i++) if (robots[i].known) { anyKnown = true; break; }
    if (!anyKnown) {
        snprintf(g_test.status, sizeof(g_test.status), "No robots known yet");
        return;
    }

    g_test.running  = true;
    g_test.suiteIdx = suiteIdx;
    g_test.stepIdx  = 0;
    g_test.stepStart = std::chrono::steady_clock::now();
    applyTestStep(SUITES[suiteIdx].steps[0]);
    snprintf(g_test.status, sizeof(g_test.status),
             "%s — Step 1/%d: %s",
             SUITES[suiteIdx].name,
             (int)SUITES[suiteIdx].steps.size(),
             SUITES[suiteIdx].steps[0].name);
}

// ═══════════════════════════════════════════════════════════════
// UI
// ═══════════════════════════════════════════════════════════════

static int  g_selectedRobot = -1;   // -1 = all
static bool g_testMenu      = false;

static constexpr int8_t CTRL_SPEED = 30;  // 30% of full range (100)

// Differential-drive WASD polling (macOS only).
// All four keys are sampled simultaneously each loop tick so combined
// inputs (e.g. W+D = forward-right arc) work naturally.
// Mixing:  L = throttle + steer,  R = throttle - steer
//   W alone  →  forward straight
//   S alone  →  reverse straight
//   A alone  →  spin left
//   D alone  →  spin right
//   W+A / W+D  →  arc forward left / right
//   S+A / S+D  →  arc reverse left / right
static int8_t g_wasdL = 0, g_wasdR = 0;  // last commanded values (change-detection)

static void pollWASD(int fd) {
    if (g_test.running) return;

    bool fwd = keyDown(kKey_W);
    bool bwd = keyDown(kKey_S);
    bool lft = keyDown(kKey_A);
    bool rgt = keyDown(kKey_D);

    int8_t throttle = (fwd && !bwd) ? CTRL_SPEED : (!fwd && bwd) ? -CTRL_SPEED : 0;
    int8_t steer    = (rgt && !lft) ? CTRL_SPEED*0.5 : (!rgt && lft) ? -CTRL_SPEED*0.5 : 0;

    int L = (int)throttle + (int)steer;
    int R = (int)throttle - (int)steer;
    if (L >  100) L =  100; else if (L < -100) L = -100;
    if (R >  100) R =  100; else if (R < -100) R = -100;
    int8_t newL = (int8_t)L, newR = (int8_t)R;

    if (newL == g_wasdL && newR == g_wasdR) return;  // nothing changed
    g_wasdL = newL;
    g_wasdR = newR;

    if (g_selectedRobot < 0) {
        for (int i = 0; i < MAX_ROBOTS; i++) { speeds[i][0] = newL; speeds[i][1] = newR; }
    } else {
        speeds[g_selectedRobot][0] = newL;
        speeds[g_selectedRobot][1] = newR;
    }
    sendSwarm(fd);
}

static void stopSelected() {
    if (g_selectedRobot < 0) {
        for (int i = 0; i < MAX_ROBOTS; i++) { speeds[i][0] = 0; speeds[i][1] = 0; }
    } else {
        speeds[g_selectedRobot][0] = 0; speeds[g_selectedRobot][1] = 0;
    }
}

static void drawUI() {
    printf("\033[H\033[J");

    // Header
    printf("\033[1;30m══════════════════════════════════════════════════════════════\033[0m\n");
    printf("\033[1;37m  SWARM CONTROLLER\033[0m");
    if (g_test.running)
        printf("  \033[33m[TEST RUNNING]\033[0m");
    else if (g_selectedRobot < 0)
        printf("  \033[90mTarget: \033[1;37mALL\033[0m");
    else
        printf("  \033[90mTarget: \033[1;33mRobot %d\033[0m", g_selectedRobot);
    printf("\n");
    printf("\033[1;30m═══════════════════════════════════════════════════════════════\033[0m\n\n");

    // Motor output table
    printf("\033[1;37m ID │ Motor L │ Motor R\033[0m\n");
    printf("\033[90m────┼─────────┼─────────\033[0m\n");

    int known = 0;
    for (int i = 0; i < MAX_ROBOTS; i++) {
        if (!robots[i].known) continue;
        known++;
        bool sel = (g_selectedRobot < 0 || g_selectedRobot == i);
        const char* rowColor = sel ? "\033[1;37m" : "\033[37m";
        printf("%s %2d │  %+4d   │  %+4d\033[0m\n",
               rowColor, i, (int)speeds[i][0], (int)speeds[i][1]);
    }
    if (known == 0)
        printf("\033[90m  Waiting for robots...\033[0m\n");

    printf("\n");

    // Test status
    printf("\033[90m────────────────────────────────────────────────────────────────\033[0m\n");
    printf("  Test: \033[%sm%s\033[0m\n",
           g_test.running ? "33" : "90", g_test.status);
    printf("\033[90m────────────────────────────────────────────────────────────────\033[0m\n\n");

    if (g_testMenu) {
        // Test menu overlay
        printf("\033[1;37m  TEST MENU\033[0m\n");
        for (int i = 0; i < NUM_SUITES; i++)
            printf("  \033[33m%d\033[0m  %s\n", i + 1, SUITES[i].name);
        printf("  \033[33mESC\033[0m  Cancel\n\n");
    } else {
        // Controls
        printf("\033[90m  \033[33m0-9\033[90m Select robot   \033[33m.\033[90m All\033[0m\n");
        printf("\033[90m  \033[33mW/S\033[90m Throttle   \033[33mA/D\033[90m Steer   \033[33mW+A/D\033[90m Arc   (diff-drive, multi-key, auto-stop)\033[0m\n");
        printf("\033[90m  \033[33mSPC\033[90m Stop selected   \033[33mESC\033[90m Stop all   \033[33mt\033[90m Tests   \033[33mq\033[90m Quit\033[0m\n");
    }
    fflush(stdout);
}

// ═══════════════════════════════════════════════════════════════
// Input
// ═══════════════════════════════════════════════════════════════

// Returns true if quit requested
static bool handleInput(int fd) {
    uint8_t buf[4];
    ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    if (n <= 0) return false;

    // Skip escape sequences (arrow keys etc.) — not used in WASD mode
    if (n >= 2 && buf[0] == 0x1B && buf[1] == '[') return false;

    uint8_t c = buf[0];

    // ESC alone — stop all
    if (c == 0x1B && n == 1) {
        if (g_testMenu) {
            g_testMenu = false;
        } else if (g_test.running) {
            stopAll(); g_test.running = false;
            snprintf(g_test.status, sizeof(g_test.status), "Aborted");
        } else {
            g_wasdL = g_wasdR = 0; stopAll();
        }
        return false;
    }

    if (g_testMenu) {
        if (c >= '1' && c <= '0' + NUM_SUITES) {
            startTest(c - '1');
            g_testMenu = false;
        } else if (c == 0x1B || c == 'q' || c == 't') {
            g_testMenu = false;
        }
        return false;
    }

    if (g_test.running) {
        if (c == ' ' || c == 0x1B) {
            stopAll(); g_test.running = false;
            snprintf(g_test.status, sizeof(g_test.status), "Aborted");
        }
        return false;
    }

    switch (c) {
        case 'q': case 3: return true;   // q or Ctrl-C

        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
            g_selectedRobot = c - '0';
            break;
        case '.':
            g_selectedRobot = -1;
            break;

        // WASD is handled by pollWASD() via multi-key polling — ignored here.
        case 'w': case 'a': case 's': case 'd': break;

        case ' ':
            g_wasdL = g_wasdR = 0;
            stopSelected();
            sendSwarm(fd);
            break;

        case 't':
            g_testMenu = true;
            break;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════
// Signal
// ═══════════════════════════════════════════════════════════════

void signal_handler(int) { g_running = false; }

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    int drawIntervalMs = 200;  // default 5 fps

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "-r") && i + 1 < argc) {
            int r = std::stoi(argv[++i]);
            if (r > 0) drawIntervalMs = r;
        }
    }

    int fd = hub_connect_or_start();
    if (fd < 0) return 1;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    rawMode();

#ifndef __APPLE__
    g_xDisplay = XOpenDisplay(nullptr);  // nullptr OK — pollWASD checks for nullptr
#endif

    using Clock = std::chrono::steady_clock;
    auto lastSwarm = Clock::now();
    auto lastDraw  = Clock::now();
    auto lastPing  = Clock::now();
    int  pingRobot = 0;

    while (g_running) {
        readFromHub(fd);
        advanceTest();

        if (handleInput(fd)) break;

        pollWASD(fd);

        auto now = Clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSwarm).count() >= 50) {
            sendSwarm(fd);
            lastSwarm = now;
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastDraw).count() >= drawIntervalMs) {
            drawUI();
            lastDraw = now;
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPing).count() >= 200) {
            // Round-robin ping through known robots
            for (int attempt = 0; attempt < MAX_ROBOTS; attempt++) {
                pingRobot = (pingRobot + 1) % MAX_ROBOTS;
                if (robots[pingRobot].known) break;
            }
            if (robots[pingRobot].known) {
                using namespace std::chrono;
                uint32_t ts = (uint32_t)duration_cast<microseconds>(now.time_since_epoch()).count();
                uint8_t payload[5] = {
                    (uint8_t)pingRobot,
                    (uint8_t)ts, (uint8_t)(ts >> 8), (uint8_t)(ts >> 16), (uint8_t)(ts >> 24)
                };
                uint8_t frame[10];
                buildFrame(frame, MSG_PING, payload, 5);
                write(fd, frame, 10);
            }
            lastPing = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    stopAll();
    sendSwarm(fd);
    close(fd);
    normalMode();
#ifndef __APPLE__
    if (g_xDisplay) { XCloseDisplay(g_xDisplay); g_xDisplay = nullptr; }
#endif
    printf("\033[2J\033[H\033[0m");
    printf("Stopped.\n");
    return 0;
}
