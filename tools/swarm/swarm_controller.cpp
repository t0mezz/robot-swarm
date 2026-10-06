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
//   ./swarm_controller [-r ms] [--keys auto|local|terminal]
//
// Keyboard source: WASD normally reads the keyboard attached to this machine
// (evdev / CoreGraphics). Over ssh that is the wrong keyboard, so under ssh
// (SSH_CONNECTION/SSH_TTY set) it reads the terminal's input instead — exact
// press/release on kitty-protocol terminals, a hold timeout elsewhere; see
// terminal_keys.h. --keys local|terminal overrides the detection.
//
// Steuerung:
//   0-9        Select robot by ID (. = all)
//   Arrows     Throttle / Steer
//   SPACE      Stop selected robot(s)
//   s          Stop ALL robots
//   t          Open test menu
//   q / Ctrl+C Quit

#include "SwarmClient.h"
#include "terminal_keys.h"

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
#include <unistd.h>
#include <termios.h>

#ifdef __APPLE__
#include <CoreGraphics/CoreGraphics.h>
using KeyHandle = CGKeyCode;
static constexpr KeyHandle kKey_A = 0x00;
static constexpr KeyHandle kKey_S = 0x01;
static constexpr KeyHandle kKey_D = 0x02;
static constexpr KeyHandle kKey_W = 0x0D;
static inline bool localKeyDown(KeyHandle code) {
    return CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, code);
}
#else
#include "evdev_keys.h"
using KeyHandle = int;
static constexpr KeyHandle kKey_A = KEY_A;
static constexpr KeyHandle kKey_S = KEY_S;
static constexpr KeyHandle kKey_D = KEY_D;
static constexpr KeyHandle kKey_W = KEY_W;
static EvdevKeyboard g_keyboard;
static inline bool localKeyDown(KeyHandle code) {
    return g_keyboard.down(code);
}
static int g_kbDeviceCount = -1;  // # evdev devices opened — surfaced in the debug line below

#endif

// WASD source: the terminal's input stream (ssh) or the local keyboard.
static TerminalKeys g_termKeys;

static inline bool keyDown(KeyHandle code) {
    if (!g_termKeys.active()) return localKeyDown(code);
    char c = code == kKey_W ? 'w' : code == kKey_A ? 'a' : code == kKey_S ? 's' : 'd';
    return g_termKeys.down(c);
}

// ═══════════════════════════════════════════════════════════════
// Robot State
// ═══════════════════════════════════════════════════════════════

static constexpr int MAX_ROBOTS = SC_MAX_ROBOTS;
static SwarmClient    g_swarm;
static int8_t         speeds[MAX_ROBOTS][2] = {};   // commanded L/R
static volatile bool  g_running = true;

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

static void sendSwarm() {
    // Only include known robots so the frame stays small.
    // With N active robots the payload is N*3 bytes instead of always MAX_ROBOTS*3.
    // At 115200 baud a full 32-robot frame takes 8.77 ms on the serial link;
    // a single-robot frame takes 0.69 ms — a 12× reduction that cuts both
    // fixed latency and jitter amplification from frame bursts.
    for (int i = 0; i < MAX_ROBOTS; i++) {
        if (!g_swarm.isKnown(i)) continue;
        g_swarm.setSpeed((uint8_t)i, speeds[i][0], speeds[i][1]);
    }
    g_swarm.flush();
}

static void stopAll() {
    memset(speeds, 0, sizeof(speeds));
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
        if (!g_swarm.isKnown(i)) continue;
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
    for (int i = 0; i < MAX_ROBOTS; i++) if (g_swarm.isKnown(i)) { anyKnown = true; break; }
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

// Differential-drive WASD polling (CoreGraphics on macOS, evdev on Linux).
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

static void pollWASD() {
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
    sendSwarm();
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
        if (!g_swarm.isKnown(i)) continue;
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

    // WASD debug — raw key states straight from keyDown(), so input issues
    // (wrong device grabbed, missing 'input' group, etc.) are visible at a glance.
    {
        bool w = keyDown(kKey_W), a = keyDown(kKey_A), s = keyDown(kKey_S), d = keyDown(kKey_D);
        char kw = w ? 'W' : '_', ka = a ? 'A' : '_', ks = s ? 'S' : '_', kd = d ? 'D' : '_';
        if (g_termKeys.active())
            printf("\033[36m  [wasd debug] ssh terminal keys (%s)   raw keys: %c %c %c %c   →  out L:%+4d R:%+4d\033[0m\n",
                   g_termKeys.kitty() ? "kitty protocol: exact" : "timeout fallback: no W+D arcs",
                   kw, ka, ks, kd, (int)g_wasdL, (int)g_wasdR);
        else
#ifndef __APPLE__
        printf("\033[36m  [wasd debug] kbd devices: %d   raw keys: %c %c %c %c   →  out L:%+4d R:%+4d\033[0m\n",
               g_kbDeviceCount, kw, ka, ks, kd, (int)g_wasdL, (int)g_wasdR);
#else
        printf("\033[36m  [wasd debug] raw keys: %c %c %c %c   →  out L:%+4d R:%+4d\033[0m\n",
               kw, ka, ks, kd, (int)g_wasdL, (int)g_wasdR);
#endif
    }

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

// Next command character, or -1. Terminal-key mode parses stdin itself (and
// consumes WASD into g_termKeys); otherwise only the first byte is looked at.
static int readCommand() {
    if (g_termKeys.active()) return g_termKeys.nextCommand();

    uint8_t buf[4];
    ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    if (n <= 0) return -1;

    // Skip escape sequences (arrow keys etc.) — not used in WASD mode
    if (n >= 2 && buf[0] == 0x1B && buf[1] == '[') return -1;
    return buf[0];
}

// Returns true if quit requested
static bool handleInput() {
    int c = readCommand();
    if (c < 0) return false;

    // ESC alone — stop all
    if (c == 0x1B) {
        if (g_testMenu) {
            g_testMenu = false;
        } else if (g_test.running) {
            stopAll(); g_test.running = false;
            snprintf(g_test.status, sizeof(g_test.status), "Aborted");
        } else {
            g_wasdL = g_wasdR = 0; stopAll(); g_termKeys.releaseAll();
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
            sendSwarm();
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
    std::string keysMode = "auto";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "-r") && i + 1 < argc) {
            int r = std::stoi(argv[++i]);
            if (r > 0) drawIntervalMs = r;
        } else if (arg == "--keys" && i + 1 < argc) {
            keysMode = argv[++i];
            if (keysMode != "auto" && keysMode != "local" && keysMode != "terminal") {
                fprintf(stderr, "--keys takes auto, local or terminal\n");
                return 2;
            }
        }
    }

    if (!g_swarm.connect()) return 1;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGHUP,  signal_handler);   // ssh dropped: stop the robots rather than hold the last command

    rawMode();

    bool viaSsh = getenv("SSH_CONNECTION") || getenv("SSH_TTY");
    bool useTerminalKeys = keysMode == "terminal" || (keysMode == "auto" && viaSsh);
    if (useTerminalKeys) g_termKeys.begin();

#ifndef __APPLE__
    if (!useTerminalKeys) g_kbDeviceCount = g_keyboard.open();
    if (!useTerminalKeys && g_kbDeviceCount == 0)
        fprintf(stderr, "[wasd] no readable keyboard in /dev/input — WASD disabled. "
                        "Add yourself to the 'input' group (sudo usermod -aG input $USER, "
                        "then log out and back in).\n");
#endif

    using Clock = std::chrono::steady_clock;
    auto lastSwarm = Clock::now();
    auto lastDraw  = Clock::now();

    while (g_running) {
        g_swarm.poll();  // also drives the round-robin auto-ping (see SwarmClient::poll)
        advanceTest();

        if (handleInput()) break;

        pollWASD();

        auto now = Clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSwarm).count() >= 50) {
            sendSwarm();
            lastSwarm = now;
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastDraw).count() >= drawIntervalMs) {
            drawUI();
            lastDraw = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    stopAll();
    sendSwarm();
    g_swarm.disconnect();
    g_termKeys.end();
    normalMode();
#ifndef __APPLE__
    g_keyboard.close();
#endif
    printf("\033[2J\033[H\033[0m");
    printf("Stopped.\n");
    return 0;
}
