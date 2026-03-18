// swarm_hub.cpp
// ═══════════════════════════════════════════════════════════════
// Swarm Hub — Pure Serial ↔ Socket Bridge
// ═══════════════════════════════════════════════════════════════
//
// Owns the serial port exclusively (TIOCEXCL).
// Forwards complete frames: serial → all socket clients.
// Forwards raw bytes: any socket client → serial.
//
// No protocol logic, no periodic sends, no robot tracking.
// The controller is responsible for MSG_SWARM and MSG_PING.
//
// Kompilieren (macOS):
//   g++ swarm_hub.cpp -o swarm_hub -std=c++17 -framework IOKit
//
// Kompilieren (Linux):
//   g++ swarm_hub.cpp -o swarm_hub -std=c++17
//
// Aufruf:
//   ./swarm_hub [--daemon] <port>
//   ./swarm_hub --stop

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <csignal>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

// ═══════════════════════════════════════════════════════════════
// Konstanten
// ═══════════════════════════════════════════════════════════════

#define HUB_SOCK_PATH  "/tmp/swarm_hub.sock"
#define HUB_PID_PATH   "/tmp/swarm_hub.pid"
#define HUB_LOG_PATH   "/tmp/swarm_hub.log"
#define MAX_CLIENTS    16

// Frame magic — only needed for serial→client frame extraction
#define MAGIC_0  0xAA
#define MAGIC_1  0x55

// ═══════════════════════════════════════════════════════════════
// Frame helpers (used only for serial→client extraction)
// ═══════════════════════════════════════════════════════════════

static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

static int frameSize(uint8_t plen) { return 4 + plen + 1; }

static bool validateFrame(const uint8_t* data, int len) {
    if (len < 5) return false;
    if (data[0] != MAGIC_0 || data[1] != MAGIC_1) return false;
    uint8_t plen = data[3];
    if (4 + plen + 1 > len) return false;
    return crc8(&data[2], plen + 2) == data[4 + plen];
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
    // Exclusive lock — prevents a second swarm_hub from opening the same port
    if (ioctl(fd, TIOCEXCL) < 0) {
        std::cerr << "Cannot lock " << path << " exclusively: " << strerror(errno)
                  << "\n(Another swarm_hub may already be running)\n";
        close(fd);
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
    tty.c_cc[VMIN]  = 0;
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
// Hub State
// ═══════════════════════════════════════════════════════════════

struct ClientConn {
    int     fd    = -1;
    bool    active = false;
};

static ClientConn    clients[MAX_CLIENTS];
static int           g_serialFd = -1;
static volatile bool g_running  = true;

void signal_handler(int) { g_running = false; }

// ═══════════════════════════════════════════════════════════════
// Client Management
// ═══════════════════════════════════════════════════════════════

static void client_close(int slot) {
    if (clients[slot].active) {
        close(clients[slot].fd);
        clients[slot].active = false;
        clients[slot].fd     = -1;
        std::cout << "[hub] Client disconnected (slot " << slot << ")\n";
    }
}

static void broadcast_to_clients(const uint8_t* frame, int len) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients[i].active) continue;
        ssize_t n = write(clients[i].fd, frame, len);
        if (n < 0 && (errno == EPIPE || errno == EWOULDBLOCK || errno == EAGAIN)) {
            client_close(i);
        }
    }
}

static void client_accept(int serverFd) {
    int cfd = accept(serverFd, nullptr, nullptr);
    if (cfd < 0) return;

    int slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients[i].active) { slot = i; break; }
    }
    if (slot < 0) {
        std::cerr << "[hub] Max clients reached, rejecting\n";
        close(cfd);
        return;
    }

    int flags = fcntl(cfd, F_GETFL, 0);
    fcntl(cfd, F_SETFL, flags | O_NONBLOCK);

    clients[slot].fd     = cfd;
    clients[slot].active = true;
    std::cout << "[hub] Client connected (slot " << slot << ")\n";
}

// ═══════════════════════════════════════════════════════════════
// Serial → Clients (frame-complete forwarding)
// ═══════════════════════════════════════════════════════════════

static uint8_t serialRxBuf[1024];
static int     serialRxLen = 0;

static void serial_read_and_broadcast(int serialFd) {
    uint8_t tmp[512];
    ssize_t n = read(serialFd, tmp, sizeof(tmp));
    if (n <= 0) return;

    int space = (int)sizeof(serialRxBuf) - serialRxLen;
    int copy  = (n < space) ? (int)n : space;
    memcpy(&serialRxBuf[serialRxLen], tmp, copy);
    serialRxLen += copy;

    while (serialRxLen >= 5) {
        // Find magic header
        int idx = -1;
        for (int i = 0; i < serialRxLen - 1; i++) {
            if (serialRxBuf[i] == MAGIC_0 && serialRxBuf[i+1] == MAGIC_1) { idx = i; break; }
        }
        if (idx < 0) { serialRxLen = 0; return; }
        if (idx > 0) { memmove(serialRxBuf, serialRxBuf + idx, serialRxLen - idx); serialRxLen -= idx; }
        if (serialRxLen < 4) return;

        uint8_t plen = serialRxBuf[3];
        int     flen = frameSize(plen);
        if (serialRxLen < flen) return;

        if (validateFrame(serialRxBuf, flen)) {
            broadcast_to_clients(serialRxBuf, flen);
        }

        memmove(serialRxBuf, serialRxBuf + flen, serialRxLen - flen);
        serialRxLen -= flen;
    }
}

// ═══════════════════════════════════════════════════════════════
// Clients → Serial (raw pass-through)
// ═══════════════════════════════════════════════════════════════

static void process_client_data(int slot) {
    uint8_t tmp[256];
    ssize_t n = read(clients[slot].fd, tmp, sizeof(tmp));
    if (n <= 0) {
        client_close(slot);
        return;
    }
    // Non-blocking write: at 115200 baud the serial TX kernel buffer (~4 KB) is never
    // full at our packet rate (~2 kB/s), so EAGAIN won't occur in practice.
    // Keeping O_NONBLOCK avoids stalling the poll loop and blocking the dongle's loop()
    // by starving the hub's serial reads (which would cause Serial.write() on the dongle
    // to block when its USB CDC TX buffer fills).
    ssize_t written = write(g_serialFd, tmp, n);
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        std::cerr << "[hub] Serial write error: " << strerror(errno) << "\n";
    }
}

// ═══════════════════════════════════════════════════════════════
// Server Socket
// ═══════════════════════════════════════════════════════════════

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
// Daemon Helpers
// ═══════════════════════════════════════════════════════════════

static void daemonize() {
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); exit(1); }
    if (pid > 0) {
        std::ofstream pidFile(HUB_PID_PATH);
        if (pidFile) pidFile << pid << "\n";
        std::cout << "[hub] Daemon started (PID " << pid << ")\n";
        std::cout << "[hub] Log: " << HUB_LOG_PATH << "\n";
        exit(0);
    }
    setsid();
    int logFd = open(HUB_LOG_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (logFd >= 0) { dup2(logFd, STDOUT_FILENO); dup2(logFd, STDERR_FILENO); close(logFd); }
    int nullFd = open("/dev/null", O_RDONLY);
    if (nullFd >= 0) { dup2(nullFd, STDIN_FILENO); close(nullFd); }
}

static int hub_stop() {
    std::ifstream pidFile(HUB_PID_PATH);
    if (!pidFile) {
        // No PID file — hub may have been started in foreground without --daemon.
        // Fall back to killing any process named swarm_hub.
        std::cout << "No PID file — trying pkill swarm_hub...\n";
        int r = system("pkill swarm_hub 2>/dev/null");
        if (r == 0) {
            std::cout << "Sent SIGTERM via pkill, waiting...\n";
            usleep(500000);  // 500ms is enough for foreground process to exit
        } else {
            std::cerr << "No swarm_hub process found.\n";
        }
        unlink(HUB_SOCK_PATH);
        return 0;
    }
    pid_t pid; pidFile >> pid;
    if (pid <= 0) { std::cerr << "Invalid PID in " << HUB_PID_PATH << "\n"; return 1; }

    if (kill(pid, SIGTERM) < 0) {
        if (errno == ESRCH) {
            std::cerr << "swarm_hub (PID " << pid << ") not running — stale PID file removed.\n";
            unlink(HUB_PID_PATH);
            return 0;
        }
        perror("kill"); return 1;
    }
    std::cout << "Sent SIGTERM to swarm_hub (PID " << pid << "), waiting..." << std::flush;
    for (int i = 0; i < 20; i++) {
        usleep(100000);
        if (kill(pid, 0) != 0) goto done;
    }
    // Still running after 2s — escalate to SIGKILL
    std::cout << " escalating to SIGKILL..." << std::flush;
    kill(pid, SIGKILL);
    for (int i = 0; i < 10; i++) {
        usleep(100000);
        if (kill(pid, 0) != 0) break;
    }
done:
    std::cout << " done.\n";
    unlink(HUB_SOCK_PATH);
    unlink(HUB_PID_PATH);
    return 0;
}

// ═══════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    bool        daemon_mode = false;
    bool        stop_mode   = false;
    std::string port;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--daemon" || arg == "-d") daemon_mode = true;
        else if (arg == "--stop")             stop_mode   = true;
        else                                  port        = arg;
    }

    if (stop_mode) return hub_stop();

    if (port.empty()) {
        std::cerr << "Usage: " << argv[0] << " [--daemon|-d] <port>\n";
        std::cerr << "       " << argv[0] << " --stop\n";
        return 1;
    }

    if (daemon_mode) {
        daemonize();
        std::cout << std::unitbuf;  // flush every line — required when stdout is redirected to log file
    }

    int serialFd = serial_open(port, 115200);
    if (serialFd < 0) return 1;
    g_serialFd = serialFd;

    int serverFd = hub_server_create();
    if (serverFd < 0) { close(serialFd); return 1; }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);

    std::cout << "═══ Swarm Hub ═══\n";
    std::cout << "Serial : " << port << "\n";
    std::cout << "Socket : " << HUB_SOCK_PATH << "\n";
    std::cout << "Ready.\n\n";

    while (g_running) {
        struct pollfd fds[2 + MAX_CLIENTS];
        int           slotOf[2 + MAX_CLIENTS];
        int           nfds = 0;

        fds[nfds] = { serialFd, POLLIN, 0 }; slotOf[nfds] = -1; nfds++;
        fds[nfds] = { serverFd, POLLIN, 0 }; slotOf[nfds] = -2; nfds++;

        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].active) {
                fds[nfds]    = { clients[i].fd, POLLIN, 0 };
                slotOf[nfds] = i;
                nfds++;
            }
        }

        poll(fds, nfds, -1);  // block until event — no artificial delay

        if (fds[0].revents & (POLLHUP | POLLERR)) {
            std::cerr << "[hub] Serial port disconnected — exiting\n";
            break;
        }

        for (int i = 0; i < nfds; i++) {
            if (!(fds[i].revents & POLLIN)) continue;
            if      (slotOf[i] == -1) serial_read_and_broadcast(serialFd);
            else if (slotOf[i] == -2) client_accept(serverFd);
            else                      process_client_data(slotOf[i]);
        }
    }

    for (int i = 0; i < MAX_CLIENTS; i++) client_close(i);
    close(serverFd);
    close(serialFd);
    unlink(HUB_SOCK_PATH);
    unlink(HUB_PID_PATH);

    std::cout << "\n[hub] Stopped.\n";
    return 0;
}
