// test_mjpeg_server.cpp
// Tests for lib/ArucoTracker/mjpeg_server.h — the loopback MJPEG server behind
// vision_hub's stream. Real TCP sockets in this process, no camera, no OpenCV:
// the "JPEG" is arbitrary bytes, since the server never looks inside it.
//
// Plain asserts with a pass/fail tally, run via `make test`.

#include "../lib/ArucoTracker/mjpeg_server.h"

#include <chrono>
#include <cstdio>
#include <thread>

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_TRUE(cond, msg) \
    do { if (cond) g_pass++; else { g_fail++; std::printf("FAIL %s: %s\n", __func__, msg); } } while (0)

static const int PORT = 38417;   // arbitrary, loopback only

static int dial() {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port        = htons(PORT);
    if (::connect(fd, (sockaddr*)&a, sizeof(a)) < 0) { ::close(fd); return -1; }
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    return fd;
}

static void get(int fd, const char* path) {
    std::string r = std::string("GET ") + path + " HTTP/1.1\r\nHost: x\r\n\r\n";
    (void)!::write(fd, r.data(), r.size());
}

// Pumps the server and reads from `fd` until `needle` shows up (or ~1 s).
static bool readUntil(MjpegServer& s, int fd, std::string& got, const std::string& needle) {
    for (int i = 0; i < 1000; i++) {
        s.poll();
        char b[4096];
        ssize_t n;
        while ((n = ::read(fd, b, sizeof(b))) > 0) got.append(b, (size_t)n);
        if (got.find(needle) != std::string::npos) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

static void pump(MjpegServer& s, int ms) {
    for (int i = 0; i < ms; i++) { s.poll(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}

static void test_stream_delivers_frames() {
    MjpegServer s;
    EXPECT_TRUE(s.start(PORT), "server starts");
    int fd = dial();
    get(fd, "/stream");
    std::string got;
    EXPECT_TRUE(readUntil(s, fd, got, "boundary=frame"), "stream headers arrive");
    EXPECT_TRUE(s.viewers() == 1 && s.wantsFrames(), "one viewer counted");

    const uint8_t jpg[] = {0xFF, 0xD8, 1, 2, 3, 0xFF, 0xD9};
    s.publish(jpg, sizeof(jpg));
    EXPECT_TRUE(readUntil(s, fd, got, "Content-Length: 7"), "frame part arrives with its length");
    EXPECT_TRUE(got.find(std::string((const char*)jpg, sizeof(jpg))) != std::string::npos, "payload intact");
    ::close(fd);
}

static void test_viewer_count_follows_disconnect() {
    MjpegServer s;
    s.start(PORT);
    int fd = dial();
    get(fd, "/stream");
    std::string got;
    readUntil(s, fd, got, "boundary=frame");
    EXPECT_TRUE(s.viewers() == 1, "attached");
    ::close(fd);
    pump(s, 50);
    EXPECT_TRUE(s.viewers() == 0, "viewer gone once the socket closes");
}

static void test_slow_viewer_does_not_queue() {
    MjpegServer s;
    s.start(PORT);
    int fd = dial();
    get(fd, "/stream");
    std::string got;
    readUntil(s, fd, got, "boundary=frame");

    // Never read again. A 4 MB frame cannot fit the socket buffers, so the
    // first one stalls mid-write and every later publish() must be dropped
    // rather than appended behind it.
    std::vector<uint8_t> big(4u << 20, 0xAB);
    for (int i = 0; i < 20; i++) { s.publish(big.data(), big.size()); s.poll(); }
    EXPECT_TRUE(s.viewers() == 1, "slow viewer stays attached");

    // Drain: the total must be about one frame, not twenty.
    size_t total = 0;
    for (int i = 0; i < 2000; i++) {
        s.poll();
        char b[65536];
        ssize_t n;
        while ((n = ::read(fd, b, sizeof(b))) > 0) total += (size_t)n;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(total < 3 * big.size(), "no backlog built up behind a slow viewer");
    ::close(fd);
}

static void test_snapshot_and_page() {
    MjpegServer s;
    s.start(PORT);

    // A snapshot request is demand for a frame, and waits for the next one.
    int fd = dial();
    get(fd, "/snapshot.jpg");
    pump(s, 20);
    EXPECT_TRUE(s.wantsFrames(), "a waiting snapshot asks for frames");
    EXPECT_TRUE(s.viewers() == 0, "but it is not a stream viewer");

    const uint8_t jpg[] = {0xFF, 0xD8, 9, 9, 0xFF, 0xD9};
    s.publish(jpg, sizeof(jpg));
    std::string got;
    EXPECT_TRUE(readUntil(s, fd, got, "image/jpeg"), "snapshot answered by the next frame");
    EXPECT_TRUE(got.find(std::string((const char*)jpg, sizeof(jpg))) != std::string::npos, "payload intact");
    ::close(fd);
    pump(s, 20);
    EXPECT_TRUE(!s.wantsFrames(), "demand ends once it is answered");

    fd = dial();
    get(fd, "/");
    got.clear();
    EXPECT_TRUE(readUntil(s, fd, got, "/stream"), "index page references the stream");
    EXPECT_TRUE(!s.wantsFrames(), "the page alone asks for nothing");
    ::close(fd);
}

static void test_publish_without_viewers_is_harmless() {
    MjpegServer s;
    s.start(PORT);
    const uint8_t jpg[] = {1, 2, 3};
    for (int i = 0; i < 100; i++) { s.publish(jpg, sizeof(jpg)); s.poll(); }
    EXPECT_TRUE(s.isRunning() && s.viewers() == 0, "nobody watching, nothing breaks");
}

static void test_port_in_use_is_refused() {
    MjpegServer a, b;
    EXPECT_TRUE(a.start(PORT), "first binds");
    EXPECT_TRUE(!b.start(PORT), "second is refused");
}

int main() {
    test_stream_delivers_frames();
    test_viewer_count_follows_disconnect();
    test_slow_viewer_does_not_queue();
    test_snapshot_and_page();
    test_publish_without_viewers_is_harmless();
    test_port_in_use_is_refused();
    std::printf("\ntest_mjpeg_server: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
