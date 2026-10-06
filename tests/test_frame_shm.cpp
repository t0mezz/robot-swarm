// test_frame_shm.cpp
// Tests for lib/ArucoTracker/frame_shm.h — the shared-memory frame ring behind
// vision_hub. Writer and reader live in this process but map the segment
// independently (separate shm_open + mmap), so they exercise the same shared
// layout two processes would. No camera and no OpenCV.
//
// Uses a private segment name so a running vision_hub is never disturbed.
//
// Plain asserts with a pass/fail tally, run via `make test`.

#include "../lib/ArucoTracker/frame_shm.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_TRUE(cond, msg) \
    do { if (cond) g_pass++; else { g_fail++; std::printf("FAIL %s: %s\n", __func__, msg); } } while (0)

static const char* NAME = "/swarm_vision_frames_test";

static std::vector<uint8_t> filled(size_t n, uint8_t v) { return std::vector<uint8_t>(n, v); }

static void test_no_segment_no_reader() {
    shm_unlink(NAME);
    ShmFrameReader r;
    EXPECT_TRUE(!r.open(NAME), "nothing to open before a writer exists");
    EXPECT_TRUE(!r.isOpen(), "reader stays closed");
}

static void test_round_trip() {
    ShmFrameWriter w;
    EXPECT_TRUE(w.create(64, 48, 3, NAME), "writer creates");
    ShmFrameReader r;
    EXPECT_TRUE(r.open(NAME), "reader maps it");
    EXPECT_TRUE(r.width() == 64 && r.height() == 48 && r.channels() == 3, "geometry travels with the segment");
    EXPECT_TRUE(r.frameBytes() == 64u * 48 * 3, "frame size");
    EXPECT_TRUE(r.latestId() == 0, "no frame before the first write");

    std::vector<uint8_t> dst(r.frameBytes());
    EXPECT_TRUE(r.copyLatest(dst.data()) == 0, "nothing to copy yet");

    auto a = filled(64 * 48 * 3, 0x11);
    uint64_t id1 = w.write(a.data(), 64, 48, 3, 64 * 3);
    EXPECT_TRUE(id1 == 1, "first frame is id 1");
    EXPECT_TRUE(r.copyLatest(dst.data()) == 1 && dst == a, "reader gets the frame, intact");

    auto b = filled(64 * 48 * 3, 0x22);
    w.write(b.data(), 64, 48, 3, 64 * 3);
    EXPECT_TRUE(r.copyLatest(dst.data()) == 2 && dst == b, "and then the newer one");

    for (int i = 0; i < 10; i++) w.write(a.data(), 64, 48, 3, 64 * 3);   // wraps the three slots
    EXPECT_TRUE(r.copyLatest(dst.data()) == 12 && dst == a, "ring wraps cleanly");
}

static void test_strided_source() {
    ShmFrameWriter w;
    w.create(8, 4, 3, NAME);
    ShmFrameReader r;
    r.open(NAME);

    // A source with row padding (a cropped cv::Mat looks like this).
    const size_t stride = 8 * 3 + 5;
    std::vector<uint8_t> src(stride * 4, 0xEE);
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 8 * 3; x++) src[y * stride + x] = (uint8_t)(y * 40 + x);
    w.write(src.data(), 8, 4, 3, stride);

    std::vector<uint8_t> dst(r.frameBytes());
    EXPECT_TRUE(r.copyLatest(dst.data()) == 1, "copied");
    bool ok = true;
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 8 * 3; x++) ok &= dst[y * 8 * 3 + x] == (uint8_t)(y * 40 + x);
    EXPECT_TRUE(ok, "padding is dropped, rows are packed");
}

static void test_wrong_geometry_is_refused() {
    ShmFrameWriter w;
    w.create(16, 16, 3, NAME);
    auto f = filled(32 * 32 * 3, 1);
    EXPECT_TRUE(w.write(f.data(), 32, 32, 3, 32 * 3) == 0, "a different size is not squeezed in");
    EXPECT_TRUE(w.write(f.data(), 16, 16, 1, 16) == 0, "nor a different channel count");
    EXPECT_TRUE(!ShmFrameWriter().create(0, 10, 3, NAME), "empty geometry rejected");
}

static void test_demand_heartbeat() {
    ShmFrameWriter w;
    w.create(8, 8, 3, NAME);
    ShmFrameReader r;
    r.open(NAME);

    EXPECT_TRUE(!w.wanted(), "nobody has asked yet, so nothing needs copying");
    r.touch();
    EXPECT_TRUE(w.wanted(), "a touch is demand");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    EXPECT_TRUE(!w.wanted(20), "a stale heartbeat expires — a dead reader leaks nothing");
    EXPECT_TRUE(w.wanted(1000), "…but still counts within a longer window");
}

static void test_writer_exit_removes_segment() {
    {
        ShmFrameWriter w;
        w.create(8, 8, 3, NAME);
        ShmFrameReader r;
        EXPECT_TRUE(r.open(NAME), "segment exists while the writer lives");
    }
    ShmFrameReader r2;
    EXPECT_TRUE(!r2.open(NAME), "and is gone when it exits");
}

static void test_reader_survives_writer_exit() {
    auto w = std::make_unique<ShmFrameWriter>();
    w->create(8, 8, 3, NAME);
    auto f = filled(8 * 8 * 3, 0x5A);
    w->write(f.data(), 8, 8, 3, 8 * 3);
    ShmFrameReader r;
    r.open(NAME);
    w.reset();   // writer exits, segment unlinked

    std::vector<uint8_t> dst(r.frameBytes());
    EXPECT_TRUE(r.copyLatest(dst.data()) == 1 && dst == f, "an old mapping stays valid, it just stops advancing");
}

static void test_rewritten_segment_is_a_new_one() {
    ShmFrameWriter w1;
    w1.create(8, 8, 3, NAME);
    ShmFrameReader old;
    old.open(NAME);

    ShmFrameWriter w2;   // a restarted hub: replaces, never truncates
    EXPECT_TRUE(w2.create(16, 16, 3, NAME), "replacement created");
    auto f = filled(16 * 16 * 3, 9);
    w2.write(f.data(), 16, 16, 3, 16 * 3);

    EXPECT_TRUE(old.width() == 8, "the old reader still sees its old (valid) mapping");
    ShmFrameReader fresh;
    EXPECT_TRUE(fresh.open(NAME) && fresh.width() == 16, "a re-opened reader gets the new one");
    std::vector<uint8_t> dst(fresh.frameBytes());
    EXPECT_TRUE(fresh.copyLatest(dst.data()) == 1 && dst == f, "with its frames");
    w1.destroy();   // the old writer exiting must not unlink its successor's segment
    ShmFrameReader after;
    EXPECT_TRUE(after.open(NAME) && after.width() == 16, "the replacement outlives the old writer");
}

// The property the seqlock exists for: a reader copying while the writer is
// flat out must only ever get whole frames. Every frame is one repeated byte, so
// a torn copy shows up as a mixed buffer.
static void test_no_torn_frames_under_load() {
    const uint32_t W = 256, H = 256, C = 3;
    ShmFrameWriter w;
    w.create(W, H, C, NAME);
    ShmFrameReader r;
    r.open(NAME);

    std::atomic<bool> stop{false};
    std::thread writer([&] {
        std::vector<uint8_t> f(W * H * C);
        uint8_t v = 0;
        while (!stop) {
            std::memset(f.data(), ++v, f.size());
            w.write(f.data(), W, H, C, W * C);
        }
    });

    std::vector<uint8_t> dst(r.frameBytes());
    long good = 0, torn = 0, missed = 0;
    uint64_t lastId = 0;
    bool monotonic = true;
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < end) {
        uint64_t id = r.copyLatest(dst.data());
        if (!id) { missed++; continue; }
        bool uniform = true;
        for (size_t i = 1; i < dst.size(); i++) if (dst[i] != dst[0]) { uniform = false; break; }
        (uniform ? good : torn)++;
        if (id < lastId) monotonic = false;
        lastId = id;
    }
    stop = true;
    writer.join();

    std::printf("      (%ld whole frames read, %ld retries exhausted, %lu written)\n", good, missed, (unsigned long)lastId);
    EXPECT_TRUE(good > 0, "the reader got frames while the writer ran flat out");
    EXPECT_TRUE(torn == 0, "never a torn frame");
    EXPECT_TRUE(monotonic, "ids never go backwards");
}

int main() {
    test_no_segment_no_reader();
    test_round_trip();
    test_strided_source();
    test_wrong_geometry_is_refused();
    test_demand_heartbeat();
    test_writer_exit_removes_segment();
    test_reader_survives_writer_exit();
    test_rewritten_segment_is_a_new_one();
    test_no_torn_frames_under_load();
    shm_unlink(NAME);
    std::printf("\ntest_frame_shm: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
