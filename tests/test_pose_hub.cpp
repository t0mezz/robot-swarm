// test_pose_hub.cpp
// Round-trip tests for lib/ArucoTracker/pose_hub.h — the publish/subscribe
// path that lets pose-only tools read the tracker without opening the camera.
//
// Publisher and subscriber both run in this process over a real Unix socket,
// so this exercises the actual framing, partial reads and disconnect handling
// rather than a mock. No camera and no OpenCV involved, which is precisely the
// property the header exists to provide.
//
// No test framework — plain asserts with a pass/fail tally, run via
// `make test` (see tests/Makefile).

#include "../lib/ArucoTracker/pose_hub.h"

#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_EQ(actual, expected, msg) \
    do { \
        auto _actual_val   = (actual); \
        auto _expected_val = (expected); \
        if (_actual_val == _expected_val) { \
            g_pass++; \
        } else { \
            g_fail++; \
            std::printf("FAIL %s: expected %lld, got %lld (%s)\n", \
                        __func__, (long long)_expected_val, (long long)_actual_val, msg); \
        } \
    } while (0)

#define EXPECT_TRUE(cond, msg)  EXPECT_EQ((bool)(cond), true, msg)
#define EXPECT_NEAR(a, b, tol, msg) \
    do { \
        double _d = (double)(a) - (double)(b); \
        if (_d < 0) _d = -_d; \
        if (_d <= (tol)) { g_pass++; } \
        else { g_fail++; std::printf("FAIL %s: %f vs %f (%s)\n", __func__, (double)(a), (double)(b), msg); } \
    } while (0)

// The subscriber is non-blocking, so a published snapshot needs a moment to
// arrive. Polls until something lands rather than sleeping a fixed interval.
static bool pumpUntilFresh(PoseHubSubscriber& sub, int attempts = 200) {
    for (int i = 0; i < attempts; i++) {
        if (sub.poll()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

static std::vector<HubPose> samplePoses() {
    return {
        HubPose{0,  1210.f, 1450.f,   0.f,  620.f,  740.f},
        HubPose{7,   980.f,  760.f,  90.f,  501.f,  389.f},
        HubPose{19, 1640.f, 1120.f, -45.f, 1002.f,  573.f},
    };
}

static void test_round_trip() {
    PoseHubPublisher pub;
    EXPECT_TRUE(pub.start(), "publisher should bind a free socket");

    PoseHubSubscriber sub;
    EXPECT_TRUE(sub.connect(), "subscriber should connect");

    // A snapshot published before the subscriber is accepted is simply missed;
    // publish() accepts first, so this one lands.
    const auto poses = samplePoses();
    pub.publish(poses, 116.5f, 2048, 2048);
    EXPECT_EQ(pub.clientCount(), (size_t)1, "publisher should see one client");

    EXPECT_TRUE(pumpUntilFresh(sub), "snapshot should arrive");
    EXPECT_EQ(sub.poses().size(), (size_t)3, "all poses survive the trip");
    EXPECT_NEAR(sub.detectionFps(), 116.5f, 0.01, "fps survives");
    EXPECT_EQ(sub.frameWidth(), 2048, "frame width survives");
    EXPECT_EQ(sub.frameHeight(), 2048, "frame height survives");

    for (size_t i = 0; i < poses.size(); i++) {
        EXPECT_EQ(sub.poses()[i].id, poses[i].id, "id survives");
        EXPECT_NEAR(sub.poses()[i].x,   poses[i].x,   0.001, "x survives");
        EXPECT_NEAR(sub.poses()[i].y,   poses[i].y,   0.001, "y survives");
        EXPECT_NEAR(sub.poses()[i].yaw, poses[i].yaw, 0.001, "yaw survives");
        EXPECT_NEAR(sub.poses()[i].px,  poses[i].px,  0.001, "px survives");
        EXPECT_NEAR(sub.poses()[i].py,  poses[i].py,  0.001, "py survives");
    }
}

static void test_empty_snapshot() {
    PoseHubPublisher pub;
    EXPECT_TRUE(pub.start(), "publisher should start");
    PoseHubSubscriber sub;
    EXPECT_TRUE(sub.connect(), "subscriber should connect");

    pub.publish(samplePoses(), 60.f, 2048, 2048);
    EXPECT_TRUE(pumpUntilFresh(sub), "first snapshot arrives");
    EXPECT_EQ(sub.poses().size(), (size_t)3, "three tracked");

    // Every marker leaving the frame must clear the subscriber's list, not
    // leave the last known positions on screen forever.
    pub.publish({}, 60.f, 2048, 2048);
    EXPECT_TRUE(pumpUntilFresh(sub), "empty snapshot arrives");
    EXPECT_EQ(sub.poses().size(), (size_t)0, "no markers means no poses");
}

static void test_keeps_only_newest() {
    PoseHubPublisher pub;
    EXPECT_TRUE(pub.start(), "publisher should start");
    PoseHubSubscriber sub;
    EXPECT_TRUE(sub.connect(), "subscriber should connect");

    // A subscriber that fell behind wants current positions, not a backlog:
    // one poll() must collapse everything queued down to the latest snapshot.
    for (int i = 0; i < 20; i++) {
        std::vector<HubPose> p{HubPose{1, (float)i, 0.f, 0.f, 0.f, 0.f}};
        pub.publish(p, 30.f + (float)i, 2048, 2048);
    }
    EXPECT_TRUE(pumpUntilFresh(sub), "snapshots arrive");
    EXPECT_EQ(sub.poses().size(), (size_t)1, "one marker");
    EXPECT_NEAR(sub.poses()[0].x, 19.f, 0.001, "newest snapshot wins");
    EXPECT_NEAR(sub.detectionFps(), 49.f, 0.001, "newest metadata wins");
}

static void test_publisher_exit_disconnects_subscriber() {
    PoseHubSubscriber sub;
    {
        PoseHubPublisher pub;
        EXPECT_TRUE(pub.start(), "publisher should start");
        EXPECT_TRUE(sub.connect(), "subscriber should connect");
        pub.publish(samplePoses(), 116.f, 2048, 2048);
        EXPECT_TRUE(pumpUntilFresh(sub), "snapshot arrives while publisher lives");
    }   // publisher destructs — the demo owning the camera has exited

    // The subscriber must notice, so the dashboard can fall back to "no camera"
    // and start retrying instead of showing stale poses forever.
    for (int i = 0; i < 200 && sub.isConnected(); i++) {
        sub.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE(!sub.isConnected(), "subscriber should detect the publisher exiting");
}

static void test_second_publisher_is_refused() {
    PoseHubPublisher first;
    EXPECT_TRUE(first.start(), "first publisher takes the socket");

    // Only the camera owner publishes and the camera has one owner, so this
    // should never happen in practice — but a live socket must not be stolen.
    PoseHubPublisher second;
    EXPECT_TRUE(!second.start(), "a live publisher must not be displaced");
    EXPECT_TRUE(first.isRunning(), "the original keeps the socket");
}

static void test_stale_socket_is_reclaimed() {
    // A publisher killed with SIGKILL leaves the socket file behind. The next
    // one must be able to take it, or vision stays broken until someone
    // manually deletes a file in /tmp. Built by hand rather than through the
    // publisher, so the production class needs no test-only escape hatch:
    // bind the path, then drop the socket without unlinking it.
    ::unlink(POSE_HUB_SOCK_PATH);
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    EXPECT_TRUE(fd >= 0, "raw socket");
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, POSE_HUB_SOCK_PATH, sizeof(addr.sun_path) - 1);
    EXPECT_EQ(::bind(fd, (sockaddr*)&addr, sizeof(addr)), 0, "bind the stale path");
    ::close(fd);   // file survives; nothing is listening on it

    PoseHubPublisher next;
    EXPECT_TRUE(next.start(), "a stale socket file must not block a new publisher");
}

static void test_no_subscribers_is_harmless() {
    PoseHubPublisher pub;
    EXPECT_TRUE(pub.start(), "publisher should start");
    for (int i = 0; i < 100; i++) pub.publish(samplePoses(), 116.f, 2048, 2048);
    EXPECT_EQ(pub.clientCount(), (size_t)0, "nobody listening");
    EXPECT_TRUE(pub.isRunning(), "publishing into the void must not break it");
}

int main() {
    test_round_trip();
    test_empty_snapshot();
    test_keeps_only_newest();
    test_publisher_exit_disconnects_subscriber();
    test_second_publisher_is_refused();
    test_stale_socket_is_reclaimed();
    test_no_subscribers_is_harmless();

    std::printf("\ntest_pose_hub: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
