// test_camera_intrinsics.cpp
// Unit tests for lib/ArucoTracker/camera_intrinsics.h — the lens model the
// tracker uses to undistort tracked points. OpenCV only, no camera: the
// round trips and the file-compatibility rules are what must hold.

#include "../lib/ArucoTracker/camera_intrinsics.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

static int g_pass = 0, g_fail = 0;
#define EXPECT_NEAR(actual, expected, tol, msg)                                  \
    do {                                                                         \
        double _a = (actual), _e = (expected);                                   \
        if (std::fabs(_a - _e) <= (tol)) g_pass++;                               \
        else { g_fail++; std::printf("FAIL %s: expected %g, got %g (%s)\n",      \
                                     __func__, _e, _a, msg); }                   \
    } while (0)
#define EXPECT_TRUE(cond, msg)                                                   \
    do { if (cond) g_pass++; else { g_fail++; std::printf("FAIL %s: %s\n", __func__, msg); } } while (0)

static const cv::Size SZ(2048, 2048);

static void write(const std::string& path, int w, int h, int ox, int oy) {
    cv::Mat K = (cv::Mat_<double>(3, 3) << 2727, 0, 1030, 0, 2727, 1019, 0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(1, 5) << -0.11, 0.07, 0.0004, -0.0003, 0);
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    fs << "K" << K << "D" << D << "image_size" << cv::Size(w, h)
       << "offset_x" << ox << "offset_y" << oy << "rms" << 0.2;
}

static CameraIntrinsics make(bool mirror) {
    CameraIntrinsics I;
    write("/tmp/test_intrinsics.yml", 2048, 2048, 160, 0);
    I.load("/tmp/test_intrinsics.yml", SZ, {160, 0});
    I.mirror = mirror;
    return I;
}

static void test_identity_without_a_lens() {
    CameraIntrinsics I;
    cv::Point2f p(123.f, 456.f);
    EXPECT_TRUE(!I.valid(), "fresh object is invalid");
    EXPECT_NEAR(I.undistort(p).x, 123, 0, "undistort is identity");
    EXPECT_NEAR(I.distort(p).y, 456, 0, "distort is identity");
}

static void test_round_trip() {
    for (bool mirror : {false, true}) {
        CameraIntrinsics I = make(mirror);
        EXPECT_TRUE(I.valid(), "loads");
        double worst = 0;
        for (int y = 0; y <= 2047; y += 128)
            for (int x = 0; x <= 2047; x += 128) {
                cv::Point2f p((float)x, (float)y);
                worst = std::max(worst, (double)cv::norm(I.distort(I.undistort(p)) - p));
            }
        EXPECT_NEAR(worst, 0, 0.01, "distort(undistort(p)) == p to 0.01 px");
    }
}

static void test_principal_point_barely_moves() {
    CameraIntrinsics I = make(false);
    cv::Point2f c(1030.f, 1019.f);
    EXPECT_NEAR(cv::norm(I.undistort(c) - c), 0, 0.5, "principal point is (nearly) fixed");
}

static void test_barrel_pulls_edges_outward_when_undistorted() {
    // k1 < 0 (barrel): straight lines bow outward, so correcting moves an edge
    // point *away* from the centre.
    CameraIntrinsics I = make(false);
    cv::Point2f c(1030.f, 1019.f), e(2000.f, 1019.f);
    EXPECT_TRUE(cv::norm(I.undistort(e) - c) > cv::norm(e - c) + 1.0, "edge moves outward");
}

static void test_mirror_is_a_reflection_of_the_unmirrored_model() {
    // A frame flipped left-right is the same lens seen in a mirror: undistorting
    // the flipped point must give the flipped result. Only exact when the
    // principal point is centred, so check the symmetric part: the y shift.
    CameraIntrinsics A = make(false), B = make(true);
    cv::Point2f p(300.f, 400.f);
    cv::Point2f pf((float)(SZ.width - 1) - p.x, p.y);
    cv::Point2f ua = A.undistort(p), ub = B.undistort(pf);
    EXPECT_NEAR(ub.x, (SZ.width - 1) - ua.x, 1e-3, "x mirrored");
    EXPECT_NEAR(ub.y, ua.y, 1e-3, "y unchanged");
}

static void test_rejects_stale_files() {
    write("/tmp/test_intrinsics.yml", 1920, 1080, 160, 0);
    CameraIntrinsics I;
    EXPECT_TRUE(!I.load("/tmp/test_intrinsics.yml", SZ, {160, 0}), "different resolution rejected");
    write("/tmp/test_intrinsics.yml", 2048, 2048, 0, 0);
    EXPECT_TRUE(!I.load("/tmp/test_intrinsics.yml", SZ, {160, 0}), "different sensor offset rejected");
    EXPECT_TRUE(!I.valid(), "stays invalid after a rejection");
    EXPECT_TRUE(!I.load("/tmp/does_not_exist.yml", SZ), "missing file rejected");
}

static void test_global_helpers() {
    arucoIntrinsics() = CameraIntrinsics{};
    cv::Point2f p(10.f, 20.f);
    EXPECT_NEAR(arucoUndistortPixel(p).x, 10, 0, "global is identity until loaded");
    arucoIntrinsics() = make(false);
    EXPECT_TRUE(cv::norm(arucoUndistortPixel(cv::Point2f(2000.f, 1019.f)) - cv::Point2f(2000.f, 1019.f)) > 1.0,
                "global applies the loaded lens");
    auto v = arucoUndistortPixels({{2000.f, 1019.f}, {1030.f, 1019.f}});
    EXPECT_TRUE(v.size() == 2, "vector overload");
}

int main() {
    test_identity_without_a_lens();
    test_round_trip();
    test_principal_point_barely_moves();
    test_barrel_pulls_edges_outward_when_undistorted();
    test_mirror_is_a_reflection_of_the_unmirrored_model();
    test_rejects_stale_files();
    test_global_helpers();
    std::printf("test_camera_intrinsics: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
