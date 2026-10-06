#pragma once
// camera_intrinsics.h — lens model (K, D) shared by the tracker and the tools.
//
// Rectilinear pinhole + Brown-Conrady distortion (cv::calibrateCamera), written
// by tools/build/intrinsics to camera_intrinsics.yml. This is NOT the fisheye
// model: the Basler C125-0618-5M is a plain 6 mm lens.
//
// The tracker corrects *points*, never frames. A full-frame remap of a
// 2048x2048 image costs more than the whole ArUco detection at 115 fps; the
// tracked centres and heading tips are a handful of points. "Undistorted pixel"
// below means: where the pixel would sit with the same K and zero distortion.
// The homography is calibrated in, and applied to, that space.
//
// OpenCV only — no pylon, no SwarmClient — so any tool can include it.
// Everything is the identity until a calibration has been loaded, so tools that
// call arucoUndistortPixel() behave exactly as before without one.

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <cstdio>
#include <string>
#include <vector>

struct CameraIntrinsics {
    cv::Mat  K, D;          // 3x3 and 1x5 (CV_64F)
    cv::Size size;          // resolution it was calibrated at
    cv::Point offset;       // sensor ROI offset (offset_x/offset_y) it was calibrated at
    double   rms = 0;       // reprojection RMS from calibration, px
    // The tracker flips frames *before* detection when mirror_input is set, but
    // K and D describe the unflipped sensor image. Points are un-flipped before
    // the lens model is applied and flipped back after, so callers stay in the
    // coordinates of the frame they see.
    bool     mirror = false;

    bool valid() const { return !K.empty() && !D.empty(); }

    // Fails (leaving *this invalid) if the file is absent or was made at a
    // different resolution — K scales with the image, so a mismatch would
    // silently mis-scale every pose.
    // The principal point depends on which part of the sensor is read out, so
    // a different ROI offset invalidates K just as a different size does.
    bool load(const std::string& path, cv::Size frame, cv::Point roiOffset = {0, 0}) {
        *this = CameraIntrinsics{};
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        cv::Mat k, d; cv::Size sz;
        fs["K"] >> k;  fs["D"] >> d;  fs["image_size"] >> sz;
        double r = 0;  if (!fs["rms"].empty()) fs["rms"] >> r;
        if (k.empty() || d.empty()) return false;
        if (sz.width > 0 && sz.height > 0 && sz != frame) {
            fprintf(stderr,
                "[intrinsics] '%s' was calibrated at %dx%d but the camera is %dx%d "
                "— ignoring it. Re-run tools/build/intrinsics.\n",
                path.c_str(), sz.width, sz.height, frame.width, frame.height);
            return false;
        }
        cv::Point off(0, 0);
        if (!fs["offset_x"].empty()) fs["offset_x"] >> off.x;
        if (!fs["offset_y"].empty()) fs["offset_y"] >> off.y;
        if (off != roiOffset) {
            fprintf(stderr,
                "[intrinsics] '%s' was calibrated with sensor offset (%d,%d) but the "
                "config has (%d,%d) — ignoring it. Re-run tools/build/intrinsics.\n",
                path.c_str(), off.x, off.y, roiOffset.x, roiOffset.y);
            return false;
        }
        k.convertTo(K, CV_64F);  d.convertTo(D, CV_64F);
        D = D.reshape(1, 1);
        size = frame;  offset = off;  rms = r;
        return true;
    }

    // frame pixel -> undistorted pixel
    std::vector<cv::Point2f> undistort(const std::vector<cv::Point2f>& in) const {
        if (!valid() || in.empty()) return in;
        std::vector<cv::Point2f> p = in, out;
        if (mirror) for (auto& q : p) q.x = (float)(size.width - 1) - q.x;
        cv::undistortPoints(p, out, K, D, cv::noArray(), K,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 1e-9));
        if (mirror) for (auto& q : out) q.x = (float)(size.width - 1) - q.x;
        return out;
    }
    cv::Point2f undistort(cv::Point2f p) const {
        return valid() ? undistort(std::vector<cv::Point2f>{p})[0] : p;
    }

    // undistorted pixel -> frame pixel (for drawing a world-space overlay back
    // onto the camera image)
    std::vector<cv::Point2f> distort(const std::vector<cv::Point2f>& in) const {
        if (!valid() || in.empty()) return in;
        const double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
        const double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);
        std::vector<cv::Point3f> rays;
        rays.reserve(in.size());
        for (auto q : in) {
            if (mirror) q.x = (float)(size.width - 1) - q.x;
            rays.push_back({(float)((q.x - cx) / fx), (float)((q.y - cy) / fy), 1.f});
        }
        std::vector<cv::Point2f> out;
        cv::projectPoints(rays, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D, out);
        if (mirror) for (auto& q : out) q.x = (float)(size.width - 1) - q.x;
        return out;
    }
    cv::Point2f distort(cv::Point2f p) const {
        return valid() ? distort(std::vector<cv::Point2f>{p})[0] : p;
    }
};

// One lens per process. Loaded by ArucoTracker::open(); identity until then.
inline CameraIntrinsics& arucoIntrinsics() {
    static CameraIntrinsics c;
    return c;
}
inline cv::Point2f arucoUndistortPixel(cv::Point2f p) { return arucoIntrinsics().undistort(p); }
inline cv::Point2f arucoDistortPixel(cv::Point2f p)   { return arucoIntrinsics().distort(p); }
inline std::vector<cv::Point2f> arucoUndistortPixels(const std::vector<cv::Point2f>& v) {
    return arucoIntrinsics().undistort(v);
}
inline std::vector<cv::Point2f> arucoDistortPixels(const std::vector<cv::Point2f>& v) {
    return arucoIntrinsics().distort(v);
}
