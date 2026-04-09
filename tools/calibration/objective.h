#pragma once
// ─── IObjective ───────────────────────────────────────────────────────────────
// Scoring function for a given ArucoConfig. Lower value = better.
// Each concrete objective (static, motion, fisheye …) implements this interface
// and plugs into the optimisation loop in calib_main.cpp without changes there.

#include "param_space.h"
#include <string>
#include <vector>
#include <set>

struct IObjective {
    virtual ~IObjective() = default;
    virtual std::string name()                       const = 0;
    // Evaluate a candidate config. Called for every CMA-ES/GP candidate — must
    // be fast (detection on pre-captured frames, not live camera access).
    virtual double      evaluate(const ArucoConfig&)       = 0;
    // IDs reliably detected during the capture phase (available after capture()).
    virtual const std::vector<int>& detectedIds()    const {
        static const std::vector<int> empty;
        return empty;
    }
};

// ─── Detection helper ─────────────────────────────────────────────────────────
// Run a detector on a single pre-processed grayscale frame, respecting the
// half-res sweep setting from the config.  Corners are returned in full-res
// pixel coordinates regardless of the sweep scale.

struct DetResult {
    std::vector<int>                      ids;
    std::vector<std::vector<cv::Point2f>> corners;
};

inline DetResult detectFrame(const cv::Mat& gray,
                              const ArucoConfig& cfg,
                              cv::aruco::ArucoDetector& det) {
    DetResult r;
    std::vector<std::vector<cv::Point2f>> rej;
    if (cfg.halfResSweep) {
        cv::Mat half;
        cv::resize(gray, half, {}, 0.5, 0.5, cv::INTER_AREA);
        det.detectMarkers(half, r.corners, r.ids, rej);
        for (auto& c : r.corners)
            for (auto& pt : c) { pt.x *= 2.0f; pt.y *= 2.0f; }
    } else {
        det.detectMarkers(gray, r.corners, r.ids, rej);
    }
    return r;
}

// Convenience: preprocess a grayscale frame the same way ArucoTracker does
// (CLAHE, no fisheye — add a FisheyeUndistortPreprocessor stage upstream if needed).
inline cv::Mat preprocessGray(const cv::Mat& bgr, const ArucoConfig& cfg) {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    if (cfg.claheClip > 0) {
        auto clahe = cv::createCLAHE(cfg.claheClip, {cfg.claheTile, cfg.claheTile});
        clahe->apply(gray, gray);
    }
    return gray;
}
