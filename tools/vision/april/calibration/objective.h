#pragma once
// ─── IObjective ───────────────────────────────────────────────────────────────
// Scoring function for a given AprilConfig. Lower value = better.

#include "param_space.h"
#include <string>
#include <vector>

struct IObjective {
    virtual ~IObjective() = default;
    virtual std::string name()                        const = 0;
    virtual double      evaluate(const AprilConfig&)        = 0;
    virtual const std::vector<int>& detectedIds()     const {
        static const std::vector<int> empty;
        return empty;
    }
};

// ─── Detection helper ─────────────────────────────────────────────────────────

struct DetResult {
    std::vector<int>                      ids;
    std::vector<std::vector<cv::Point2f>> corners;
};

inline DetResult detectFrame(const cv::Mat& gray,
                              const AprilConfig& cfg,
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

inline cv::Mat preprocessGray(const cv::Mat& bgr, const AprilConfig& cfg) {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    if (cfg.claheClip > 0) {
        auto clahe = cv::createCLAHE(cfg.claheClip, {cfg.claheTile, cfg.claheTile});
        clahe->apply(gray, gray);
    }
    return gray;
}
