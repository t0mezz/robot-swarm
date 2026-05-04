#pragma once
// ─── StaticObjective ─────────────────────────────────────────────────────────
// Evaluates a set of pre-captured still-scene frames against any candidate
// ArucoConfig.  Frames are provided by the caller (see calib_main.cpp's
// interactive capture loop) rather than captured internally.
//
// Score = -(0.8 · detection_rate + 0.2 · corner_stability)   ∈ [-1, 0]
//
//   detection_rate   = fraction of (expected_id × frame) pairs where the marker
//                      was successfully detected
//
//   corner_stability = 1 − mean_jitter / (5% of marker perimeter)
//                      where mean_jitter = per-corner std dev across frames
//                      Clamped to [0, 1] so a single bad result doesn't dominate.

#include "objective.h"
#include <map>
#include <array>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <unordered_map>

struct StaticObjective : IObjective {
    std::string             name()         const override { return "static"; }
    const std::vector<int>& detectedIds()  const override { return detectedIds_; }

    // Load pre-captured frames and resolve the expected marker ID set.
    // If hintIds is non-empty those IDs are used directly; otherwise IDs seen
    // in >20% of frames are auto-detected from the base config.
    // Returns false if frames is empty or no markers are found.
    bool load(const std::vector<cv::Mat>& frames, const ArucoConfig& baseCfg,
              const std::vector<int>& hintIds = {}) {
        if (frames.empty()) return false;
        frames_ = frames;
        int nFrames = (int)frames_.size();

        if (!hintIds.empty()) {
            detectedIds_ = hintIds;
        } else {
            auto det = makeDetector(baseCfg);
            std::map<int, int> counts;
            for (auto& frame : frames_) {
                auto gray = preprocessGray(frame, baseCfg);
                auto res  = detectFrame(gray, baseCfg, det);
                for (int id : res.ids) counts[id]++;
            }
            int thresh = std::max(1, nFrames / 5);
            detectedIds_.clear();
            for (auto& [id, cnt] : counts)
                if (cnt >= thresh) detectedIds_.push_back(id);
        }

        printf("[calib] Expected IDs: ");
        for (int id : detectedIds_) printf("%d ", id);
        printf("(%zu ids, %d frames)\n", detectedIds_.size(), nFrames);

        return !detectedIds_.empty();
    }

    // Evaluate a candidate config on the loaded frames.
    double evaluate(const ArucoConfig& cfg) override {
        auto det = makeDetector(cfg);

        int totalExpected = 0, totalFound = 0;
        std::map<int, std::vector<std::array<cv::Point2f, 4>>> cornerHist;

        for (auto& frame : frames_) {
            auto gray = preprocessGray(frame, cfg);
            auto res  = detectFrame(gray, cfg, det);

            std::unordered_map<int, int> idToIdx;
            for (int j = 0; j < (int)res.ids.size(); ++j) idToIdx[res.ids[j]] = j;

            for (int eid : detectedIds_) {
                totalExpected++;
                auto it = idToIdx.find(eid);
                if (it != idToIdx.end()) {
                    totalFound++;
                    auto& c = res.corners[it->second];
                    cornerHist[eid].push_back({c[0], c[1], c[2], c[3]});
                }
            }
        }

        double detRate = totalExpected > 0
                         ? (double)totalFound / totalExpected : 0.0;

        double stability = 0.0;
        int    nMarkersStable = 0;
        for (auto& [id, history] : cornerHist) {
            if ((int)history.size() < 3) continue;

            double perim = 0;
            int nSamples = std::min((int)history.size(), 10);
            for (int f = 0; f < nSamples; ++f) {
                auto& c = history[f];
                for (int k = 0; k < 4; ++k)
                    perim += cv::norm(c[k] - c[(k+1)%4]);
            }
            perim /= nSamples;

            double jitter = 0;
            for (int k = 0; k < 4; ++k) {
                double mx = 0, my = 0;
                for (auto& c : history) { mx += c[k].x; my += c[k].y; }
                mx /= history.size(); my /= history.size();
                double var = 0;
                for (auto& c : history)
                    var += (c[k].x-mx)*(c[k].x-mx) + (c[k].y-my)*(c[k].y-my);
                jitter += std::sqrt(var / history.size());
            }
            jitter /= 4.0;

            double ref = 0.05 * std::max(perim, 1.0);
            stability += std::max(0.0, 1.0 - jitter / ref);
            nMarkersStable++;
        }
        if (nMarkersStable > 0) stability /= nMarkersStable;

        return -(0.8 * detRate + 0.2 * stability);
    }

private:
    std::vector<cv::Mat> frames_;
    std::vector<int>     detectedIds_;
};
