#pragma once
// ─── StaticObjective ─────────────────────────────────────────────────────────
// Captures N frames of a still scene, then evaluates any candidate ArucoConfig
// offline against that replay dataset.
//
// Score = -(0.7 · detection_rate + 0.3 · corner_stability)   ∈ [-1, 0]
//
//   detection_rate   = fraction of (expected_id × frame) pairs where the marker
//                      was successfully detected
//
//   corner_stability = 1 − mean_jitter / (5% of marker perimeter)
//                      where mean_jitter = per-corner std dev across frames
//                      Clamped to [0, 1] so a single bad result doesn't dominate.
//
// Future objectives (motion, fisheye) follow the same interface — implement
// IObjective, add a capture() method if needed, and drop into calib_main.cpp.

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

    // Open camera, capture nFrames, auto-detect expected marker IDs from the
    // base config, and close the camera.  Returns false if the camera can't
    // be opened or no markers are found.
    bool capture(int camIdx, int nFrames, const ArucoConfig& baseCfg) {
        cv::VideoCapture cap;
        cap.open(camIdx, cv::CAP_AVFOUNDATION);
        if (!cap.isOpened()) {
            fprintf(stderr, "[calib] cannot open camera %d\n", camIdx);
            return false;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  baseCfg.width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, baseCfg.height);
        cap.set(cv::CAP_PROP_FPS,          baseCfg.fps);

        // Discard first few frames — cameras often return dark/unstable frames
        // immediately after open before the auto-exposure settles.
        for (int i = 0; i < 8; ++i) { cv::Mat tmp; cap.read(tmp); }

        frames_.clear();
        frames_.reserve(nFrames);
        while ((int)frames_.size() < nFrames) {
            cv::Mat f;
            if (cap.read(f) && !f.empty()) {
                frames_.push_back(std::move(f));
                printf("\r[calib] Capturing... %d/%d", (int)frames_.size(), nFrames);
                fflush(stdout);
            }
        }
        printf("\n");

        // Auto-detect which IDs are reliably present (seen in >20% of frames)
        // using the base config so the starting point acts as a sanity check.
        auto det   = makeDetector(baseCfg);
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

        printf("[calib] Expected IDs: ");
        for (int id : detectedIds_) printf("%d ", id);
        printf("(%zu/%d frames avg)\n", detectedIds_.size(), nFrames);

        return !detectedIds_.empty();
    }

    // Evaluate a candidate config on the captured replay dataset.
    double evaluate(const ArucoConfig& cfg) override {
        auto det = makeDetector(cfg);

        int totalExpected = 0, totalFound = 0;
        // cornerHist[id] = list of {corner[0..3]} across frames where id was found
        std::map<int, std::vector<std::array<cv::Point2f, 4>>> cornerHist;

        for (auto& frame : frames_) {
            auto gray = preprocessGray(frame, cfg);
            auto res  = detectFrame(gray, cfg, det);

            // index detected ids for O(1) lookup
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

        // Corner stability: for each reliably detected ID, compute mean per-corner
        // std-dev across frames and normalise by 5% of the marker's apparent perimeter.
        double stability = 0.0;
        int    nMarkersStable = 0;
        for (auto& [id, history] : cornerHist) {
            if ((int)history.size() < 3) continue;  // need at least 3 samples

            // Estimate apparent perimeter from first few detections
            double perim = 0;
            int nSamples = std::min((int)history.size(), 10);
            for (int f = 0; f < nSamples; ++f) {
                auto& c = history[f];
                for (int k = 0; k < 4; ++k)
                    perim += cv::norm(c[k] - c[(k+1)%4]);
            }
            perim /= nSamples;  // pixels

            // Mean per-corner positional std-dev across frames
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

            double ref = 0.05 * std::max(perim, 1.0);  // 5% of marker perimeter
            stability += std::max(0.0, 1.0 - jitter / ref);
            nMarkersStable++;
        }
        if (nMarkersStable > 0) stability /= nMarkersStable;

        return -(0.7 * detRate + 0.3 * stability);
    }

private:
    std::vector<cv::Mat> frames_;
    std::vector<int>     detectedIds_;
};
