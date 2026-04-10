#pragma once
// ─── StaticObjective ─────────────────────────────────────────────────────────
// Captures N frames of a still scene, then evaluates any candidate AprilConfig
// offline against that replay dataset.
//
// Score = -(0.8 · detection_rate + 0.2 · corner_stability)   ∈ [-1, 0]

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

    bool capture(int camIdx, int nFrames, const AprilConfig& baseCfg,
                 const std::vector<int>& hintIds = {}) {
        cv::VideoCapture cap;
        cap.open(camIdx, cv::CAP_AVFOUNDATION);
        if (!cap.isOpened()) {
            fprintf(stderr, "[april-calib] cannot open camera %d\n", camIdx);
            return false;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  baseCfg.width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, baseCfg.height);
        cap.set(cv::CAP_PROP_FPS,          baseCfg.fps);

        for (int i = 0; i < 8; ++i) { cv::Mat tmp; cap.read(tmp); }

        frames_.clear();
        frames_.reserve(nFrames);
        while ((int)frames_.size() < nFrames) {
            cv::Mat f;
            if (cap.read(f) && !f.empty()) {
                frames_.push_back(std::move(f));
                printf("\r[april-calib] Capturing... %d/%d", (int)frames_.size(), nFrames);
                fflush(stdout);
            }
        }
        printf("\n");

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

        printf("[april-calib] Expected IDs: ");
        for (int id : detectedIds_) printf("%d ", id);
        printf("(%zu ids, %d frames)\n", detectedIds_.size(), nFrames);

        return !detectedIds_.empty();
    }

    double evaluate(const AprilConfig& cfg) override {
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
