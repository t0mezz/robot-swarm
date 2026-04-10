#pragma once
// ─── MotionObjective ─────────────────────────────────────────────────────────
// Captures N frames while robots are moving at typical operating speed, then
// evaluates any candidate ArucoConfig offline against that replay dataset.
//
// Why a separate objective from StaticObjective:
//   Motion blur stresses different parameters than a still scene.
//   Large adaptive-threshold windows, lower threshC, and higher errorCorr all
//   help recover smeared marker edges — but the static objective won't push
//   the search in that direction because it never sees blur.  Running both
//   objectives (or combining them) gives a config that works under both conditions.
//
// Score = -(0.5 · detection_rate + 0.3 · streak_ratio + 0.2 · smoothness)
//
//   detection_rate  = fraction of (expected_id × frame) pairs where the marker
//                     was detected — same primary metric as StaticObjective.
//
//   streak_ratio    = per-ID: longest consecutive detection streak / total frames.
//                     Rewards configs that don't flicker — a single-frame dropout
//                     in a moving sequence breaks the Kalman filter's prediction
//                     window, so streak continuity matters more than in static use.
//
//   smoothness      = 1 − velocity_std / (mean_speed + 1)  per ID, averaged.
//                     Measures whether the detected positions form a plausible
//                     trajectory.  A flickering or noisy detector produces large
//                     apparent accelerations; a good config tracks cleanly.
//
// Tip: use --frames 120 for motion (4 s at 30 fps gives the optimizer more
// trajectory data to distinguish good configs from bad ones).

#include "objective.h"
#include <map>
#include <unordered_map>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

struct MotionObjective : IObjective {
    std::string             name()        const override { return "motion"; }
    const std::vector<int>& detectedIds() const override { return detectedIds_; }

    // Open camera, countdown 3 s (so the user can start robots), capture nFrames,
    // then resolve expected marker IDs.  If hintIds is non-empty those IDs are
    // used directly; otherwise IDs seen in >10% of frames are auto-detected
    // (lower threshold than static because robots may leave the frame).
    bool capture(int camIdx, int nFrames, const ArucoConfig& baseCfg,
                 const std::vector<int>& hintIds = {}) {
        cv::VideoCapture cap;
        cap.open(camIdx, cv::CAP_AVFOUNDATION);
        if (!cap.isOpened()) {
            fprintf(stderr, "[calib] cannot open camera %d\n", camIdx);
            return false;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  baseCfg.width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, baseCfg.height);
        cap.set(cv::CAP_PROP_FPS,          baseCfg.fps);

        for (int i = 0; i < 8; ++i) { cv::Mat tmp; cap.read(tmp); }  // warm up

        // Countdown — gives the user time to start robots before recording begins
        printf("[calib] Drive robots at typical operating speed.\n");
        printf("[calib] Capture starts in: ");
        for (int t = 3; t >= 1; --t) {
            printf("%d... ", t); fflush(stdout);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        printf("GO!\n");

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

        if (!hintIds.empty()) {
            // Caller specified IDs explicitly — use them as-is.
            detectedIds_ = hintIds;
        } else {
            // Auto-detect expected IDs — 10% threshold so a robot that occasionally
            // leaves frame or gets occluded is still included in the scoring.
            auto det = makeDetector(baseCfg);
            std::map<int, int> counts;
            for (auto& frame : frames_) {
                auto gray = preprocessGray(frame, baseCfg);
                auto res  = detectFrame(gray, baseCfg, det);
                for (int id : res.ids) counts[id]++;
            }
            int thresh = std::max(1, nFrames / 10);
            detectedIds_.clear();
            for (auto& [id, cnt] : counts)
                if (cnt >= thresh) detectedIds_.push_back(id);
        }

        printf("[calib] Expected IDs: ");
        for (int id : detectedIds_) printf("%d ", id);
        printf("(%d-frame sequence)\n", nFrames);

        return !detectedIds_.empty();
    }

    double evaluate(const ArucoConfig& cfg) override {
        auto det = makeDetector(cfg);

        int totalExpected = 0, totalFound = 0;

        // trajectories[id] = list of (frame_index, centroid_px) for detected frames
        std::map<int, std::vector<std::pair<int, cv::Point2f>>> trajectories;

        for (int fi = 0; fi < (int)frames_.size(); ++fi) {
            auto gray = preprocessGray(frames_[fi], cfg);
            auto res  = detectFrame(gray, cfg, det);

            std::unordered_map<int, int> idToIdx;
            for (int j = 0; j < (int)res.ids.size(); ++j) idToIdx[res.ids[j]] = j;

            for (int eid : detectedIds_) {
                totalExpected++;
                auto it = idToIdx.find(eid);
                if (it != idToIdx.end()) {
                    totalFound++;
                    auto& c = res.corners[it->second];
                    cv::Point2f cen(0, 0);
                    for (auto& pt : c) { cen.x += pt.x; cen.y += pt.y; }
                    trajectories[eid].push_back({fi, cen * 0.25f});
                }
            }
        }

        double detRate = totalExpected > 0
                         ? (double)totalFound / totalExpected : 0.0;

        // ── Streak ratio ──────────────────────────────────────────────────────
        // For each ID, find the longest run of consecutive-frame detections.
        // Normalise by total frame count so short sequences don't get free points.
        double streakRatio = 0.0;
        int    nIds        = 0;
        for (int eid : detectedIds_) {
            auto& traj = trajectories[eid];
            if (traj.empty()) continue;
            int maxStreak = 1, curStreak = 1;
            for (int i = 1; i < (int)traj.size(); ++i) {
                if (traj[i].first == traj[i-1].first + 1) {
                    maxStreak = std::max(maxStreak, ++curStreak);
                } else {
                    curStreak = 1;
                }
            }
            streakRatio += (double)maxStreak / frames_.size();
            nIds++;
        }
        if (nIds > 0) streakRatio /= nIds;

        // ── Trajectory smoothness ─────────────────────────────────────────────
        // Compute per-frame velocities (px/frame) from consecutive detections.
        // Smoothness = 1 − velocity_std / (mean_speed + 1).
        // A config that loses and re-acquires a marker produces large apparent
        // velocity jumps → low smoothness.
        double smoothness = 0.0;
        int    nSmooth    = 0;
        for (int eid : detectedIds_) {
            auto& traj = trajectories[eid];
            if ((int)traj.size() < 3) continue;

            std::vector<double> speeds;
            for (int i = 1; i < (int)traj.size(); ++i) {
                float dt = (float)(traj[i].first - traj[i-1].first);
                if (dt <= 0) continue;
                speeds.push_back(cv::norm(traj[i].second - traj[i-1].second) / dt);
            }
            if (speeds.size() < 2) continue;

            double mean = 0;
            for (auto s : speeds) mean += s;
            mean /= speeds.size();

            double var = 0;
            for (auto s : speeds) var += (s - mean) * (s - mean);
            double stddev = std::sqrt(var / speeds.size());

            smoothness += std::max(0.0, 1.0 - stddev / (mean + 1.0));
            nSmooth++;
        }
        if (nSmooth > 0) smoothness /= nSmooth;

        return -(0.5 * detRate + 0.3 * streakRatio + 0.2 * smoothness);
    }

private:
    std::vector<cv::Mat> frames_;
    std::vector<int>     detectedIds_;
};
