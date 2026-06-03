// tools/calibration/calib_main.cpp
// Tunes aruco_tracker_config.json for the current lighting environment.
//
// Architecture:
//   IOptimizer  — CMAES                  future: gp_ard.h
//   IObjective  — StaticObjective
//   param_space — encode/decode/write     shared by all optimizers and objectives
//
// Usage:
//   ./calibrate [options]
//     --eval                  evaluate the current config only (no optimisation)
//     --use-cache             skip capture and reuse frames saved by the last run
//     --cache-dir  <path>     directory for frame cache (default: /tmp/calib_frame_cache)
//     --camera, --cam  <idx>  camera index (default: 0)
//     --ids     <n>           treat IDs 0…n as the expected marker set instead of
//                             auto-detecting from the captured frames (e.g. --ids 2
//                             → expects markers 0, 1, 2)
//     --iters   <n>           number of CMA-ES generations (default: 150)
//     --sigma   <f>           initial step size in [0,1] space (default: 0.30)
//     --config  <path>        base config to start from
//                             (default: ../aruco_tracker_config.json)
//     --output  <path>        where to write the optimised config
//                             (default: same as --config)
//
// Capture phase:
//   A live preview window opens.  Press SPACE to capture a frame; press ENTER
//   or Q when you have enough pictures.  At least 5 frames are recommended —
//   more pictures from varied positions give the optimiser a broader dataset.
//   The camera is not accessed again once capture is complete.
//   Frames are automatically saved to --cache-dir so subsequent runs can use
//   --use-cache to skip recapture.
//   If mirror_input is set in the config the preview is shown mirrored (matching
//   what the tracker sees) and detection during optimisation is done on the
//   mirrored frames — no manual adjustment needed.

#include <cstdio>
#include <cmath>
#include <string>
#include <memory>
#include <algorithm>
#include <filesystem>

#include "cmaes.h"
#include "param_space.h"
#include "objective.h"
#include "objective_static.h"

// ─── Argument parsing ─────────────────────────────────────────────────────────

struct Args {
    std::string baslerSerial = "";
    std::string baslerIp     = "";
    bool        eval     = false;
    bool        useCache = false;
    std::string cacheDir = "/tmp/calib_frame_cache";
    int         idsMax   = -1;
    int         maxIter  = 150;
    double      sigma0   = 0.30;
    std::string config   = "../aruco_tracker_config.json";
    std::string output   = "../aruco_tracker_config_optimised.json";
};

static Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        if      (k == "--eval")      a.eval     = true;
        else if (k == "--use-cache") a.useCache = true;
        else if (i + 1 < argc) {
            if      (k == "--serial")    a.baslerSerial = argv[++i];
            else if (k == "--ip")        a.baslerIp     = argv[++i];
            else if (k == "--cache-dir") a.cacheDir     = argv[++i];
            else if (k == "--ids")       a.idsMax       = std::stoi(argv[++i]);
            else if (k == "--iters")     a.maxIter      = std::stoi(argv[++i]);
            else if (k == "--sigma")     a.sigma0       = std::stod(argv[++i]);
            else if (k == "--config")    a.config       = argv[++i];
            else if (k == "--output")    a.output       = argv[++i];
            else fprintf(stderr, "Unknown option: %s\n", k.c_str());
        }
    }
    if (a.output.empty()) a.output = a.config;
    return a;
}

// ─── Frame cache ──────────────────────────────────────────────────────────────
// Frames are saved as PNG so the user can re-run the optimiser with --use-cache
// without having to set up the camera again.

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

static void saveFrameCache(const std::vector<cv::Mat>& frames,
                           const std::string& dir) {
    fs::create_directories(dir);
    for (auto& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".png") fs::remove(e.path());
    for (int i = 0; i < (int)frames.size(); ++i) {
        char path[512];
        snprintf(path, sizeof(path), "%s/frame_%03d.png", dir.c_str(), i);
        cv::imwrite(path, frames[i]);
    }
    printf("[calib] Saved %d frame(s) to cache: %s\n", (int)frames.size(), dir.c_str());
}

static std::vector<cv::Mat> loadFrameCache(const std::string& dir) {
    if (!fs::exists(dir)) return {};
    std::vector<fs::path> paths;
    for (auto& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".png") paths.push_back(e.path());
    std::sort(paths.begin(), paths.end());
    std::vector<cv::Mat> frames;
    for (auto& p : paths) {
        cv::Mat f = cv::imread(p.string());
        if (!f.empty()) frames.push_back(f);
    }
    return frames;
}

// ─── Interactive capture ──────────────────────────────────────────────────────
// Opens the camera and shows a live preview.  SPACE captures the current frame;
// ENTER or Q closes the preview and returns all captured frames.
// An overlay shows how many frames have been captured so far.
// If mirror_input is set in the config the preview is shown mirrored (matching
// what the tracker sees); raw frames are stored so the flip is applied uniformly
// during evaluation via preprocessGray.

static std::vector<cv::Mat> captureInteractive(const ArucoConfig& cfg) {
    BaslerPylonSource cam;
    if (!cam.open(cfg)) {
        fprintf(stderr, "[calib] cannot open Basler camera\n");
        return {};
    }

    // Flush pipeline frames before interactive capture.
    for (int i = 0; i < 5; ++i) { cv::Mat tmp; cam.read(tmp); }

    const char* WIN = "Calibration  —  SPACE: capture frame  |  ENTER / Q: done";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN, cfg.width, cfg.height);

    printf("Place all ArUco markers in view.\n");
    printf("SPACE: capture a frame  |  ENTER or Q: start optimisation\n");
    fflush(stdout);

    std::vector<cv::Mat> frames;
    cv::Mat frame;

    while (true) {
        if (!cam.read(frame) || frame.empty()) { cv::waitKey(10); continue; }

        cv::Mat display = frame.clone();
        if (cfg.mirrorInput) cv::flip(display, display, 1);
        std::string label = "Captured: " + std::to_string(frames.size());
        if (cfg.mirrorInput) label += "  [mirrored]";
        cv::putText(display, label, {10, 36},
                    cv::FONT_HERSHEY_SIMPLEX, 1.1, {0, 220, 0}, 2, cv::LINE_AA);
        cv::imshow(WIN, display);

        int key = cv::waitKey(30);
        if (key == ' ') {
            frames.push_back(frame.clone());
            printf("[calib] Frame %d captured\n", (int)frames.size());
            fflush(stdout);
        } else if (key == 13 || key == 10 || key == 'q' || key == 'Q') {
            break;
        }
    }

    cv::destroyWindow(WIN);
    return frames;
}

// ─── Optimisation loop ────────────────────────────────────────────────────────

static void runOptimisation(IOptimizer&        opt,
                             IObjective&        obj,
                             const ArucoConfig& base,
                             int                maxIter) {
    opt.setMean(encode(base));

    printf("\n%-7s %-10s %-10s %-10s\n", "gen", "best%", "gen_best%", "sigma");
    printf("------- ---------- ---------- ----------\n");

    for (int gen = 0; gen < maxIter && !opt.converged(); ++gen) {
        auto candidates = opt.ask();

        std::vector<double> fits(opt.lambda());
        for (int k = 0; k < opt.lambda(); ++k)
            fits[k] = obj.evaluate(decode(candidates[k], base));

        opt.tell(fits);

        double genBest = *std::min_element(fits.begin(), fits.end());
        printf("%-7d %-10.1f %-10.1f %-10.6f\n",
               gen + 1,
               -opt.bestFit() * 100.0,
               -genBest       * 100.0,
               opt.sigma());
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    printf("=== ArUco Tracker Calibration (CMA-ES%s) ===\n",
           args.eval ? " / eval" : "");
    if (!args.baslerSerial.empty()) printf("Serial: %s\n", args.baslerSerial.c_str());
    if (!args.baslerIp.empty())     printf("IP:     %s\n", args.baslerIp.c_str());
    if (!args.eval)
        printf("MaxIter: %d  Sigma0: %.2f\n", args.maxIter, args.sigma0);
    printf("Base config:   %s\n", args.config.c_str());
    printf("Output config: %s\n", args.output.c_str());
    printf("Cache dir:     %s\n\n", args.cacheDir.c_str());

    ArucoConfig base  = ArucoConfig::fromFile(args.config);
    if (!args.baslerSerial.empty()) base.baslerSerial = args.baslerSerial;
    if (!args.baslerIp.empty())     base.baslerIp     = args.baslerIp;
    base.debugOverlay = false;

    std::vector<int> expectedIds;
    if (args.idsMax >= 0) {
        for (int i = 0; i <= args.idsMax; ++i) expectedIds.push_back(i);
        printf("Expected IDs (from --ids): ");
        for (int id : expectedIds) printf("%d ", id);
        printf("\n");
    }

    // ── Capture phase ─────────────────────────────────────────────────────────
    std::vector<cv::Mat> frames;
    if (args.useCache) {
        frames = loadFrameCache(args.cacheDir);
        if (frames.empty()) {
            fprintf(stderr, "[calib] no cached frames in %s — run without --use-cache first.\n",
                    args.cacheDir.c_str());
            return 1;
        }
        printf("[calib] Loaded %d frame(s) from cache: %s\n",
               (int)frames.size(), args.cacheDir.c_str());
    } else {
        frames = captureInteractive(base);
        if (frames.empty()) {
            fprintf(stderr, "Error: no frames captured.\n");
            return 1;
        }
        saveFrameCache(frames, args.cacheDir);
    }
    if ((int)frames.size() < 5)
        printf("[calib] Warning: only %d frame(s) — 5 or more recommended.\n",
               (int)frames.size());
    printf("[calib] %d frame(s) ready for optimisation.\n\n", (int)frames.size());

    auto so = std::make_unique<StaticObjective>();
    if (!so->load(frames, base, expectedIds)) {
        fprintf(stderr, "Error: no markers detected in captured frames.\n");
        return 1;
    }

    // ── Baseline score ────────────────────────────────────────────────────────
    double baseScore = so->evaluate(base);
    printf("\nBaseline score: %.1f%%\n", -baseScore * 100.0);

    if (args.eval) {
        printf("\n(--eval mode: no optimisation run)\n");
        return 0;
    }

    // ── Optimisation phase ────────────────────────────────────────────────────
    printf("\nStarting CMA-ES  (n=%d, lambda=%d, max_gen=%d)\n",
           kNParams, 4 + (int)(3 * std::log(kNParams)), args.maxIter);

    CMAES opt(kNParams, args.sigma0);
    runOptimisation(opt, *so, base, args.maxIter);

    // ── Write result ──────────────────────────────────────────────────────────
    ArucoConfig bestCfg = decode(opt.bestX(), base);
    writeConfig(bestCfg, args.output);

    printf("\n=== Results ===\n");
    printf("Baseline score:  %.1f%%\n", -baseScore     * 100.0);
    printf("Optimised score: %.1f%%\n", -opt.bestFit() * 100.0);
    printf("Improvement:    +%.1f%%\n", (-opt.bestFit() + baseScore) * 100.0);
    printf("Written to:      %s\n",     args.output.c_str());

    printf("\nChanged parameters:\n");
    auto baseX = encode(base);
    auto bestX = opt.bestX();
    bool anyChange = false;
    for (int i = 0; i < kNParams; ++i) {
        double bv  = fromNorm(baseX[i], kParams[i]);
        double ov  = fromNorm(bestX[i], kParams[i]);
        double rel = std::abs(ov - bv) / (kParams[i].hi - kParams[i].lo);
        if (rel > 0.01) {
            printf("  %-18s  %.4f  →  %.4f\n", kParams[i].key, bv, ov);
            anyChange = true;
        }
    }
    if (!anyChange) printf("  (none — config was already well-tuned)\n");

    return 0;
}
