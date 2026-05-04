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

#include <cstdio>
#include <cmath>
#include <string>
#include <memory>
#include <algorithm>

#include "cmaes.h"
#include "param_space.h"
#include "objective.h"
#include "objective_static.h"

// ─── Argument parsing ─────────────────────────────────────────────────────────

struct Args {
    int         camIdx  = 0;
    bool        eval    = false;
    int         idsMax  = -1;
    int         maxIter = 150;
    double      sigma0  = 0.30;
    std::string config  = "../../aruco_tracker_config.json";
    std::string output;
};

static Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        if      (k == "--eval")                a.eval    = true;
        else if (i + 1 < argc) {
            if      (k == "--camera" || k == "--cam") a.camIdx  = std::stoi(argv[++i]);
            else if (k == "--ids")    a.idsMax  = std::stoi(argv[++i]);
            else if (k == "--iters")  a.maxIter = std::stoi(argv[++i]);
            else if (k == "--sigma")  a.sigma0  = std::stod(argv[++i]);
            else if (k == "--config") a.config  = argv[++i];
            else if (k == "--output") a.output  = argv[++i];
            else fprintf(stderr, "Unknown option: %s\n", k.c_str());
        }
    }
    if (a.output.empty()) a.output = a.config;
    return a;
}

// ─── Interactive capture ──────────────────────────────────────────────────────
// Opens the camera and shows a live preview.  SPACE captures the current frame;
// ENTER or Q closes the preview and returns all captured frames.
// An overlay shows how many frames have been captured so far.

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

static std::vector<cv::Mat> captureInteractive(int camIdx, const ArucoConfig& cfg) {
    cv::VideoCapture cap;
    cap.open(camIdx, cv::CAP_AVFOUNDATION);
    if (!cap.isOpened()) {
        fprintf(stderr, "[calib] cannot open camera %d\n", camIdx);
        return {};
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  cfg.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
    cap.set(cv::CAP_PROP_FPS,          cfg.fps);

    for (int i = 0; i < 5; ++i) { cv::Mat tmp; cap.read(tmp); }

    const char* WIN = "Calibration  —  SPACE: capture frame  |  ENTER / Q: done";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN, cfg.width, cfg.height);

    printf("Place all ArUco markers in view.\n");
    printf("SPACE: capture a frame  |  ENTER or Q: start optimisation\n");
    fflush(stdout);

    std::vector<cv::Mat> frames;
    cv::Mat frame;

    while (true) {
        cap.read(frame);
        if (frame.empty()) { cv::waitKey(10); continue; }

        cv::Mat display = frame.clone();
        std::string label = "Captured: " + std::to_string(frames.size());
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
    cap.release();
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
    printf("Camera: %d\n", args.camIdx);
    if (!args.eval)
        printf("MaxIter: %d  Sigma0: %.2f\n", args.maxIter, args.sigma0);
    printf("Base config:   %s\n", args.config.c_str());
    printf("Output config: %s\n\n", args.output.c_str());

    ArucoConfig base  = ArucoConfig::fromFile(args.config);
    base.camIndex     = args.camIdx;
    base.debugOverlay = false;

    std::vector<int> expectedIds;
    if (args.idsMax >= 0) {
        for (int i = 0; i <= args.idsMax; ++i) expectedIds.push_back(i);
        printf("Expected IDs (from --ids): ");
        for (int id : expectedIds) printf("%d ", id);
        printf("\n");
    }

    // ── Capture phase ─────────────────────────────────────────────────────────
    auto frames = captureInteractive(args.camIdx, base);
    if (frames.empty()) {
        fprintf(stderr, "Error: no frames captured.\n");
        return 1;
    }
    if ((int)frames.size() < 5)
        printf("[calib] Warning: only %d frame(s) captured — 5 or more recommended.\n",
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
