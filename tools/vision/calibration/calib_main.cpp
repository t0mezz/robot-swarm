// tools/calibration/calib_main.cpp
// Tunes aruco_tracker_config.json for the current lighting environment.
//
// Architecture:
//   IOptimizer  — CMAES                  future: gp_ard.h
//   IObjective  — StaticObjective         select with --mode
//                 MotionObjective         future: objective_fisheye.h
//   param_space — encode/decode/write     shared by all optimizers and objectives
//
// Usage:
//   ./calibrate [options]
//     --static                optimise for a still scene (default)
//     --motion                optimise for moving robots
//     --eval                  evaluate the current config only (no optimisation)
//     --camera, --cam  <idx>  camera index (default: 0)
//     --frames  <n>           frames to capture once before optimisation begins
//                             (default: 60 for static, 120 for motion/eval)
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
// How iterations work:
//   Frames are captured ONCE at startup and stored in memory.  Every CMA-ES
//   generation then runs the detector on those cached frames — the camera is
//   never accessed again during optimisation.  With lambda≈11 candidates,
//   60 frames, and ~1 ms per detection, one generation takes ~660 ms and a
//   full 150-generation run completes in roughly 2 minutes.

#include <cstdio>
#include <cmath>
#include <string>
#include <memory>
#include <algorithm>

#include "cmaes.h"
#include "param_space.h"
#include "objective.h"
#include "objective_static.h"
#include "objective_motion.h"
// Future drop-in:
//   #include "objective_fisheye.h"
//   #include "gp_ard.h"

// ─── Argument parsing ─────────────────────────────────────────────────────────

struct Args {
    int         camIdx  = 0;
    bool        motion  = false;     // --motion sets this; default is static
    bool        eval    = false;     // --eval: score baseline only, no optimisation
    int         nFrames = -1;        // -1 = use mode default
    int         idsMax  = -1;        // --ids N: expect IDs 0…N; -1 = auto-detect
    int         maxIter = 150;
    double      sigma0  = 0.30;
    std::string config  = "../../aruco_tracker_config.json";
    std::string output;              // defaults to config if empty
};

static Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        if      (k == "--static")              a.motion  = false;
        else if (k == "--motion")              a.motion  = true;
        else if (k == "--eval")                a.eval    = true;
        else if (i + 1 < argc) {
            if      (k == "--camera" || k == "--cam") a.camIdx  = std::stoi(argv[++i]);
            else if (k == "--frames") a.nFrames = std::stoi(argv[++i]);
            else if (k == "--ids")    a.idsMax  = std::stoi(argv[++i]);
            else if (k == "--iters")  a.maxIter = std::stoi(argv[++i]);
            else if (k == "--sigma")  a.sigma0  = std::stod(argv[++i]);
            else if (k == "--config") a.config  = argv[++i];
            else if (k == "--output") a.output  = argv[++i];
            else fprintf(stderr, "Unknown option: %s\n", k.c_str());
        }
    }
    if (a.output.empty()) a.output = a.config;
    if (a.nFrames < 0)    a.nFrames = (a.motion || a.eval) ? 120 : 60;
    return a;
}

// ─── Live preview ────────────────────────────────────────────────────────────
// Opens the camera, streams frames into a window, and returns once the user
// presses ENTER (either in the terminal or as key 13/10 in the preview window).
// Uses a background thread to drain stdin so both inputs work simultaneously.

#include <opencv2/highgui.hpp>
#include <atomic>
#include <thread>

static void showPreviewUntilEnter(int camIdx, const ArucoConfig& cfg) {
    cv::VideoCapture cap;
    cap.open(camIdx, cv::CAP_AVFOUNDATION);
    if (!cap.isOpened()) {
        fprintf(stderr, "[calib] preview: cannot open camera %d\n", camIdx);
        getchar();
        return;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  cfg.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
    cap.set(cv::CAP_PROP_FPS,          cfg.fps);

    // Drain warm-up frames
    for (int i = 0; i < 5; ++i) { cv::Mat tmp; cap.read(tmp); }

    const char* WIN = "Calibration preview  —  press ENTER to start";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN, cfg.width, cfg.height);

    // Background thread: block on stdin ENTER, then signal the loop to stop.
    std::atomic<bool> enterPressed{false};
    std::thread stdinThread([&enterPressed]() {
        getchar();
        enterPressed.store(true);
    });
    stdinThread.detach();

    cv::Mat frame;
    while (!enterPressed.load()) {
        if (cap.read(frame) && !frame.empty())
            cv::imshow(WIN, frame);
        int key = cv::waitKey(30);
        if (key == 13 || key == 10)   // ENTER in the preview window
            enterPressed.store(true);
    }

    cv::destroyWindow(WIN);
    cap.release();
}

// ─── Optimisation loop ────────────────────────────────────────────────────────
// Decoupled from concrete optimizer and objective types.
// Swapping CMA-ES for GP-ARD only requires changing the instantiation in main().

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

    printf("=== ArUco Tracker Calibration (CMA-ES / %s) ===\n",
           args.eval ? "eval" : args.motion ? "motion" : "static");
    printf("Camera: %d  Frames: %d\n", args.camIdx, args.nFrames);
    if (!args.eval)
        printf("MaxIter: %d  Sigma0: %.2f\n", args.maxIter, args.sigma0);
    printf("Base config:   %s\n", args.config.c_str());
    printf("Output config: %s\n\n", args.output.c_str());

    ArucoConfig base  = ArucoConfig::fromFile(args.config);
    base.camIndex     = args.camIdx;
    base.debugOverlay = false;

    // Build the explicit ID list if --ids was given.
    std::vector<int> expectedIds;
    if (args.idsMax >= 0) {
        for (int i = 0; i <= args.idsMax; ++i) expectedIds.push_back(i);
        printf("Expected IDs (from --ids): ");
        for (int id : expectedIds) printf("%d ", id);
        printf("\n");
    }

    // ── Capture phase ─────────────────────────────────────────────────────────
    std::unique_ptr<IObjective> obj;

    if (!args.motion) {
        printf("Place all ArUco markers in view and keep them stationary.\n");
        printf("Press ENTER (terminal or preview window) to start capturing %d frames...\n", args.nFrames);
        fflush(stdout);
        showPreviewUntilEnter(args.camIdx, base);
        auto so = std::make_unique<StaticObjective>();
        if (!so->capture(args.camIdx, args.nFrames, base, expectedIds)) {
            fprintf(stderr, "Error: camera unavailable or no markers detected.\n");
            return 1;
        }
        obj = std::move(so);
    } else {
        printf("Place all ArUco markers in view.\n");
        printf("Press ENTER (terminal or preview window) when ready — robots should start moving on GO...\n");
        fflush(stdout);
        showPreviewUntilEnter(args.camIdx, base);
        auto mo = std::make_unique<MotionObjective>();
        if (!mo->capture(args.camIdx, args.nFrames, base, expectedIds)) {
            fprintf(stderr, "Error: camera unavailable or no markers detected.\n");
            return 1;
        }
        obj = std::move(mo);
    }

    // ── Baseline score ────────────────────────────────────────────────────────
    double baseScore = obj->evaluate(base);
    printf("\nBaseline score: %.1f%%\n", -baseScore * 100.0);

    if (args.eval) {
        printf("\n(--eval mode: no optimisation run)\n");
        return 0;
    }

    // ── Optimisation phase ────────────────────────────────────────────────────
    printf("\nStarting CMA-ES  (n=%d, lambda=%d, max_gen=%d)\n",
           kNParams, 4 + (int)(3 * std::log(kNParams)), args.maxIter);

    // Swap CMAES for GPARD here when gp_ard.h is ready.
    CMAES opt(kNParams, args.sigma0);
    runOptimisation(opt, *obj, base, args.maxIter);

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
