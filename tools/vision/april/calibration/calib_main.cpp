// tools/april-vision/calibration/calib_main.cpp
// Tunes april_vision_config.json for the current lighting environment.
//
// Architecture mirrors tools/calibration/calib_main.cpp — same CMA-ES loop,
// same objectives (static / motion), same param space shape — but operates
// on AprilConfig and april_vision_config.json instead of ArucoConfig.
//
// Usage:
//   ./april_calibrate [options]
//     --static                optimise for a still scene (default)
//     --motion                optimise for moving robots
//     --eval                  evaluate the current config only (no optimisation)
//     --camera, --cam  <idx>  camera index (default: 0)
//     --frames  <n>           frames to capture (default: 60 static, 120 motion)
//     --ids     <n>           expect IDs 0…n (e.g. --ids 2 → markers 0,1,2)
//     --iters   <n>           CMA-ES generations (default: 150)
//     --sigma   <f>           initial step size in [0,1] space (default: 0.30)
//     --config  <path>        base config (default: ../april_vision_config.json)
//     --output  <path>        where to write result (default: same as --config)

#include <cstdio>
#include <cmath>
#include <string>
#include <memory>
#include <algorithm>

#include "../../calibration/cmaes.h"
#include "param_space.h"
#include "objective.h"
#include "objective_static.h"
#include "objective_motion.h"

// ─── Argument parsing ─────────────────────────────────────────────────────────

struct Args {
    int         camIdx  = 0;
    bool        motion  = false;
    bool        eval    = false;
    int         nFrames = -1;
    int         idsMax  = -1;
    int         maxIter = 150;
    double      sigma0  = 0.30;
    std::string config  = "../april_vision_config.json";
    std::string output;
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

// ─── Live preview ─────────────────────────────────────────────────────────────

#include <opencv2/highgui.hpp>
#include <atomic>
#include <thread>

static void showPreviewUntilEnter(int camIdx, const AprilConfig& cfg) {
    cv::VideoCapture cap;
    cap.open(camIdx, cv::CAP_AVFOUNDATION);
    if (!cap.isOpened()) {
        fprintf(stderr, "[april-calib] preview: cannot open camera %d\n", camIdx);
        getchar();
        return;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  cfg.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
    cap.set(cv::CAP_PROP_FPS,          cfg.fps);

    for (int i = 0; i < 5; ++i) { cv::Mat tmp; cap.read(tmp); }

    const char* WIN = "Calibration preview  —  press ENTER to start";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN, cfg.width, cfg.height);

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
        if (key == 13 || key == 10)
            enterPressed.store(true);
    }

    cv::destroyWindow(WIN);
    cap.release();
}

// ─── Optimisation loop ────────────────────────────────────────────────────────

static void runOptimisation(IOptimizer&         opt,
                             IObjective&         obj,
                             const AprilConfig&  base,
                             int                 maxIter) {
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

    printf("=== AprilTag Tracker Calibration (CMA-ES / %s) ===\n",
           args.eval ? "eval" : args.motion ? "motion" : "static");
    printf("Camera: %d  Frames: %d\n", args.camIdx, args.nFrames);
    if (!args.eval)
        printf("MaxIter: %d  Sigma0: %.2f\n", args.maxIter, args.sigma0);
    printf("Base config:   %s\n", args.config.c_str());
    printf("Output config: %s\n\n", args.output.c_str());

    AprilConfig base  = AprilConfig::fromFile(args.config);
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
    std::unique_ptr<IObjective> obj;

    if (!args.motion) {
        printf("Place all AprilTag markers in view and keep them stationary.\n");
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
        printf("Place all AprilTag markers in view.\n");
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

    CMAES opt(kNParams, args.sigma0);
    runOptimisation(opt, *obj, base, args.maxIter);

    // ── Write result ──────────────────────────────────────────────────────────
    AprilConfig bestCfg = decode(opt.bestX(), base);
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
