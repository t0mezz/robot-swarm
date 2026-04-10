#pragma once
// ─── Parameter space for AprilTag detector calibration ───────────────────────
// Defines which AprilConfig fields to optimise and their search bounds.
// To add/remove a parameter: edit kParams[] and the encode()/decode() bodies
// (one entry each). Nothing else needs changing.

#include "../april_vision.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

struct ParamSpec {
    const char* key;
    double      lo, hi;
    bool        logScale;
};

// ─── Search space ─────────────────────────────────────────────────────────────
// AprilTags have stronger error correction than ArUco so the error_corr range
// is shifted slightly lower.  All other bounds are the same.

static const ParamSpec kParams[] = {
    // key               lo       hi     log
    { "thresh_c",        3.0,    15.0,  false },
    { "win_max",        11.0,    31.0,  false },  // decoded → nearest odd int
    { "win_step",        1.0,     4.0,  false },  // decoded → int
    { "min_perim_rate",  0.010,   0.06, false },
    { "poly_approx",     0.020,   0.10, false },
    { "error_corr",      0.40,    0.90, false },  // AprilTag: slightly wider range
    { "min_otsu_stddev", 2.0,    12.0,  false },
    { "clahe_clip",      0.5,     5.0,  false },
    { "kf_proc_vel",     0.001,   0.10, true  },
    { "kf_meas",         1.0,    20.0,  false },
    { "roi_pad",         1.0,     2.0,  false },
};
static constexpr int kNParams = (int)(sizeof(kParams) / sizeof(kParams[0]));

// ─── Normalisation helpers ────────────────────────────────────────────────────

inline double toNorm(double v, const ParamSpec& p) {
    double lo = p.logScale ? std::log(p.lo) : p.lo;
    double hi = p.logScale ? std::log(p.hi) : p.hi;
    return ((p.logScale ? std::log(v) : v) - lo) / (hi - lo);
}

inline double fromNorm(double t, const ParamSpec& p) {
    double lo = p.logScale ? std::log(p.lo) : p.lo;
    double hi = p.logScale ? std::log(p.hi) : p.hi;
    double v  = lo + std::clamp(t, 0.0, 1.0) * (hi - lo);
    return p.logScale ? std::exp(v) : v;
}

// ─── Encode / Decode ──────────────────────────────────────────────────────────

inline std::vector<double> encode(const AprilConfig& c) {
    return {
        toNorm(c.threshC,       kParams[0]),
        toNorm(c.winMax,        kParams[1]),
        toNorm(c.winStep,       kParams[2]),
        toNorm(c.minPerimRate,  kParams[3]),
        toNorm(c.polyApprox,    kParams[4]),
        toNorm(c.errorCorr,     kParams[5]),
        toNorm(c.minOtsuStdDev, kParams[6]),
        toNorm(c.claheClip,     kParams[7]),
        toNorm(c.kfProcVel,     kParams[8]),
        toNorm(c.kfMeas,        kParams[9]),
        toNorm(c.roiPad,        kParams[10]),
    };
}

inline AprilConfig decode(const std::vector<double>& x, AprilConfig cfg) {
    cfg.threshC       = (float)fromNorm(x[0],  kParams[0]);
    cfg.winMax        = (int)std::round(fromNorm(x[1], kParams[1]));
    if (cfg.winMax % 2 == 0) cfg.winMax++;
    cfg.winStep       = std::max(1, (int)std::round(fromNorm(x[2], kParams[2])));
    cfg.minPerimRate  = (float)fromNorm(x[3],  kParams[3]);
    cfg.polyApprox    = (float)fromNorm(x[4],  kParams[4]);
    cfg.errorCorr     = (float)fromNorm(x[5],  kParams[5]);
    cfg.minOtsuStdDev = (float)fromNorm(x[6],  kParams[6]);
    cfg.claheClip     = (float)fromNorm(x[7],  kParams[7]);
    cfg.kfProcVel     = (float)fromNorm(x[8],  kParams[8]);
    cfg.kfMeas        = (float)fromNorm(x[9],  kParams[9]);
    cfg.roiPad        = (float)fromNorm(x[10], kParams[10]);
    cfg.winStep = std::min(cfg.winStep, std::max(1, (cfg.winMax - cfg.winMin) / 2));
    return cfg;
}

// ─── Detector factory ─────────────────────────────────────────────────────────

inline cv::aruco::ArucoDetector makeDetector(const AprilConfig& cfg) {
    auto dict   = cv::aruco::getPredefinedDictionary(cfg.dictId);
    auto params = cv::aruco::DetectorParameters();
    params.adaptiveThreshWinSizeMin              = cfg.winMin;
    params.adaptiveThreshWinSizeMax              = cfg.winMax;
    params.adaptiveThreshWinSizeStep             = cfg.winStep;
    params.adaptiveThreshConstant                = cfg.threshC;
    params.minMarkerPerimeterRate                = cfg.minPerimRate;
    params.polygonalApproxAccuracyRate           = cfg.polyApprox;
    params.perspectiveRemovePixelPerCell         = cfg.pixPerCell;
    params.perspectiveRemoveIgnoredMarginPerCell = cfg.cellMargin;
    params.errorCorrectionRate                   = cfg.errorCorr;
    params.minOtsuStdDev                         = cfg.minOtsuStdDev;
    params.minMarkerDistanceRate                 = 0.0;
    params.cornerRefinementMethod                = cv::aruco::CORNER_REFINE_SUBPIX;
    params.cornerRefinementWinSize               = cfg.cornerWin;
    params.cornerRefinementMaxIterations         = cfg.cornerMaxIter;
    return cv::aruco::ArucoDetector(dict, params);
}

// ─── Config serialisation ─────────────────────────────────────────────────────

inline void writeConfig(const AprilConfig& cfg, const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) {
        fprintf(stderr, "[april-calib] cannot write config to '%s'\n", path.c_str());
        return;
    }
    fs << "cam_index"       << cfg.camIndex
       << "cam_width"       << cfg.width
       << "cam_height"      << cfg.height
       << "cam_fps"         << cfg.fps
       << "dict_id"         << cfg.dictId
       << "win_min"         << cfg.winMin
       << "win_max"         << cfg.winMax
       << "win_step"        << cfg.winStep
       << "thresh_c"        << cfg.threshC
       << "min_perim_rate"  << cfg.minPerimRate
       << "poly_approx"     << cfg.polyApprox
       << "pix_per_cell"    << cfg.pixPerCell
       << "cell_margin"     << cfg.cellMargin
       << "error_corr"      << cfg.errorCorr
       << "min_otsu_stddev" << cfg.minOtsuStdDev
       << "corner_win"      << cfg.cornerWin
       << "corner_max_iter" << cfg.cornerMaxIter
       << "half_res_sweep"  << (int)cfg.halfResSweep
       << "kf_proc_pos"     << cfg.kfProcPos
       << "kf_proc_vel"     << cfg.kfProcVel
       << "kf_meas"         << cfg.kfMeas
       << "kf_init_cov"     << cfg.kfInitCov
       << "roi_pad"         << cfg.roiPad
       << "roi_grow"        << cfg.roiGrow
       << "roi_fail_max"    << cfg.roiFailMax
       << "roi_area_max"    << cfg.roiAreaMax
       << "global_reset"    << cfg.globalReset
       << "clahe_clip"      << cfg.claheClip
       << "clahe_tile"      << cfg.claheTile
       << "debug_overlay"   << (int)cfg.debugOverlay;
}
