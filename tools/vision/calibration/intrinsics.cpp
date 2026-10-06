// tools/vision/calibration/intrinsics.cpp
// Calibrates the camera's lens (focal length, principal point, distortion) from
// a printed ChArUco board and writes camera_intrinsics.yml, which ArucoTracker
// then uses to undistort every tracked point.
//
// Rectilinear lens model (k1 k2 [k3] p1 p2) via cv::calibrateCamera — NOT the
// fisheye model. Make the board with make_charuco.py first.
//
// Usage:
//   ./intrinsics [options]
//     (no input option)       live capture from the Basler, then calibrate
//     --images <dir>          calibrate from image files instead (no camera)
//     --use-cache             reuse the frames the last live run saved
//     --cache-dir <dir>       frame cache (default: ./intrinsics_frames)
//     --verify                live preview of the saved calibration: toggle
//                             raw / undistorted with a straight-line grid
//     --board <json>          board geometry from make_charuco.py
//                             (default: tools/vision/calibration/charuco_board.json)
//     --square-mm <f>         MEASURED printed square size (caliper it!). The
//                             marker size is scaled by the same factor.
//     --output <path>         default: tools/vision/camera_intrinsics.yml
//     --focal-mm <f>          initial guess, lens focal length   (default 6)
//     --pixel-um <f>          initial guess, sensor pixel pitch  (default 2.2)
//     --cam-height-mm <f>     working distance; only used to express the
//                             distortion in mm at the arena plane (default 0=skip)
//     --k3                    also fit k3 (default fixed at 0: a 6 mm lens at
//                             ~40 degrees field of view does not need it, and a
//                             free k3 overfits unless the corners are well covered)
//     --zero-tangent          force p1 = p2 = 0
//     --free-aspect           fit fx and fy separately (default: square pixels)
//     --min-corners <n>       a view needs this many board corners (default 12)
//     --no-prune              keep views with a large reprojection error
//     --config <path>         tracker config for camera settings
//     --serial <sn>, --ip <ip>
//
// Capture keys:
//   SPACE   take this view          a   toggle auto-capture (waits for a new
//   u       drop the last view          pose, steady board, enough corners)
//   ENTER   finish and calibrate    q/Esc  quit without calibrating
//
// Move the board to *different places and tilts* — distortion is largest at
// the frame edges and corners, and a view only constrains the lens where its
// corners are. The coverage grid shows where the views are missing.
//
// The calibration is tied to the resolution and sensor ROI offset it was made
// with; the tracker ignores it (with a message) if either changes.

#include "aruco_tracker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <numeric>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// ─── Arguments ────────────────────────────────────────────────────────────────

struct Args {
    std::string serial, ip;
    std::string config   = ArucoConfig::defaultConfigPath();
    std::string imagesDir;
    bool        useCache = false;
    bool        verify   = false;
    std::string cacheDir = "intrinsics_frames";
    std::string boardFile = arucoVisionDataPath("calibration/charuco_board.json");
    std::string output    = arucoVisionDataPath("camera_intrinsics.yml");
    double      squareMm = -1;
    double      focalMm  = 6.0, pixelUm = 2.2, camHeightMm = 0;
    bool        k3 = false, zeroTangent = false, freeAspect = false, prune = true;
    int         minCorners = 12;
};

static bool parseArgs(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) { fprintf(stderr, "missing value for %s\n", k.c_str()); exit(1); }
            return argv[++i];
        };
        if      (k == "--serial")        a.serial = next();
        else if (k == "--ip")            a.ip = next();
        else if (k == "--config")        a.config = next();
        else if (k == "--images")        a.imagesDir = next();
        else if (k == "--use-cache")     a.useCache = true;
        else if (k == "--verify")        a.verify = true;
        else if (k == "--cache-dir")     a.cacheDir = next();
        else if (k == "--board")         a.boardFile = next();
        else if (k == "--output")        a.output = next();
        else if (k == "--square-mm")     a.squareMm = atof(next());
        else if (k == "--focal-mm")      a.focalMm = atof(next());
        else if (k == "--pixel-um")      a.pixelUm = atof(next());
        else if (k == "--cam-height-mm") a.camHeightMm = atof(next());
        else if (k == "--k3")            a.k3 = true;
        else if (k == "--zero-tangent")  a.zeroTangent = true;
        else if (k == "--free-aspect")   a.freeAspect = true;
        else if (k == "--no-prune")      a.prune = false;
        else if (k == "--min-corners")   a.minCorners = atoi(next());
        else if (k == "-h" || k == "--help") return false;
        else { fprintf(stderr, "unknown option %s\n", k.c_str()); return false; }
    }
    return true;
}

// ─── Board ────────────────────────────────────────────────────────────────────

struct BoardSpec {
    int    sx = 10, sy = 7;
    double squareMm = 36.0, markerMm = 26.87;
    int    dictId = cv::aruco::DICT_4X4_50;
};

static bool loadBoardSpec(const std::string& path, BoardSpec& b) {
    cv::FileStorage fs(path, cv::FileStorage::READ);   // reads the .json make_charuco.py writes
    if (!fs.isOpened()) return false;
    fs["squares_x"] >> b.sx;  fs["squares_y"] >> b.sy;
    fs["square_mm"] >> b.squareMm;  fs["marker_mm"] >> b.markerMm;
    fs["dict_id"]   >> b.dictId;
    return b.sx >= 3 && b.sy >= 3 && b.squareMm > 0 && b.markerMm > 0;
}

struct BoardDetector {
    cv::aruco::CharucoBoard    board;
    cv::aruco::CharucoDetector det;
    explicit BoardDetector(const BoardSpec& s)
        : board(cv::Size(s.sx, s.sy), (float)s.squareMm, (float)s.markerMm,
                cv::aruco::getPredefinedDictionary(s.dictId)),
          det(board, makeCharucoParams(), makeDetectorParams()) {}

    static cv::aruco::CharucoParameters makeCharucoParams() {
        cv::aruco::CharucoParameters p;
        p.tryRefineMarkers = false;
        return p;
    }
    static cv::aruco::DetectorParameters makeDetectorParams() {
        cv::aruco::DetectorParameters p;
        p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        // The board is large in the image; refine over a window sized to its cells.
        p.cornerRefinementWinSize = 5;
        return p;
    }

    // Board corners in this image: sub-pixel positions + their board ids.
    int detect(const cv::Mat& img, std::vector<cv::Point2f>& corners, std::vector<int>& ids) {
        cv::Mat gray;
        if (img.channels() == 3) cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY); else gray = img;
        corners.clear(); ids.clear();
        std::vector<cv::Point2f> c; std::vector<int> i;
        det.detectBoard(gray, c, i);
        corners = c; ids = i;
        return (int)ids.size();
    }
};

// ─── Frame cache ──────────────────────────────────────────────────────────────

static std::vector<cv::Mat> loadFrames(const std::string& dir) {
    std::vector<fs::path> paths;
    if (fs::exists(dir))
        for (auto& e : fs::directory_iterator(dir)) {
            auto ext = e.path().extension().string();
            for (auto& c : ext) c = (char)tolower(c);
            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" || ext == ".tif")
                paths.push_back(e.path());
        }
    std::sort(paths.begin(), paths.end());
    std::vector<cv::Mat> frames;
    for (auto& p : paths) {
        cv::Mat f = cv::imread(p.string(), cv::IMREAD_COLOR);
        if (f.empty()) fprintf(stderr, "[intrinsics] cannot read %s\n", p.c_str());
        else           frames.push_back(f);
    }
    return frames;
}

static void saveFrames(const std::string& dir, const std::vector<cv::Mat>& frames) {
    fs::create_directories(dir);
    for (auto& e : fs::directory_iterator(dir))
        if (e.path().filename().string().rfind("frame_", 0) == 0 && e.path().extension() == ".png")
            fs::remove(e.path());
    for (size_t i = 0; i < frames.size(); ++i) {
        char name[64]; snprintf(name, sizeof(name), "frame_%03d.png", (int)i);
        cv::imwrite((fs::path(dir) / name).string(), frames[i]);
    }
    printf("[intrinsics] saved %d frame(s) to %s (re-run with --use-cache)\n",
           (int)frames.size(), dir.c_str());
}

// ─── Live capture ─────────────────────────────────────────────────────────────

// How a view differs from the ones already taken. A new view is only worth
// taking if the board moved to a new place, size or tilt.
struct ViewKey { cv::Point2f centre; double area, aspect; };

static ViewKey viewKey(const std::vector<cv::Point2f>& pts) {
    cv::Rect2f r = cv::boundingRect(pts);
    cv::RotatedRect rr = cv::minAreaRect(pts);
    double a = std::max(rr.size.width, rr.size.height), b = std::min(rr.size.width, rr.size.height);
    return {{r.x + r.width / 2, r.y + r.height / 2}, (double)r.area(), b / std::max(1.0, a)};
}

static bool isNovel(const ViewKey& k, const std::vector<ViewKey>& taken, cv::Size sz) {
    for (auto& t : taken) {
        double dc = cv::norm(k.centre - t.centre) / sz.width;
        double ar = k.area / std::max(1.0, t.area);
        double da = std::fabs(k.aspect - t.aspect);
        if (dc < 0.10 && ar > 0.8 && ar < 1.25 && da < 0.12) return false;
    }
    return true;
}

// Coverage: 8x8 cells of the frame, which hold at least one board corner.
static constexpr int GRID = 8;
static void addCoverage(std::vector<int>& cov, const std::vector<cv::Point2f>& pts, cv::Size sz) {
    for (auto& p : pts) {
        int gx = std::clamp((int)(p.x * GRID / sz.width), 0, GRID - 1);
        int gy = std::clamp((int)(p.y * GRID / sz.height), 0, GRID - 1);
        cov[gy * GRID + gx]++;
    }
}

static std::vector<cv::Mat> captureLive(const ArucoConfig& cfg, BoardDetector& bd, int minCorners) {
    BaslerPylonSource cam;
    if (!cam.open(cfg)) { fprintf(stderr, "[intrinsics] cannot open Basler camera\n"); return {}; }
    for (int i = 0; i < 5; ++i) { cv::Mat t; cam.read(t); }

    const char* WIN = "Intrinsics - SPACE take view | a auto | u undo | ENTER calibrate | q quit";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    const double dscale = std::min(1.0, 1100.0 / std::max(cfg.width, cfg.height));
    cv::resizeWindow(WIN, (int)(cfg.width * dscale), (int)(cfg.height * dscale));

    printf("Hold the board flat and still. Spread views over the WHOLE frame, tilt it\n"
           "(up to ~45 degrees) and bring it into every corner; red cells have no data.\n"
           "SPACE: take view | a: auto-capture | u: undo | ENTER: calibrate | q: quit\n");

    std::vector<cv::Mat>     frames;
    std::vector<ViewKey>     keys;
    std::vector<int>         cov(GRID * GRID, 0);
    std::vector<std::vector<cv::Point2f>> viewPts;
    bool   autoCap = false;
    auto   lastTaken = std::chrono::steady_clock::now();
    cv::Point2f lastCentre(-1e9f, -1e9f);
    int    steady = 0;

    cv::Size fsz(cfg.width, cfg.height);   // replaced by the real frame size on the first frame
    auto rebuildCov = [&]() {
        std::fill(cov.begin(), cov.end(), 0);
        for (auto& v : viewPts) addCoverage(cov, v, fsz);
    };

    cv::Mat frame;
    while (true) {
        if (!cam.read(frame) || frame.empty()) { cv::waitKey(10); continue; }
        const cv::Size sz = frame.size();
        fsz = sz;

        std::vector<cv::Point2f> corners; std::vector<int> ids;
        int n = bd.detect(frame, corners, ids);
        bool ok = n >= minCorners;

        // "Steady" = the board centroid barely moved since the last frame, so
        // the view is not motion-blurred.
        ViewKey k{};
        if (ok) {
            k = viewKey(corners);
            steady = (cv::norm(k.centre - lastCentre) < 0.004 * sz.width) ? steady + 1 : 0;
            lastCentre = k.centre;
        } else steady = 0;

        auto now = std::chrono::steady_clock::now();
        bool take = false;
        if (autoCap && ok && steady >= 4 && isNovel(k, keys, sz) &&
            std::chrono::duration<double>(now - lastTaken).count() > 1.2) take = true;

        // ── Draw ──────────────────────────────────────────────────────────
        cv::Mat disp;
        cv::resize(frame, disp, {}, dscale, dscale, cv::INTER_AREA);
        for (int gy = 0; gy < GRID; ++gy)
            for (int gx = 0; gx < GRID; ++gx) {
                cv::Rect r((int)(gx * disp.cols / (double)GRID), (int)(gy * disp.rows / (double)GRID),
                           disp.cols / GRID, disp.rows / GRID);
                cv::Scalar col = cov[gy * GRID + gx] ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 220);
                cv::Mat roi = disp(r);
                cv::Mat tint(roi.size(), roi.type(), col);
                cv::addWeighted(roi, cov[gy * GRID + gx] ? 0.9 : 0.8, tint,
                                cov[gy * GRID + gx] ? 0.1 : 0.2, 0, roi);
                cv::rectangle(disp, r, {255, 255, 255}, 1);
            }
        for (auto& c : corners)
            cv::circle(disp, c * (float)dscale, 4, ok ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 128, 255), -1);
        int covered = (int)std::count_if(cov.begin(), cov.end(), [](int c) { return c > 0; });
        char line[200];
        snprintf(line, sizeof(line), "views %d | corners %d/%d%s | coverage %d/%d | %s",
                 (int)frames.size(), n, (bd.board.getChessboardSize().width - 1) *
                 (bd.board.getChessboardSize().height - 1), ok ? "" : " (too few)",
                 covered, GRID * GRID, autoCap ? "AUTO" : "manual");
        cv::putText(disp, line, {12, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {0, 0, 0}, 4, cv::LINE_AA);
        cv::putText(disp, line, {12, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {0, 255, 0}, 2, cv::LINE_AA);
        cv::imshow(WIN, disp);

        int key = cv::waitKey(1);
        if (key == ' ') {
            if (ok) take = true; else printf("[intrinsics] only %d corners — not taken\n", n);
        } else if (key == 'a') { autoCap = !autoCap; printf("[intrinsics] auto-capture %s\n", autoCap ? "on" : "off"); }
        else if (key == 'u' && !frames.empty()) {
            frames.pop_back(); keys.pop_back(); viewPts.pop_back(); rebuildCov();
            printf("[intrinsics] dropped last view (%d left)\n", (int)frames.size());
        } else if (key == 13 || key == 10) break;
        else if (key == 'q' || key == 27) { frames.clear(); break; }

        if (take) {
            frames.push_back(frame.clone());
            keys.push_back(k);
            viewPts.push_back(corners);
            rebuildCov();
            lastTaken = now;
            printf("[intrinsics] view %d taken (%d corners)\n", (int)frames.size(), n);
            fflush(stdout);
        }
    }
    cv::destroyWindow(WIN);
    return frames;
}

// ─── Calibration ──────────────────────────────────────────────────────────────

struct View {
    int index;                              // position in the frame list (for reporting)
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    std::vector<cv::Point2f> corners;       // all detected, for coverage
    double err = 0;
};

struct CalibResult {
    cv::Mat K, D;
    double  rms = 0;
    cv::Mat stdIntr;
    std::vector<View> views;
    std::vector<int>  dropped;
};

static double calibrateOnce(std::vector<View>& views, cv::Size sz, const Args& a, CalibResult& out) {
    std::vector<std::vector<cv::Point3f>> obj;
    std::vector<std::vector<cv::Point2f>> img;
    for (auto& v : views) { obj.push_back(v.obj); img.push_back(v.img); }

    double fpx = a.focalMm * 1000.0 / a.pixelUm;
    cv::Mat K = (cv::Mat_<double>(3, 3) << fpx, 0, sz.width / 2.0, 0, fpx, sz.height / 2.0, 0, 0, 1);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);

    int flags = cv::CALIB_USE_INTRINSIC_GUESS;
    if (!a.freeAspect)  flags |= cv::CALIB_FIX_ASPECT_RATIO;
    if (!a.k3)          flags |= cv::CALIB_FIX_K3;
    if (a.zeroTangent)  flags |= cv::CALIB_ZERO_TANGENT_DIST;

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat stdI, stdE, perView;
    double rms = cv::calibrateCamera(obj, img, sz, K, D, rvecs, tvecs, stdI, stdE, perView, flags,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-9));
    for (size_t i = 0; i < views.size(); ++i) views[i].err = perView.at<double>((int)i);
    out.K = K; out.D = D.reshape(1, 1); out.rms = rms; out.stdIntr = stdI;
    return rms;
}

static CalibResult calibrate(const std::vector<cv::Mat>& frames, BoardDetector& bd,
                             const Args& a) {
    CalibResult res;
    if (frames.empty()) return res;
    const cv::Size sz = frames[0].size();

    std::vector<View> views;
    printf("\nDetecting the board in %d frame(s)...\n", (int)frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        if (frames[i].size() != sz) {
            fprintf(stderr, "  frame %zu is %dx%d, expected %dx%d — skipped\n", i,
                    frames[i].cols, frames[i].rows, sz.width, sz.height);
            continue;
        }
        std::vector<cv::Point2f> c; std::vector<int> ids;
        int n = bd.detect(frames[i], c, ids);
        if (n < a.minCorners) {
            printf("  frame %2zu: %d corners — below --min-corners %d, skipped\n", i, n, a.minCorners);
            continue;
        }
        View v; v.index = (int)i; v.corners = c;
        bd.board.matchImagePoints(c, ids, v.obj, v.img);
        if (v.obj.size() < 6) continue;
        views.push_back(std::move(v));
    }
    if (views.size() < 5) {
        fprintf(stderr, "Only %zu usable view(s); need at least 5 (10-25 recommended).\n", views.size());
        return res;
    }

    calibrateOnce(views, sz, a, res);

    // Drop views that don't fit the lens model (board bent, blurred, or
    // detected badly). Their error is much larger than the median's.
    if (a.prune) {
        for (int round = 0; round < 3 && views.size() > 8; ++round) {
            std::vector<double> e; for (auto& v : views) e.push_back(v.err);
            std::vector<double> s = e; std::sort(s.begin(), s.end());
            double med = s[s.size() / 2];
            double lim = std::max(2.5 * med, 0.4);
            auto worst = std::max_element(e.begin(), e.end());
            if (*worst <= lim) break;
            size_t wi = (size_t)(worst - e.begin());
            printf("  dropping frame %d: error %.2f px (median %.2f)\n", views[wi].index, *worst, med);
            res.dropped.push_back(views[wi].index);
            views.erase(views.begin() + (long)wi);
            calibrateOnce(views, sz, a, res);
        }
    }
    res.views = views;
    return res;
}

// ─── Report ───────────────────────────────────────────────────────────────────

static void report(const CalibResult& r, cv::Size sz, const Args& a) {
    const double fx = r.K.at<double>(0, 0), fy = r.K.at<double>(1, 1);
    const double cx = r.K.at<double>(0, 2), cy = r.K.at<double>(1, 2);
    auto sd = [&](int i) { return r.stdIntr.at<double>(i); };

    printf("\n── Per view ────────────────────────────────\n");
    printf("  frame  corners  error(px)\n");
    for (auto& v : r.views)
        printf("  %5d  %7zu  %8.3f%s\n", v.index, v.obj.size(), v.err, v.err > 0.4 ? "  <-- high" : "");

    printf("\n── Result (%d views, %dx%d) ─────────────────\n", (int)r.views.size(), sz.width, sz.height);
    printf("  RMS reprojection error : %.3f px\n", r.rms);
    printf("  fx = %8.2f ± %.2f   fy = %8.2f ± %.2f      (guess %.1f from %.1f mm / %.1f um)\n",
           fx, sd(0), fy, sd(1), a.focalMm * 1000 / a.pixelUm, a.focalMm, a.pixelUm);
    printf("  cx = %8.2f ± %.2f   cy = %8.2f ± %.2f      (frame centre %.1f, %.1f)\n",
           cx, sd(2), cy, sd(3), sz.width / 2.0, sz.height / 2.0);
    const double* d = r.D.ptr<double>();
    const char* names[5] = {"k1", "k2", "p1", "p2", "k3"};
    for (int i = 0; i < 5; ++i)
        printf("  %s = %10.5f ± %.5f%s\n", names[i], d[i], sd(4 + i),
               (i == 4 && !a.k3) || ((i == 2 || i == 3) && a.zeroTangent) ? "   (fixed)" : "");
    printf("  focal length: %.3f mm   (nominal %.1f mm)\n", fx * a.pixelUm / 1000.0, a.focalMm);

    // How much does the lens move a pixel? Sample the frame on a grid.
    double maxDisp = 0, cornerDisp = 0;
    cv::Point2f where;
    for (int j = 0; j <= 16; ++j)
        for (int i = 0; i <= 16; ++i) {
            cv::Point2f p((float)(i * (sz.width - 1) / 16.0), (float)(j * (sz.height - 1) / 16.0));
            std::vector<cv::Point2f> in{p}, out;
            cv::undistortPoints(in, out, r.K, r.D, cv::noArray(), r.K,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-10));
            double dd = cv::norm(out[0] - p);
            if (dd > maxDisp) { maxDisp = dd; where = p; }
            if ((i == 0 || i == 16) && (j == 0 || j == 16)) cornerDisp = std::max(cornerDisp, dd);
        }
    printf("\n── What the correction does ────────────────\n");
    printf("  largest shift of a pixel : %.2f px at (%.0f, %.0f)\n", maxDisp, where.x, where.y);
    printf("  shift at the frame corner: %.2f px\n", cornerDisp);
    if (a.camHeightMm > 0)
        printf("  = %.2f mm on the plane %.0f mm from the camera (largest shift)\n",
               maxDisp * a.camHeightMm / fx, a.camHeightMm);
    else
        printf("  (give --cam-height-mm to see this in mm at the arena)\n");

    // Coverage
    std::vector<int> cov(GRID * GRID, 0);
    for (auto& v : r.views) addCoverage(cov, v.corners, sz);
    int covered = (int)std::count_if(cov.begin(), cov.end(), [](int c) { return c > 0; });
    printf("\n── Coverage of the frame (corners per cell) ─\n");
    for (int y = 0; y < GRID; ++y) {
        printf("  ");
        for (int x = 0; x < GRID; ++x) printf("%4d", cov[y * GRID + x]);
        printf("\n");
    }

    printf("\n── Verdict ─────────────────────────────────\n");
    bool good = true;
    if (r.rms < 0.3)       printf("  RMS %.3f px: good.\n", r.rms);
    else if (r.rms < 0.6)  printf("  RMS %.3f px: acceptable. Flat board? sharp images?\n", r.rms);
    else { printf("  RMS %.3f px: poor — the board is probably bent, blurred, or the size is wrong.\n", r.rms); good = false; }
    if (covered < GRID * GRID * 3 / 4) {
        printf("  Only %d/%d frame cells have data; the edges and corners constrain distortion — "
               "take more views there.\n  The shifts above are EXTRAPOLATED into the empty cells "
               "and k2 in particular is not pinned down.\n", covered, GRID * GRID);
        good = false;
    }
    if (r.views.size() < 10) { printf("  Only %zu views; use 15-25.\n", r.views.size()); good = false; }
    if (std::fabs(fx * a.pixelUm / 1000.0 / a.focalMm - 1.0) > 0.05)
        printf("  Focal length is >5%% off the nominal %.1f mm — check --focal-mm/--pixel-um, "
               "or that the printed square size is right.\n", a.focalMm);
    if (maxDisp < 0.5)
        printf("  The lens is nearly distortion-free (%.2f px max): the correction is optional.\n", maxDisp);
    else if (maxDisp < 3.0)
        printf("  Small distortion (%.1f px): worth correcting for sub-mm accuracy.\n", maxDisp);
    else
        printf("  Significant distortion (%.1f px): correct it.\n", maxDisp);
    if (good) printf("  Looks trustworthy.\n");
}

static bool save(const CalibResult& r, cv::Size sz, const Args& a, const BoardSpec& b,
                 const ArucoConfig& cfg) {
    cv::FileStorage fs(a.output, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { fprintf(stderr, "cannot write %s\n", a.output.c_str()); return false; }
    char when[64]; std::time_t t = std::time(nullptr);
    std::strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    fs << "date" << when;
    fs << "K" << r.K << "D" << r.D;
    fs << "image_size" << sz;
    fs << "offset_x" << cfg.offsetX << "offset_y" << cfg.offsetY;
    fs << "rms" << r.rms << "num_views" << (int)r.views.size();
    fs << "std_intrinsics" << r.stdIntr;
    fs << "board_squares_x" << b.sx << "board_squares_y" << b.sy
       << "board_square_mm" << b.squareMm << "board_marker_mm" << b.markerMm;
    printf("\nSaved %s\n", a.output.c_str());
    printf("The tracker picks it up automatically next time it starts (use_intrinsics in the config).\n"
           "Homographies made without it are rejected — re-run tools/build/homography afterwards.\n");
    return true;
}

// ─── Verify ───────────────────────────────────────────────────────────────────

static int runVerify(const Args& a, const ArucoConfig& cfg) {
    BaslerPylonSource cam;
    if (!cam.open(cfg)) { fprintf(stderr, "cannot open camera\n"); return 1; }
    CameraIntrinsics I;
    cv::Mat probe;
    for (int i = 0; i < 5; ++i) cam.read(probe);
    if (!I.load(a.output, probe.size(), {cfg.offsetX, cfg.offsetY})) {
        fprintf(stderr, "no usable calibration at %s\n", a.output.c_str()); return 1;
    }
    cv::Mat m1, m2;
    cv::initUndistortRectifyMap(I.K, I.D, cv::Mat(), I.K, probe.size(), CV_16SC2, m1, m2);
    printf("u: raw/undistorted   g: grid   q: quit\nA straight edge should stay straight, "
           "especially near the frame border.\n");
    bool undist = true, grid = true;
    const char* WIN = "Intrinsics verify";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::Mat f;
    while (true) {
        if (!cam.read(f) || f.empty()) { cv::waitKey(10); continue; }
        cv::Mat d = f;
        if (undist) cv::remap(f, d, m1, m2, cv::INTER_LINEAR);
        double s = std::min(1.0, 1000.0 / std::max(d.cols, d.rows));
        cv::resize(d, d, {}, s, s, cv::INTER_AREA);
        if (grid) {
            for (int x = 0; x < d.cols; x += d.cols / 16) cv::line(d, {x, 0}, {x, d.rows}, {0, 255, 0}, 1);
            for (int y = 0; y < d.rows; y += d.rows / 16) cv::line(d, {0, y}, {d.cols, y}, {0, 255, 0}, 1);
        }
        cv::putText(d, undist ? "UNDISTORTED" : "RAW", {12, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
                    {0, 255, 255}, 2, cv::LINE_AA);
        cv::imshow(WIN, d);
        int k = cv::waitKey(1);
        if (k == 'u') undist = !undist; else if (k == 'g') grid = !grid;
        else if (k == 'q' || k == 27) break;
    }
    return 0;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    Args a;
    if (!parseArgs(argc, argv, a)) {
        fprintf(stderr, "see the comment at the top of intrinsics.cpp for options\n");
        return 1;
    }
    ArucoConfig cfg = ArucoConfig::fromFile(a.config);
    if (!a.serial.empty()) cfg.baslerSerial = a.serial;
    if (!a.ip.empty())     cfg.baslerIp     = a.ip;
    cfg.debugOverlay = false;

    if (a.verify) return runVerify(a, cfg);

    BoardSpec spec;
    if (!loadBoardSpec(a.boardFile, spec))
        fprintf(stderr, "[intrinsics] no board file at %s — using the make_charuco.py defaults\n",
                a.boardFile.c_str());
    if (a.squareMm > 0) {        // the printed board was measured; the marker scales with it
        spec.markerMm *= a.squareMm / spec.squareMm;
        spec.squareMm  = a.squareMm;
    }
    printf("Board: %dx%d squares, square %.3f mm, marker %.3f mm, dictionary %d\n",
           spec.sx, spec.sy, spec.squareMm, spec.markerMm, spec.dictId);
    BoardDetector bd(spec);

    std::vector<cv::Mat> frames;
    if (!a.imagesDir.empty()) {
        frames = loadFrames(a.imagesDir);
        printf("Loaded %d image(s) from %s\n", (int)frames.size(), a.imagesDir.c_str());
    } else if (a.useCache) {
        frames = loadFrames(a.cacheDir);
        printf("Loaded %d cached frame(s) from %s\n", (int)frames.size(), a.cacheDir.c_str());
    } else {
        frames = captureLive(cfg, bd, a.minCorners);
        if (!frames.empty()) saveFrames(a.cacheDir, frames);
    }
    if (frames.empty()) { fprintf(stderr, "No frames.\n"); return 1; }

    CalibResult r = calibrate(frames, bd, a);
    if (r.K.empty()) return 1;
    report(r, frames[0].size(), a);
    return save(r, frames[0].size(), a, spec, cfg) ? 0 : 1;
}
