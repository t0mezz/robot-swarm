// tools/vision/calibration/homography.cpp
// Fits the pixel -> world (mm) homography the tracker and every vision tool use,
// from many reference points instead of four hand-clicked corners, and tells
// you how good the fit actually is.
//
//   ./homography --arena 800 600                 # click reference points
//   ./homography --board --arena 800 600         # lay a ChArUco board down
//
// Both modes work in *undistorted* pixel space when tools/build/intrinsics has
// produced a camera_intrinsics.yml, and stamp that into aruco_homography.yml;
// ArucoTracker refuses a homography fitted in the other pixel space.
//
// ── Arena mode (default) ─────────────────────────────────────────────────────
// Mark the arena on the floor (tape at the corners, edge midpoints, centre) and
// click them in this order:
//     1 TL   2 TR   3 BR   4 BL   5 top-mid   6 right-mid   7 bottom-mid
//     8 left-mid   9 centre
// 4 are enough to fit, 9 let the tool check itself: it reports each point's
// residual and a leave-one-out error (fit without the point, then predict it),
// which is the honest accuracy. A magnifier follows the cursor; arrow keys
// nudge the last point by 0.25 px, 'r' snaps clicks to the nearest corner.
//
// ── Board mode (--board) ─────────────────────────────────────────────────────
// Lay the printed ChArUco board flat in the arena, upright (top edge towards
// the arena's top), SPACE to capture, then type where the board's top-left
// outer corner sits in the arena frame (mm, origin = arena top-left corner,
// x right, y down). Repeat in several places — every inner corner (54 for the
// default board) becomes a sub-pixel reference point and each placement is
// held out in turn for the accuracy figure. Remove the robots first: their
// markers share ids with the board's.
//
// ── Calibrate at the height of the markers ──────────────────────────────────
// A homography is exact for one plane. The robots' markers sit a few cm above
// the floor, and a point at height h seen at distance r from the camera axis
// lands h·r/H_cam off if you calibrate on the floor: centimetres at the arena
// edge. Put the board (or the tape marks) at the height of the robot markers,
// and pass that height as --plane-mm so it is recorded.
//
// Options:
//   --arena <W> <H>         arena size in mm (prompted for in arena mode)
//   --board                 board mode;  --board-file <json> (default as intrinsics)
//   --square-mm <f>         measured printed square size (board mode)
//   --plane-mm <f>          height of the calibrated plane above the floor (record only)
//   --output <path>         default: tools/vision/aruco_homography.yml
//   --config <path>, --serial <sn>, --ip <ip>

#include "aruco_tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

// ─── Arguments ────────────────────────────────────────────────────────────────

struct Args {
    std::string serial, ip, config = ArucoConfig::defaultConfigPath();
    std::string output    = arucoVisionDataPath("aruco_homography.yml");
    std::string boardFile = arucoVisionDataPath("calibration/charuco_board.json");
    bool   board = false;
    double arenaW = 0, arenaH = 0, squareMm = -1, planeMm = 0;
};

static bool parseArgs(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto need = [&](int n) { if (i + n >= argc) { fprintf(stderr, "missing value for %s\n", k.c_str()); exit(1); } };
        if      (k == "--arena")      { need(2); a.arenaW = atof(argv[++i]); a.arenaH = atof(argv[++i]); }
        else if (k == "--board")      a.board = true;
        else if (k == "--board-file") { need(1); a.boardFile = argv[++i]; }
        else if (k == "--square-mm")  { need(1); a.squareMm = atof(argv[++i]); }
        else if (k == "--plane-mm")   { need(1); a.planeMm = atof(argv[++i]); }
        else if (k == "--output")     { need(1); a.output = argv[++i]; }
        else if (k == "--config")     { need(1); a.config = argv[++i]; }
        else if (k == "--serial")     { need(1); a.serial = argv[++i]; }
        else if (k == "--ip")         { need(1); a.ip = argv[++i]; }
        else return false;
    }
    return true;
}

// ─── Fit + quality report ─────────────────────────────────────────────────────

struct Sample {
    cv::Point2f pix;     // undistorted pixel
    cv::Point2f world;   // mm
    int         group;   // what is held out together for cross-validation
    std::string name;
};

struct Fit {
    cv::Mat H;
    double  rms = 0, maxErr = 0, cvRms = -1;
    std::vector<double> err;       // per sample, mm
    std::vector<double> cvErr;     // per sample, held out; <0 = not available
};

static cv::Mat fitH(const std::vector<Sample>& s, int skipGroup = -1) {
    std::vector<cv::Point2f> p, w;
    for (auto& x : s) if (x.group != skipGroup) { p.push_back(x.pix); w.push_back(x.world); }
    if (p.size() < 4) return {};
    return cv::findHomography(p, w, 0);
}

static cv::Point2f applyH(const cv::Mat& H, cv::Point2f p) {
    std::vector<cv::Point2f> in{p}, out;
    cv::perspectiveTransform(in, out, H);
    return out[0];
}

static Fit fitAll(const std::vector<Sample>& s) {
    Fit f;
    f.H = fitH(s);
    if (f.H.empty()) return f;
    double ss = 0;
    for (auto& x : s) {
        double e = cv::norm(applyH(f.H, x.pix) - x.world);
        f.err.push_back(e); ss += e * e; f.maxErr = std::max(f.maxErr, e);
    }
    f.rms = std::sqrt(ss / s.size());

    // Hold each group out, fit the rest, predict the group. Needs the rest to
    // still pin the homography down, and more than one group to exist.
    f.cvErr.assign(s.size(), -1.0);
    std::vector<int> groups;
    for (auto& x : s) if (std::find(groups.begin(), groups.end(), x.group) == groups.end()) groups.push_back(x.group);
    if (groups.size() >= 2) {
        double cs = 0; int cn = 0;
        for (int g : groups) {
            int left = (int)std::count_if(s.begin(), s.end(), [&](const Sample& x) { return x.group != g; });
            if (left < 4 || (int)s.size() - left == 0) continue;
            cv::Mat Hg = fitH(s, g);
            if (Hg.empty()) continue;
            for (size_t i = 0; i < s.size(); ++i)
                if (s[i].group == g) {
                    double e = cv::norm(applyH(Hg, s[i].pix) - s[i].world);
                    f.cvErr[i] = e; cs += e * e; cn++;
                }
        }
        if (cn) f.cvRms = std::sqrt(cs / cn);
    }
    return f;
}

static void reportFit(const std::vector<Sample>& s, const Fit& f, cv::Size sz, bool board) {
    printf("\n── Points ──────────────────────────────────\n");
    if (!board) {
        printf("  %-10s %9s %9s %9s  %s\n", "point", "world x", "world y", "fit (mm)", "held out (mm)");
        for (size_t i = 0; i < s.size(); ++i) {
            printf("  %-10s %9.1f %9.1f %9.2f  ", s[i].name.c_str(), s[i].world.x, s[i].world.y, f.err[i]);
            if (f.cvErr[i] >= 0) printf("%9.2f", f.cvErr[i]); else printf("%9s", "-");
            printf("%s\n", f.err[i] > 2.0 * std::max(f.rms, 0.5) ? "   <-- re-click?" : "");
        }
    } else {
        std::vector<int> groups;
        for (auto& x : s) if (std::find(groups.begin(), groups.end(), x.group) == groups.end()) groups.push_back(x.group);
        printf("  %-10s %7s %10s %14s\n", "placement", "points", "fit rms mm", "held out rms mm");
        for (int g : groups) {
            double a = 0, c = 0; int n = 0, cn = 0;
            for (size_t i = 0; i < s.size(); ++i)
                if (s[i].group == g) { a += f.err[i] * f.err[i]; n++; if (f.cvErr[i] >= 0) { c += f.cvErr[i] * f.cvErr[i]; cn++; } }
            printf("  %-10d %7d %10.2f ", g + 1, n, std::sqrt(a / n));
            if (cn) printf("%14.2f\n", std::sqrt(c / cn)); else printf("%14s\n", "-");
        }
    }

    // Local scale, to spot a plane that is not the one you thought.
    auto scaleAt = [&](cv::Point2f p) {
        return cv::norm(applyH(f.H, p + cv::Point2f(1, 0)) - applyH(f.H, p));
    };
    printf("\n── Result (%zu points) ──────────────────────\n", s.size());
    printf("  fit RMS        : %.2f mm   (max %.2f mm)\n", f.rms, f.maxErr);
    if (f.cvRms >= 0) printf("  held-out RMS   : %.2f mm   <- the honest accuracy\n", f.cvRms);
    else              printf("  held-out RMS   : n/a (need >= 5 points)\n");
    printf("  scale          : %.3f mm/px centre, %.3f / %.3f mm/px at opposite corners\n",
           scaleAt({sz.width / 2.f, sz.height / 2.f}), scaleAt({0, 0}),
           scaleAt({(float)sz.width, (float)sz.height}));
    double det = cv::determinant(f.H(cv::Rect(0, 0, 2, 2)));
    if (det < 0) printf("  WARNING: the fit mirrors the image (check the click order / board orientation)\n");

    printf("\n── Verdict ─────────────────────────────────\n");
    double acc = f.cvRms >= 0 ? f.cvRms : f.rms;
    if (s.size() < 6 && !board)
        printf("  Only %zu points: a 4-point fit has no redundancy, so the error above is zero by\n"
               "  construction. Add the 5 midpoint/centre marks for a real check.\n", s.size());
    else if (acc < 1.0) printf("  %.2f mm: good.\n", acc);
    else if (acc < 3.0) printf("  %.2f mm: usable. More/better-placed points would help.\n", acc);
    else                printf("  %.2f mm: poor. Wrong arena size, bent board, or points that are not coplanar.\n", acc);
}

// ─── Overlay: the world grid, drawn back onto the camera image ───────────────

static void drawGrid(cv::Mat& img, const cv::Mat& H, double W, double Hh, double scale) {
    cv::Mat Hinv = H.inv();
    auto toImg = [&](double x, double y) {
        std::vector<cv::Point2f> in{{(float)x, (float)y}}, out;
        cv::perspectiveTransform(in, out, Hinv);
        out = arucoDistortPixels(out);                  // back to the pixels we display
        return out[0] * (float)scale;
    };
    auto line = [&](double x0, double y0, double x1, double y1, cv::Scalar c, int th) {
        cv::Point2f prev = toImg(x0, y0);
        const int n = 40;
        for (int i = 1; i <= n; ++i) {
            cv::Point2f q = toImg(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n);
            cv::line(img, prev, q, c, th, cv::LINE_AA);
            prev = q;
        }
    };
    for (double x = 0; x <= W + 1e-6; x += 100) line(x, 0, x, Hh, {0, 255, 0}, 1);
    for (double y = 0; y <= Hh + 1e-6; y += 100) line(0, y, W, y, {0, 255, 0}, 1);
    line(0, 0, W, 0, {0, 255, 255}, 2); line(W, 0, W, Hh, {0, 255, 255}, 2);
    line(W, Hh, 0, Hh, {0, 255, 255}, 2); line(0, Hh, 0, 0, {0, 255, 255}, 2);
}

// ─── Save ─────────────────────────────────────────────────────────────────────

static bool saveFit(const std::string& path, const Fit& f, cv::Size sz, size_t n, const Args& a,
                    const char* method) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { fprintf(stderr, "cannot write %s\n", path.c_str()); return false; }
    char when[64]; std::time_t t = std::time(nullptr);
    std::strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    // Keys 1-3 are what ArucoTracker::loadHomography reads; the rest is provenance.
    fs << "H" << f.H;
    fs << "img_width" << sz.width << "img_height" << sz.height;
    fs << "undistorted" << (arucoIntrinsics().valid() ? 1 : 0);
    fs << "date" << when << "method" << method << "num_points" << (int)n;
    fs << "rms_mm" << f.rms << "heldout_rms_mm" << f.cvRms;
    fs << "arena_w_mm" << a.arenaW << "arena_h_mm" << a.arenaH << "plane_height_mm" << a.planeMm;
    printf("\nSaved %s  (%s lens correction)\n", path.c_str(),
           arucoIntrinsics().valid() ? "with" : "WITHOUT");
    printf("The saved ring/circle fixtures (car_following_ring.yml, circle_demo.yml) are in the old "
           "world frame:\nre-set them if the arena origin or size changed.\n");
    return true;
}

// ─── UI state ────────────────────────────────────────────────────────────────

struct UI {
    double scale = 1.0;
    cv::Point2f mouse{0, 0};          // full-res pixels
    bool clicked = false, undo = false;
    cv::Point2f click{0, 0};
};
static void onMouse(int ev, int x, int y, int, void* ud) {
    auto* u = (UI*)ud;
    u->mouse = {(float)(x / u->scale), (float)(y / u->scale)};
    if (ev == cv::EVENT_LBUTTONDOWN) { u->clicked = true; u->click = u->mouse; }
    if (ev == cv::EVENT_RBUTTONDOWN) u->undo = true;
}

static void magnifier(cv::Mat& disp, const cv::Mat& full, cv::Point2f c) {
    const int R = 16, Z = 8;
    cv::Mat crop(2 * R + 1, 2 * R + 1, full.type(), cv::Scalar::all(0));
    cv::Point o((int)std::lround(c.x) - R, (int)std::lround(c.y) - R);
    cv::Rect src = cv::Rect(o.x, o.y, 2 * R + 1, 2 * R + 1) & cv::Rect(0, 0, full.cols, full.rows);
    if (src.empty()) return;
    full(src).copyTo(crop(cv::Rect(src.x - o.x, src.y - o.y, src.width, src.height)));
    cv::Mat big; cv::resize(crop, big, {}, Z, Z, cv::INTER_NEAREST);
    int mid = (2 * R + 1) * Z / 2;
    cv::line(big, {mid, 0}, {mid, big.rows}, {0, 0, 255}, 1);
    cv::line(big, {0, mid}, {big.cols, mid}, {0, 0, 255}, 1);
    cv::rectangle(big, {0, 0}, {big.cols - 1, big.rows - 1}, {255, 255, 255}, 2);
    cv::Rect dst(disp.cols - big.cols - 10, 10, big.cols, big.rows);
    if (dst.x >= 0 && dst.y + dst.height <= disp.rows) big.copyTo(disp(dst));
}

static void label(cv::Mat& img, const std::string& t, cv::Point o, double sc = 0.8) {
    cv::putText(img, t, o, cv::FONT_HERSHEY_SIMPLEX, sc, {0, 0, 0}, 4, cv::LINE_AA);
    cv::putText(img, t, o, cv::FONT_HERSHEY_SIMPLEX, sc, {0, 255, 0}, 2, cv::LINE_AA);
}

// Phase result
enum class Phase { Collected, Quit };

// ─── Collection: arena clicks ─────────────────────────────────────────────────

static Phase collectArena(BaslerPylonSource& cam, const ArucoConfig& cfg, const Args& a,
                          std::vector<Sample>& out) {
    const double W = a.arenaW, H = a.arenaH;
    struct Spec { const char* name; double x, y; };
    const Spec spec[9] = {{"TL", 0, 0}, {"TR", W, 0}, {"BR", W, H}, {"BL", 0, H},
                          {"top-mid", W / 2, 0}, {"right-mid", W, H / 2},
                          {"bottom-mid", W / 2, H}, {"left-mid", 0, H / 2}, {"centre", W / 2, H / 2}};

    cv::Mat frame;
    for (int i = 0; i < 5; ++i) cam.read(frame);
    UI ui; ui.scale = std::min(1.0, 1100.0 / std::max(frame.cols, frame.rows));
    const char* WIN = "Homography - click points";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, (int)(frame.cols * ui.scale), (int)(frame.rows * ui.scale));
    cv::setMouseCallback(WIN, onMouse, &ui);

    printf("Click in order: TL TR BR BL, then top-mid right-mid bottom-mid left-mid centre.\n"
           "Right-click / u: undo   arrows: nudge 0.25 px   r: snap to corner   ENTER: fit (>=4)   q: quit\n");

    std::vector<cv::Point2f> raw;       // as clicked, distorted pixels (kept so undistortion can't drift)
    bool snap = false;
    cv::Mat shown;

    while (true) {
        if (!cam.read(frame) || frame.empty()) { cv::waitKey(10); continue; }
        if (cfg.mirrorInput) cv::flip(frame, frame, 1);
        cv::resize(frame, shown, {}, ui.scale, ui.scale, cv::INTER_AREA);

        if (ui.clicked) {
            ui.clicked = false;
            if (raw.size() < 9) {
                cv::Point2f p = ui.click;
                if (snap) {
                    cv::Mat g; cv::cvtColor(frame, g, cv::COLOR_BGR2GRAY);
                    std::vector<cv::Point2f> v{p};
                    cv::cornerSubPix(g, v, {9, 9}, {-1, -1},
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
                    if (cv::norm(v[0] - p) < 8.0) p = v[0];
                }
                raw.push_back(p);
                printf("  %-10s pixel (%.2f, %.2f)\n", spec[raw.size() - 1].name, p.x, p.y);
            }
        }
        if (ui.undo && !raw.empty()) { raw.pop_back(); }
        ui.undo = false;

        for (size_t i = 0; i < raw.size(); ++i) {
            cv::Point2f p = raw[i] * (float)ui.scale;
            cv::circle(shown, p, 6, {0, 0, 255}, 2, cv::LINE_AA);
            label(shown, std::to_string(i + 1) + " " + spec[i].name, {(int)p.x + 10, (int)p.y - 8}, 0.6);
        }
        std::string st = raw.size() < 9
            ? "next: " + std::to_string(raw.size() + 1) + " " + spec[raw.size()].name
            : "all 9 points - ENTER to fit";
        st += snap ? "   [snap on]" : "   [snap off]";
        label(shown, st, {12, 32});
        magnifier(shown, frame, ui.mouse);
        cv::imshow(WIN, shown);

        int k = cv::waitKeyEx(1);
        if (k == 'q' || k == 27) { cv::destroyWindow(WIN); return Phase::Quit; }
        if (k == 'u' && !raw.empty()) raw.pop_back();
        if (k == 'r') snap = !snap;
        // Arrow keys (X11 keycodes via waitKeyEx): left 81 up 82 right 83 down 84 (also 65361..).
        if (!raw.empty()) {
            const float d = 0.25f;
            if (k == 65361 || k == 2424832) raw.back().x -= d;
            if (k == 65363 || k == 2555904) raw.back().x += d;
            if (k == 65362 || k == 2490368) raw.back().y -= d;
            if (k == 65364 || k == 2621440) raw.back().y += d;
        }
        if ((k == 13 || k == 10) && raw.size() >= 4) break;
    }
    cv::destroyWindow(WIN);

    std::vector<cv::Point2f> und = arucoUndistortPixels(raw);
    out.clear();
    for (size_t i = 0; i < raw.size(); ++i)
        out.push_back({und[i], {(float)spec[i].x, (float)spec[i].y}, (int)i, spec[i].name});
    return Phase::Collected;
}

// ─── Collection: ChArUco placements ──────────────────────────────────────────

static Phase collectBoard(BaslerPylonSource& cam, const ArucoConfig& cfg, const Args& a,
                          std::vector<Sample>& out) {
    cv::FileStorage bf(a.boardFile, cv::FileStorage::READ);
    int sx = 10, sy = 7, dictId = cv::aruco::DICT_4X4_50;
    double sqMm = 36.0, mkMm = 26.87;
    if (bf.isOpened()) { bf["squares_x"] >> sx; bf["squares_y"] >> sy; bf["square_mm"] >> sqMm;
                         bf["marker_mm"] >> mkMm; bf["dict_id"] >> dictId; }
    else fprintf(stderr, "no board file at %s - using defaults\n", a.boardFile.c_str());
    if (a.squareMm > 0) { mkMm *= a.squareMm / sqMm; sqMm = a.squareMm; }
    cv::aruco::CharucoBoard board({sx, sy}, (float)sqMm, (float)mkMm,
                                  cv::aruco::getPredefinedDictionary(dictId));
    cv::aruco::DetectorParameters dp;
    dp.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    cv::aruco::CharucoParameters cp; cp.tryRefineMarkers = false;
    cv::aruco::CharucoDetector det(board, cp, dp);

    cv::Mat frame;
    for (int i = 0; i < 5; ++i) cam.read(frame);
    UI ui; ui.scale = std::min(1.0, 1100.0 / std::max(frame.cols, frame.rows));
    const char* WIN = "Homography - ChArUco placements";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(WIN, (int)(frame.cols * ui.scale), (int)(frame.rows * ui.scale));

    printf("Lay the board flat, upright, at the height of the robot markers. SPACE: capture a\n"
           "placement (then type its position)   u: drop last   ENTER: fit   q: quit\n");
    int placements = 0;                    // out may already hold placements ("back" from the fit)
    for (auto& s : out) placements = std::max(placements, s.group + 1);
    cv::Mat shown;
    while (true) {
        if (!cam.read(frame) || frame.empty()) { cv::waitKey(10); continue; }
        if (cfg.mirrorInput) cv::flip(frame, frame, 1);
        cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> c; std::vector<int> ids;
        det.detectBoard(gray, c, ids);

        cv::resize(frame, shown, {}, ui.scale, ui.scale, cv::INTER_AREA);
        for (auto& s : out) cv::circle(shown, arucoDistortPixel(s.pix) * (float)ui.scale, 3, {255, 160, 0}, -1);
        for (auto& p : c) cv::circle(shown, p * (float)ui.scale, 4, {0, 255, 255}, -1);
        char t[160];
        snprintf(t, sizeof(t), "placements %d | board corners now %zu/%d | SPACE capture  ENTER fit",
                 placements, ids.size(), (sx - 1) * (sy - 1));
        label(shown, t, {12, 32}, 0.7);
        cv::imshow(WIN, shown);

        int k = cv::waitKey(1);
        if (k == 'q' || k == 27) { cv::destroyWindow(WIN); return Phase::Quit; }
        if (k == 'u' && placements > 0) {
            out.erase(std::remove_if(out.begin(), out.end(),
                      [&](const Sample& s) { return s.group == placements - 1; }), out.end());
            placements--; printf("  dropped placement %d\n", placements + 1);
        }
        if ((k == 13 || k == 10) && placements >= 1 && out.size() >= 4) break;
        if (k == ' ') {
            if (ids.size() < 8) { printf("  only %zu corners visible - not taken\n", ids.size()); continue; }
            double ox = 0, oy = 0;
            printf("  placement %d: %zu corners. Board top-left outer corner at (x y) mm [0 0]: ",
                   placements + 1, ids.size());
            fflush(stdout);
            std::string line; std::getline(std::cin, line);
            if (!line.empty() && sscanf(line.c_str(), "%lf %lf", &ox, &oy) != 2) { printf("  not understood - not taken\n"); continue; }

            std::vector<cv::Point3f> obj; std::vector<cv::Point2f> img;
            board.matchImagePoints(c, ids, obj, img);
            std::vector<cv::Point2f> und = arucoUndistortPixels(img);
            for (size_t i = 0; i < obj.size(); ++i)
                out.push_back({und[i], {(float)(obj[i].x + ox), (float)(obj[i].y + oy)}, placements,
                               "board"});
            placements++;
            printf("  placement %d taken (%zu points)\n", placements, obj.size());
        }
    }
    cv::destroyWindow(WIN);
    return Phase::Collected;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    Args a;
    if (!parseArgs(argc, argv, a)) {
        fprintf(stderr, "usage: homography [--arena W H] [--board] [--square-mm f] [--plane-mm f]\n"
                        "       see the comment at the top of homography.cpp\n");
        return 1;
    }
    if (a.arenaW <= 0 || a.arenaH <= 0) {
        if (!a.board) {
            printf("Arena size in mm, width height (e.g. 800 600): ");
            if (scanf("%lf %lf", &a.arenaW, &a.arenaH) != 2 || a.arenaW <= 0 || a.arenaH <= 0) {
                fprintf(stderr, "invalid size\n"); return 1;
            }
            int ch; while ((ch = getchar()) != '\n' && ch != EOF) {}
        } else { a.arenaW = 800; a.arenaH = 600; }   // grid extent only
    }

    ArucoConfig cfg = ArucoConfig::fromFile(a.config);
    if (!a.serial.empty()) cfg.baslerSerial = a.serial;
    if (!a.ip.empty())     cfg.baslerIp     = a.ip;
    cfg.debugOverlay = false;

    BaslerPylonSource cam;
    if (!cam.open(cfg)) { fprintf(stderr, "cannot open Basler camera (is another tool holding it?)\n"); return 1; }
    cv::Mat probe; for (int i = 0; i < 5; ++i) cam.read(probe);
    const cv::Size sz = probe.size();

    // Same lens model the tracker will run with, in the same frame orientation.
    auto& I = arucoIntrinsics();
    I = CameraIntrinsics{};
    if (cfg.useIntrinsics && I.load(arucoVisionDataPath("camera_intrinsics.yml"), sz, {cfg.offsetX, cfg.offsetY})) {
        I.mirror = cfg.mirrorInput;
        printf("Lens model loaded (rms %.3f px): fitting in undistorted pixel space.\n", I.rms);
    } else {
        printf("No lens model - fitting in raw pixel space. Run tools/build/intrinsics first for best accuracy.\n");
    }
    if (a.planeMm <= 0)
        printf("Note: calibrate on the plane of the robot markers (put the board/tape at that height),\n"
               "      then pass --plane-mm so it is recorded. A floor calibration is off by h*r/H_cam.\n");

    std::vector<Sample> samples;
    while (true) {
        Phase ph = a.board ? collectBoard(cam, cfg, a, samples) : collectArena(cam, cfg, a, samples);
        if (ph == Phase::Quit) { printf("Quit - nothing saved.\n"); return 0; }

        Fit f = fitAll(samples);
        if (f.H.empty()) { fprintf(stderr, "fit failed (degenerate points?)\n"); continue; }
        reportFit(samples, f, sz, a.board);

        // Look at it: the world grid projected back onto the live image.
        printf("\nGrid shown on the live image. s: save   b: back, change points   q: quit\n");
        const char* WIN = "Homography - check the grid";
        cv::namedWindow(WIN, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
        double sc = std::min(1.0, 1100.0 / std::max(sz.width, sz.height));
        cv::resizeWindow(WIN, (int)(sz.width * sc), (int)(sz.height * sc));
        bool save = false, back = false, quit = false;
        cv::Mat frame, shown;
        while (!save && !back && !quit) {
            if (!cam.read(frame) || frame.empty()) { cv::waitKey(10); continue; }
            if (cfg.mirrorInput) cv::flip(frame, frame, 1);
            cv::resize(frame, shown, {}, sc, sc, cv::INTER_AREA);
            drawGrid(shown, f.H, a.arenaW, a.arenaH, sc);
            char t[160];
            snprintf(t, sizeof(t), "fit %.2f mm | held-out %s | s save  b back  q quit", f.rms,
                     f.cvRms >= 0 ? (std::to_string(f.cvRms).substr(0, 4) + " mm").c_str() : "n/a");
            label(shown, t, {12, 32}, 0.7);
            cv::imshow(WIN, shown);
            int k = cv::waitKey(1);
            if (k == 's') save = true; else if (k == 'b') back = true; else if (k == 'q' || k == 27) quit = true;
        }
        cv::destroyWindow(WIN);
        if (quit) { printf("Quit - nothing saved.\n"); return 0; }
        if (save) return saveFit(a.output, f, sz, samples.size(), a, a.board ? "charuco" : "arena-points") ? 0 : 1;
        // back: go around again; in arena mode the points restart, in board mode they are kept.
        if (!a.board) samples.clear();
    }
}
