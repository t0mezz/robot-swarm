#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <future>
#include <queue>
#include <functional>
#include <cmath>
#include <chrono>

#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

// ─── ICameraSource ────────────────────────────────────────────────────────────

struct ArucoConfig;

struct ICameraSource {
    virtual ~ICameraSource() = default;
    virtual bool     open(const ArucoConfig& cfg) = 0;
    virtual bool     read(cv::Mat& frame) = 0;
    virtual cv::Size size() const = 0;
    virtual float    temperature() { return -1.f; } // °C, -1 = not available
};

// ─── RobotPose ────────────────────────────────────────────────────────────────

struct RobotPose {
    int   id;
    float x, y;   // world coords (mm) if homography set, else pixels
    float yaw;    // degrees CCW from world +X
    float px, py; // raw pixel centroid
};

// ─── ArucoConfig ─────────────────────────────────────────────────────────────
// All fields have working defaults. Override any subset via
// aruco_tracker_config.json — missing keys fall back to the defaults below.

struct ArucoConfig {
    // Cwd-relative fallback location of aruco_tracker_config.json (assumes the
    // demo is run from tools/build/). Prefer defaultConfigPath(), which
    // resolves the same file relative to the executable instead, so demos work
    // from any cwd.
    static constexpr const char* kDefaultConfigPath = "../vision/aruco_tracker_config.json";

    // <exe_dir>/../vision/aruco_tracker_config.json if it exists (demo
    // binaries land in tools/build/, the config lives in tools/vision/),
    // otherwise falls back to kDefaultConfigPath.
    static std::string defaultConfigPath();

    // Camera — Basler ace2 GigE
    int         width = 1920, height = 1080, fps = 30;
    std::string baslerSerial = "";
    std::string baslerIp     = "";
    // Sensor ROI offset (GenICam OffsetX/OffsetY) — shifts the readout window
    // on the sensor itself, applied directly to the camera via pylon at
    // open() time (BaslerPylonSource::open()), not a post-detection pixel
    // nudge. Must satisfy offsetX + width <= sensor width (and similarly for
    // Y) or pylon rejects it — see the camera's WidthMax/HeightMax nodes.
    int         offsetX = 0, offsetY = 0;

    // ArUco detector
    int   dictId        = cv::aruco::DICT_4X4_50;
    int   winMin        = 3, winMax = 13, winStep = 4;  // tuned: fewer threshold passes
    float threshC       = 7.0f;
    float minPerimRate  = 0.02f;
    float polyApprox    = 0.05f;
    int   pixPerCell    = 10;
    float cellMargin    = 0.20f;
    float errorCorr     = 0.7f;
    float minOtsuStdDev = 5.0f;
    int   cornerWin     = 5, cornerMaxIter = 10;        // tuned: refinement converges fast
    bool  halfResSweep  = true;

    // Kalman filter (constant-velocity, per marker)
    float kfProcPos = 1e-4f;
    float kfProcVel = 1e-2f;
    float kfMeas    = 4.0f;
    float kfInitCov = 100.0f;
    // Resolution the kf_* process-noise params above were tuned at. The filter
    // runs in pixel space, so its process-noise variances scale with the square
    // of the pixel-per-mm factor; initKalman() rescales them from this reference
    // to the live frame size so the filter tracks the same *physical* dynamics
    // at any camera resolution. Without it, raising the resolution leaves Q too
    // small in px, so the estimate lags the marker during motion — which biases
    // the corner-derived yaw and surfaces as heading oscillation downstream.
    int   kfRefWidth  = 1224;
    int   kfRefHeight = 1024;

    // ROI state machine
    float roiPad      = 1.25f;
    float roiGrow     = 1.20f;
    int   roiFailMax  = 5;
    float roiAreaMax  = 0.40f;
    int   globalReset = 30;
    // Expected number of distinct markers (robots) in the scene. Once that
    // many distinct IDs have ever been seen (markerStates_.size() reaches
    // this count), the forced full-frame sweep below is dropped and the
    // detector goes back to pure per-marker ROI tracking. Below that count,
    // a global sweep runs every frame (regardless of per-marker ROI state)
    // so newly-appearing markers keep getting picked up even while every
    // already-known marker is locked into a tight local ROI — otherwise the
    // global sweep only fires when markerStates_ is empty or some existing
    // marker has fully lost tracking, so a marker that never had an entry
    // stays invisible indefinitely. 0 = unknown/uncapped: always keep
    // sweeping every frame.
    int   robotCount  = 0;
    // Throttle for the forced/rediscovery global sweep above: when a sweep is
    // wanted (robotCount not yet reached, or some marker fell back to
    // RoiState::GLOBAL), only actually run it on every Nth detection-thread
    // frame instead of every frame. Trades slower marker (re)acquisition —
    // up to N-1 extra detection-thread frames of pure Kalman dead-reckoning
    // before a lost/new marker is looked for again — for det_fps, since a
    // forced-every-frame global sweep costs roughly 2x a pure-ROI frame (see
    // TODO.md/circle_demo --log-perf). 1 = sweep every wanted frame (default,
    // matches pre-throttling behavior).
    int   globalSweepInterval = 1;

    // CLAHE — applied lazily (only to the image actually passed to detectMarkers)
    float claheClip = 2.0f;
    int   claheTile = 8;

    bool debugOverlay = false;
    bool mirrorInput  = false;

    static ArucoConfig fromFile(const std::string& path = defaultConfigPath());
};

inline std::string ArucoConfig::defaultConfigPath() {
    char buf[4096];
#ifdef __APPLE__
    uint32_t size = sizeof(buf);
    if (_NSGetExecutablePath(buf, &size) != 0) return kDefaultConfigPath;
#else
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return kDefaultConfigPath;
    buf[n] = '\0';
#endif
    std::string path(buf);
    size_t slash = path.rfind('/');
    if (slash == std::string::npos) return kDefaultConfigPath;
    path = path.substr(0, slash + 1) + "../vision/aruco_tracker_config.json";
    if (access(path.c_str(), R_OK) != 0) return kDefaultConfigPath;
    return path;
}

inline ArucoConfig ArucoConfig::fromFile(const std::string& path) {
    ArucoConfig c;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        fprintf(stderr, "[aruco] config '%s' not found — using defaults\n", path.c_str());
        return c;
    }
    auto ri = [&](const char* k, int&         v) { if (!fs[k].empty()) fs[k] >> v; };
    auto rf = [&](const char* k, float&       v) { if (!fs[k].empty()) fs[k] >> v; };
    auto rb = [&](const char* k, bool&        v) { int t = (int)v; if (!fs[k].empty()) { fs[k] >> t; v = (bool)t; } };
    auto rs = [&](const char* k, std::string& v) { if (!fs[k].empty()) fs[k] >> v; };
    rs("basler_serial",   c.baslerSerial);
    rs("basler_ip",       c.baslerIp);
    ri("cam_width",       c.width);
    ri("cam_height",      c.height);
    ri("cam_fps",         c.fps);
    ri("aruco_dict",      c.dictId);
    ri("win_min",         c.winMin);
    ri("win_max",         c.winMax);
    ri("win_step",        c.winStep);
    rf("thresh_c",        c.threshC);
    rf("min_perim_rate",  c.minPerimRate);
    rf("poly_approx",     c.polyApprox);
    ri("pix_per_cell",    c.pixPerCell);
    rf("cell_margin",     c.cellMargin);
    rf("error_corr",      c.errorCorr);
    rf("min_otsu_stddev", c.minOtsuStdDev);
    ri("corner_win",      c.cornerWin);
    ri("corner_max_iter", c.cornerMaxIter);
    rb("half_res_sweep",  c.halfResSweep);
    rf("kf_proc_pos",     c.kfProcPos);
    rf("kf_proc_vel",     c.kfProcVel);
    rf("kf_meas",         c.kfMeas);
    rf("kf_init_cov",     c.kfInitCov);
    ri("kf_ref_width",    c.kfRefWidth);
    ri("kf_ref_height",   c.kfRefHeight);
    rf("roi_pad",         c.roiPad);
    rf("roi_grow",        c.roiGrow);
    ri("roi_fail_max",    c.roiFailMax);
    rf("roi_area_max",    c.roiAreaMax);
    ri("global_reset",    c.globalReset);
    ri("robot_count",     c.robotCount);
    ri("global_sweep_interval", c.globalSweepInterval);
    rf("clahe_clip",      c.claheClip);
    ri("clahe_tile",      c.claheTile);
    rb("debug_overlay",   c.debugOverlay);
    rb("mirror_input",    c.mirrorInput);
    ri("offset_x",        c.offsetX);
    ri("offset_y",        c.offsetY);
    return c;
}

// Builds a detector with the same dictionary/parameters ArucoTracker uses
// internally. Exposed so one-shot/offline tools (e.g. frame_inspector) can
// run an identical detection pass without the live tracker's threading,
// Kalman filtering, or ROI state.
inline cv::aruco::ArucoDetector buildArucoDetector(const ArucoConfig& cfg) {
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

// ─── Preprocessor pipeline ────────────────────────────────────────────────────

struct IPreprocessor {
    virtual ~IPreprocessor() = default;
    virtual void process(cv::Mat& gray) = 0;
};

// CLAHE is now applied lazily inside ArucoTracker — this struct remains available
// for external use (e.g. calibration tools) or custom pipelines.
struct CLAHEPreprocessor : IPreprocessor {
    explicit CLAHEPreprocessor(float clip = 2.0f, int tile = 8)
        : clahe_(cv::createCLAHE(clip, {tile, tile})) {}
    void process(cv::Mat& gray) override { clahe_->apply(gray, gray); }
private:
    cv::Ptr<cv::CLAHE> clahe_;
};

struct FisheyeUndistortPreprocessor : IPreprocessor {
    bool load(const std::string& calibYaml, cv::Size frameSize) {
        cv::FileStorage fs(calibYaml, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        cv::Mat K, D; cv::Size imgSz;
        fs["K"] >> K;  fs["D"] >> D;  fs["image_size"] >> imgSz;
        if (K.empty() || D.empty()) return false;
        if (imgSz.empty()) imgSz = frameSize;
        cv::fisheye::initUndistortRectifyMap(
            K, D, cv::Mat::eye(3, 3, CV_64F), K, imgSz, CV_16SC2, map1_, map2_);
        ready_ = true;
        fprintf(stderr, "[aruco] fisheye calibration loaded from '%s'\n", calibYaml.c_str());
        return true;
    }
    void process(cv::Mat& gray) override {
        if (!ready_) return;
        cv::Mat out;
        cv::remap(gray, out, map1_, map2_, cv::INTER_LINEAR);
        gray = std::move(out);
    }
    bool ready() const { return ready_; }
private:
    cv::Mat map1_, map2_;
    bool    ready_ = false;
};

#include "basler_pylon_source.h"

// ─── ArucoTracker ─────────────────────────────────────────────────────────────
//
// Threading model:
//   captureThread_   — grabs frames from the camera as fast as it can
//   detectionThread_ — consumes frames, runs ArUco + Kalman, publishes results
//   main thread      — calls update() to swap in the latest result, then renders
//
// Detection and display are fully decoupled: display FPS and detection FPS are
// independent. update() is non-blocking and returns false when no fresh result
// is available yet.

class ArucoTracker {
public:
    explicit ArucoTracker(ArucoConfig cfg = {}) : cfg_(cfg) {
        detector_ = buildArucoDetector(cfg_);

        if (cfg_.claheClip > 0)
            clahe_ = cv::createCLAHE(cfg_.claheClip, {cfg_.claheTile, cfg_.claheTile});
    }

    ~ArucoTracker() { stopThreads(); }

    bool open() {
        source_ = std::make_unique<BaslerPylonSource>();
        if (!source_->open(cfg_)) { source_.reset(); return false; }
        auto sz = source_->size();
        fw_ = (float)sz.width;
        fh_ = (float)sz.height;
        captureRunning_   = true;
        detectionRunning_ = true;
        captureThread_   = std::thread(&ArucoTracker::captureLoop,   this);
        detectionThread_ = std::thread(&ArucoTracker::detectionLoop, this);
        return true;
    }

    // Non-blocking. Returns true when a fresh detection result has been swapped in.
    bool update() {
        std::unique_lock<std::mutex> lk(resultMutex_);
        if (!latestResult_.fresh) return false;
        robots_    = std::move(latestResult_.robots);
        debug_     = std::move(latestResult_.debug);
        fps_       = latestResult_.fps;
        latencyMs_ = latestResult_.latencyMs;
        latestResult_.fresh = false;
        return true;
    }

    // Signal the detection thread to zero its FPS/latency accumulators.
    void requestStatsReset() { statsReset_.store(true); }

    void setHomography(const std::vector<cv::Point2f>& pix,
                       const std::vector<cv::Point2f>& world) {
        H_ = cv::findHomography(pix, world);
        hasH_ = !H_.empty();
    }
    bool loadHomography(const std::string& path) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        fs["H"] >> H_;
        // A homography is only valid for the resolution it was calibrated at:
        // pixel coords scale with resolution, so a homography from a different
        // frame size silently maps the current pixels to mis-scaled world
        // coords (e.g. 2x off after doubling cam_width/height) — which shows up
        // downstream as radial overshoot / oscillation, not an obvious error.
        // Reject the mismatch so the caller falls back / re-calibrates instead.
        int calW = 0, calH = 0;
        if (!fs["img_width"].empty())  fs["img_width"]  >> calW;
        if (!fs["img_height"].empty()) fs["img_height"] >> calH;
        if (calW > 0 && calH > 0 && (calW != (int)fw_ || calH != (int)fh_)) {
            fprintf(stderr,
                "[aruco] Homography '%s' was calibrated at %dx%d but camera is "
                "%dx%d — ignoring it. Re-run calibration at the current "
                "resolution.\n",
                path.c_str(), calW, calH, (int)fw_, (int)fh_);
            H_.release();
            hasH_ = false;
            return false;
        }
        hasH_ = !H_.empty();
        return hasH_;
    }
    void saveHomography(const std::string& path) const {
        if (!hasH_) return;
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "H" << H_;
        // Stamp the resolution so loadHomography() can reject a stale calibration.
        fs << "img_width" << (int)fw_ << "img_height" << (int)fh_;
    }

    void prependPreprocessor(std::unique_ptr<IPreprocessor> p) {
        preprocessors_.insert(preprocessors_.begin(), std::move(p));
    }
    void addPreprocessor(std::unique_ptr<IPreprocessor> p) {
        preprocessors_.push_back(std::move(p));
    }

    const std::vector<RobotPose>& robots()    const { return robots_; }
    cv::Mat  debugFrame() const { return debug_; }
    cv::Size frameSize()  const { return {(int)fw_, (int)fh_}; }
    bool     isOpen()     const { return source_ != nullptr; }
    // Detection-thread throughput (frames actually processed per second), NOT
    // the camera's acquisition rate — with GrabStrategy_LatestImageOnly the
    // camera can deliver faster while this thread skips frames it can't keep
    // up with. The camera's own sustainable rate is printed at open() time
    // (ResultingFrameRate in basler_pylon_source.h).
    float    detectionFps() const { return fps_; }
    float    latencyMs()  const { return latencyMs_; }
    float    cameraTemperature() { return source_ ? source_->temperature() : -1.f; }

    static void drawText(cv::Mat& img, const std::string& text, cv::Point org,
                         int fontSize, cv::Scalar color) {
        if (img.empty()) return;
        org.x = std::max(0, std::min(org.x, img.cols - 1));
        org.y = std::max(fontSize, std::min(org.y, img.rows - 1));
        cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX,
                    fontSize / 28.0, color, 2, cv::LINE_AA);
    }

private:
    // ── Thread pool ───────────────────────────────────────────────────────────
    // Fixed-size pool that lives for the tracker's lifetime, avoiding per-frame
    // thread construction overhead from std::async.
    struct ThreadPool {
        explicit ThreadPool(int n) {
            for (int i = 0; i < n; ++i)
                workers_.emplace_back([this] {
                    while (true) {
                        std::function<void()> task;
                        {
                            std::unique_lock<std::mutex> lk(mtx_);
                            cv_.wait(lk, [&]{ return stop_ || !queue_.empty(); });
                            if (stop_ && queue_.empty()) return;
                            task = std::move(queue_.front());
                            queue_.pop();
                        }
                        task();
                    }
                });
        }
        ~ThreadPool() {
            { std::unique_lock<std::mutex> lk(mtx_); stop_ = true; }
            cv_.notify_all();
            for (auto& t : workers_) t.join();
        }
        template<class F>
        auto submit(F&& f) -> std::future<decltype(f())> {
            using R = decltype(f());
            auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
            auto fut  = task->get_future();
            { std::unique_lock<std::mutex> lk(mtx_); queue_.push([task]{ (*task)(); }); }
            cv_.notify_one();
            return fut;
        }
    private:
        std::vector<std::thread>          workers_;
        std::queue<std::function<void()>> queue_;
        std::mutex                        mtx_;
        std::condition_variable           cv_;
        bool                              stop_ = false;
    };

    // ── Internal types ────────────────────────────────────────────────────────
    enum class RoiState { GLOBAL, LOCAL, EXPANDING };

    struct MarkerState {
        RoiState        state = RoiState::GLOBAL;
        cv::Rect        roi;
        cv::Point2f     center, predicted;
        cv::Size2f      bboxSize;
        int             failCount = 0, globalFrames = 0;
        cv::KalmanFilter kf;
        bool            kfInit = false;
        std::chrono::steady_clock::time_point roiLastDetected =
            std::chrono::steady_clock::time_point::min();
    };

    struct DetectionResult {
        std::vector<RobotPose> robots;
        cv::Mat                debug;
        float                  fps       = 0.f;
        float                  latencyMs = 0.f;
        bool                   fresh     = false;
    };

    // ── Capture thread ────────────────────────────────────────────────────────
    void captureLoop() {
        while (captureRunning_) {
            cv::Mat f;
            if (source_->read(f) && !f.empty()) {
                std::unique_lock<std::mutex> lk(frameMutex_);
                latestFrame_ = std::move(f);
                frameCv_.notify_one();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    // ── Detection thread ──────────────────────────────────────────────────────
    void detectionLoop() {
        using Clock = std::chrono::steady_clock;

        // Pool sized to available cores minus the two dedicated threads
        int poolSize = std::max(1, (int)std::thread::hardware_concurrency() - 2);
        ThreadPool pool(poolSize);

        int   fpsFrames  = 0;
        float fps        = 0.f;
        float emaLatency = 0.f;
        Clock::time_point lastFpsTime;
        long  sweepFrameCounter = 0;  // for globalSweepInterval throttling

        while (true) {
            // Wait for a new frame from the capture thread
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lk(frameMutex_);
                frameCv_.wait(lk, [&]{ return !latestFrame_.empty() || !detectionRunning_; });
                if (!detectionRunning_ && latestFrame_.empty()) break;
                frame = std::move(latestFrame_);
            }

            if (statsReset_.exchange(false)) {
                emaLatency = 0.f; fps = 0.f; fpsFrames = 0;
                printf("[aruco] stats reset\n");
            }

            auto t0 = Clock::now();

            try {
                if (cfg_.mirrorInput)
                    cv::flip(frame, frame, 1);

                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                // User-supplied preprocessors (not CLAHE — that's lazy below)
                for (auto& p : preprocessors_) p->process(gray);

                cv::Mat sweep = gray;
                if (cfg_.halfResSweep)
                    cv::resize(gray, sweep, {}, 0.5, 0.5, cv::INTER_AREA);

                // ── FPS ───────────────────────────────────────────────────────
                auto now = Clock::now();
                if (fpsFrames++ == 0) lastFpsTime = now;
                float el = std::chrono::duration<float>(now - lastFpsTime).count();
                if (el >= 1.0f) { fps = fpsFrames / el; fpsFrames = 0; lastFpsTime = now; }

                // ── Kalman predict ────────────────────────────────────────────
                for (auto& [id, ms] : markerStates_) {
                    if (!ms.kfInit) continue;
                    cv::Mat pred = ms.kf.predict();
                    ms.center = ms.predicted = {pred.at<float>(0), pred.at<float>(1)};
                    if (ms.state != RoiState::GLOBAL) {
                        float hw = ms.roi.width * 0.5f, hh = ms.roi.height * 0.5f;
                        ms.roi = cv::Rect(
                            (int)(ms.center.x - hw), (int)(ms.center.y - hh),
                            ms.roi.width, ms.roi.height);
                    }
                }

                // ── Global sweep — CLAHE only on the sweep image ──────────────
                // Force a global sweep every frame until we've discovered as many
                // distinct markers as expected (cfg_.robotCount) — this is the
                // only path that can find a marker ID with no ROI state yet, so
                // without it, once every currently-known marker is happily
                // LOCAL-tracked, a new/late-appearing robot is never picked up
                // (see TODO.md). Once we've seen enough markers, drop back to
                // sweeping only when needed (no markers yet, or one has fully
                // lost tracking) to save the full-frame detection cost.
                bool wantsGlobal = markerStates_.empty();
                for (auto& [_, ms] : markerStates_)
                    if (ms.state == RoiState::GLOBAL) { wantsGlobal = true; break; }
                if (cfg_.robotCount <= 0 || (int)markerStates_.size() < cfg_.robotCount)
                    wantsGlobal = true;

                // Throttle: only actually run a wanted sweep on every Nth
                // detection-thread frame (see ArucoConfig::globalSweepInterval).
                bool needGlobal = wantsGlobal &&
                    (cfg_.globalSweepInterval <= 1 ||
                     sweepFrameCounter % cfg_.globalSweepInterval == 0);
                ++sweepFrameCounter;

                struct Best { float perim; std::vector<cv::Point2f> c; };
                std::unordered_map<int, Best> best;

                auto merge = [&](int id, std::vector<cv::Point2f> c, float scale, cv::Point2f off) {
                    for (auto& pt : c) { pt = pt * scale + off; }
                    for (auto& pt : c) if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) return;
                    float perim = 0;
                    for (int k = 0; k < 4; ++k) perim += (float)cv::norm(c[k] - c[(k+1)%4]);
                    if (perim < 1.0f) return;
                    if (!best.count(id) || perim > best[id].perim)
                        best[id] = {perim, std::move(c)};
                };

                if (needGlobal) {
                    cv::Mat sweepImg = sweep;
                    if (clahe_) { sweepImg = sweep.clone(); clahe_->apply(sweepImg, sweepImg); }
                    std::vector<std::vector<cv::Point2f>> cs; std::vector<int> ids;
                    std::vector<std::vector<cv::Point2f>> rej;
                    detector_.detectMarkers(sweepImg, cs, ids, rej);
                    float scale = cfg_.halfResSweep ? 2.0f : 1.0f;
                    for (int j = 0; j < (int)ids.size(); ++j)
                        merge(ids[j], cs[j], scale, {0, 0});
                }

                // ── ROI crops via thread pool — each crop gets its own CLAHE ──
                using RoiHits = std::vector<std::pair<int, std::vector<cv::Point2f>>>;
                std::vector<std::future<RoiHits>> futs;

                const float claheClip = cfg_.claheClip;
                const int   claheTile = cfg_.claheTile;
                const bool  hasClahe  = (clahe_ != nullptr);

                for (auto& [sid, sms] : markerStates_) {
                    if (sms.state == RoiState::GLOBAL) continue;
                    futs.push_back(pool.submit(
                        [this, &gray, roi = sms.roi, id = sid,
                         claheClip, claheTile, hasClahe]() -> RoiHits {
                            RoiHits out;
                            cv::Rect r = roi & cv::Rect(0, 0, gray.cols, gray.rows);
                            if (r.area() < 100) return out;
                            cv::Mat crop = gray(r).clone();
                            if (hasClahe) {
                                auto lc = cv::createCLAHE(claheClip, {claheTile, claheTile});
                                lc->apply(crop, crop);
                            }
                            std::vector<std::vector<cv::Point2f>> cs; std::vector<int> ids;
                            std::vector<std::vector<cv::Point2f>> rej;
                            detector_.detectMarkers(crop, cs, ids, rej);
                            for (int j = 0; j < (int)ids.size(); ++j) {
                                if (ids[j] != id) continue;
                                auto c = cs[j];
                                for (auto& pt : c) pt += cv::Point2f((float)r.x, (float)r.y);
                                out.push_back({id, std::move(c)});
                            }
                            return out;
                        }));
                }
                for (auto& f : futs)
                    try { for (auto& [id, c] : f.get()) merge(id, std::move(c), 1.0f, {}); }
                    catch (...) {}

                // ── ROI state machine ─────────────────────────────────────────
                const float frameArea = fw_ * fh_;
                for (auto& [id, ms] : markerStates_) {
                    if (best.count(id)) {
                        applyDetection(ms, best[id].c);
                    } else {
                        if (ms.kfInit) {
                            ms.kf.statePost    = ms.kf.statePre;
                            ms.kf.errorCovPost = ms.kf.errorCovPre;
                            ms.center = {ms.kf.statePost.at<float>(0), ms.kf.statePost.at<float>(1)};
                        }
                        switch (ms.state) {
                        case RoiState::LOCAL:
                            ms.state = RoiState::EXPANDING;
                            ms.failCount++;
                            break;
                        case RoiState::EXPANDING: {
                            ms.failCount++;
                            float cx = ms.roi.x + ms.roi.width  * 0.5f;
                            float cy = ms.roi.y + ms.roi.height * 0.5f;
                            float nw = ms.roi.width  * cfg_.roiGrow;
                            float nh = ms.roi.height * cfg_.roiGrow;
                            ms.roi = cv::Rect((int)(cx-nw*0.5f),(int)(cy-nh*0.5f),(int)nw,(int)nh);
                            if ((float)ms.roi.area()/frameArea > cfg_.roiAreaMax
                                    || ms.failCount >= cfg_.roiFailMax) {
                                ms.state = RoiState::GLOBAL;
                                ms.failCount = 0;
                            }
                            break;
                        }
                        case RoiState::GLOBAL:
                            if (++ms.globalFrames > cfg_.globalReset && ms.kfInit) {
                                ms.kfInit = false;
                                ms.globalFrames = 0;
                            }
                            break;
                        }
                    }
                }
                for (auto& [id, b] : best)
                    if (!markerStates_.count(id)) applyDetection(markerStates_[id], b.c);

                // ── Pose output — draw directly onto frame (no extra clone) ───
                std::vector<RobotPose> outRobots;
                cv::Mat debug = std::move(frame); // reuse captured frame buffer

                for (auto& [id, b] : best) {
                    auto& c  = b.c;
                    auto& ms = markerStates_[id];
                    float pcx = ms.kfInit ? ms.center.x : centroid(c, 0);
                    float pcy = ms.kfInit ? ms.center.y : centroid(c, 1);
                    cv::Point2f fwd = (c[0] + c[1]) * 0.5f - cv::Point2f(pcx, pcy);

                    float wx, wy, wyaw;
                    if (hasH_) {
                        std::vector<cv::Point2f> in{{pcx,pcy},{pcx+fwd.x,pcy+fwd.y}}, out;
                        cv::perspectiveTransform(in, out, H_);
                        wx = out[0].x; wy = out[0].y;
                        cv::Point2f wfwd = out[1] - out[0];
                        wyaw = (float)(std::atan2(wfwd.y, wfwd.x) * 180.0 / M_PI);
                    } else {
                        wx = pcx; wy = pcy;
                        wyaw = (float)(std::atan2(fwd.y, fwd.x) * 180.0 / M_PI);
                    }
                    outRobots.push_back({id, wx, wy, wyaw, pcx, pcy});

                    for (int k = 0; k < 4; ++k)
                        cv::line(debug, c[k], c[(k+1)%4], {0,255,0}, 3);
                    cv::circle(debug, {(int)pcx,(int)pcy}, 5, {0,0,255}, -1);
                    if (cv::norm(fwd) > 1.0f) {
                        cv::Point2f tip(pcx + fwd.x*2, pcy + fwd.y*2);
                        cv::arrowedLine(debug, {(int)pcx,(int)pcy}, {(int)tip.x,(int)tip.y},
                                       {0,255,255}, 2);
                    }
                    cv::putText(debug, std::to_string(id), {(int)c[0].x, (int)c[0].y - 10},
                                cv::FONT_HERSHEY_SIMPLEX, 1.2, {255,255,0}, 2);
                }

                if (cfg_.debugOverlay) drawDebugOverlay(debug, fps, outRobots.size());
                else cv::putText(debug,
                    "tags:" + std::to_string(outRobots.size()) + (hasH_ ? "  world" : "  px"),
                    {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);

                // ── Latency EMA ───────────────────────────────────────────────
                float ms_elapsed = std::chrono::duration<float, std::milli>(Clock::now() - t0).count();
                emaLatency = (emaLatency == 0.f) ? ms_elapsed : 0.9f * emaLatency + 0.1f * ms_elapsed;

                // ── Publish ───────────────────────────────────────────────────
                {
                    std::unique_lock<std::mutex> lk(resultMutex_);
                    latestResult_.robots    = std::move(outRobots);
                    latestResult_.debug     = std::move(debug);
                    latestResult_.fps       = fps;
                    latestResult_.latencyMs = emaLatency;
                    latestResult_.fresh     = true;
                }

            } catch (const std::exception& e) {
                throttledErr("[aruco] ", e.what());
            } catch (...) {
                throttledErr("[aruco] ", "unknown exception");
            }
        }
    }

    void stopThreads() {
        captureRunning_   = false;
        detectionRunning_ = false;
        frameCv_.notify_all();
        if (captureThread_.joinable())   captureThread_.join();
        if (detectionThread_.joinable()) detectionThread_.join();
        source_.reset();
    }

    void applyDetection(MarkerState& ms, const std::vector<cv::Point2f>& c) {
        float xs[]={c[0].x,c[1].x,c[2].x,c[3].x}, ys[]={c[0].y,c[1].y,c[2].y,c[3].y};
        float minx=*std::min_element(xs,xs+4), maxx=*std::max_element(xs,xs+4);
        float miny=*std::min_element(ys,ys+4), maxy=*std::max_element(ys,ys+4);
        float mx=(minx+maxx)*0.5f, my=(miny+maxy)*0.5f;
        ms.bboxSize = {maxx-minx, maxy-miny};
        if (!ms.kfInit) {
            initKalman(ms.kf, mx, my);
            ms.kfInit = true;
            ms.center = {mx, my};
        } else {
            cv::Mat meas = (cv::Mat_<float>(2,1) << mx, my);
            cv::Mat corr = ms.kf.correct(meas);
            ms.center = {corr.at<float>(0), corr.at<float>(1)};
        }
        float hw = ms.bboxSize.width * cfg_.roiPad, hh = ms.bboxSize.height * cfg_.roiPad;
        ms.roi = cv::Rect((int)(ms.center.x-hw),(int)(ms.center.y-hh),(int)(hw*2),(int)(hh*2));
        ms.state = RoiState::LOCAL;
        ms.failCount = 0;
        ms.globalFrames = 0;
        ms.roiLastDetected = std::chrono::steady_clock::now();
    }

    void initKalman(cv::KalmanFilter& kf, float x, float y) const {
        kf.init(4, 2, 0, CV_32F);
        kf.transitionMatrix = (cv::Mat_<float>(4,4) <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);
        kf.measurementMatrix = (cv::Mat_<float>(2,4) << 1, 0, 0, 0,  0, 1, 0, 0);
        // Rescale pixel-space process noise from the tuned reference resolution
        // to the live frame size (area ratio = linear pixel-scale²). Measurement
        // noise is left as-is: sub-pixel corner error is ~constant in px
        // regardless of marker size, so only Q needs to follow the resolution.
        // See cfg_.kfRefWidth for why.
        float resScale2 = 1.0f;
        if (cfg_.kfRefWidth > 0 && cfg_.kfRefHeight > 0)
            resScale2 = (fw_ * fh_) / ((float)cfg_.kfRefWidth * (float)cfg_.kfRefHeight);
        cv::setIdentity(kf.processNoiseCov,     cv::Scalar(cfg_.kfProcPos * resScale2));
        kf.processNoiseCov.at<float>(2,2) = cfg_.kfProcVel * resScale2;
        kf.processNoiseCov.at<float>(3,3) = cfg_.kfProcVel * resScale2;
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(cfg_.kfMeas));
        cv::setIdentity(kf.errorCovPost,        cv::Scalar(cfg_.kfInitCov));
        kf.statePost = (cv::Mat_<float>(4,1) << x, y, 0.f, 0.f);
    }

    void drawDebugOverlay(cv::Mat& debug, float fps, size_t robotCount) {
        auto now = std::chrono::steady_clock::now();
        for (auto& [id, ms] : markerStates_) {
            cv::Scalar col = ms.state == RoiState::LOCAL     ? cv::Scalar(0,255,0)
                           : ms.state == RoiState::EXPANDING ? cv::Scalar(0,255,255)
                                                              : cv::Scalar(0,0,255);
            bool roiVisible = ms.roiLastDetected != std::chrono::steady_clock::time_point::min()
                           && std::chrono::duration<float>(now - ms.roiLastDetected).count() < 5.0f;
            cv::Rect r = ms.roi & cv::Rect(0, 0, debug.cols, debug.rows);
            if (r.area() > 0 && roiVisible) cv::rectangle(debug, r, col, 1);
            if (ms.kfInit) {
                int px = std::clamp((int)ms.predicted.x, 6, debug.cols-7);
                int py = std::clamp((int)ms.predicted.y, 6, debug.rows-7);
                cv::line(debug, {px-6,py},{px+6,py},{255,255,255},1);
                cv::line(debug, {px,py-6},{px,py+6},{255,255,255},1);
            }
        }
        std::string hud = "det_fps:" + std::to_string((int)fps)
                        + "  tags:" + std::to_string(robotCount)
                        + (hasH_ ? "  world" : "  px");
        for (auto& [id, ms] : markerStates_)
            hud += "  " + std::to_string(id) + ":"
                + (ms.state==RoiState::LOCAL ? "L" : ms.state==RoiState::EXPANDING ? "E" : "G");
        cv::putText(debug, hud, {10,28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);
        cv::putText(debug, "cross=pred  ring=corr  ROI: G=red E=yellow L=green",
                    {10,52}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {180,180,180}, 1);
    }

    void throttledErr(const char* prefix, const char* msg) {
        static auto last = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();
        if (now - last > std::chrono::milliseconds(500)) {
            fprintf(stderr, "%s%s\n", prefix, msg);
            last = now;
        }
    }

    static float centroid(const std::vector<cv::Point2f>& c, int dim) {
        float s = 0;
        for (auto& pt : c) s += (dim == 0 ? pt.x : pt.y);
        return s / 4.0f;
    }

    // ── Config & detector ─────────────────────────────────────────────────────
    ArucoConfig cfg_;
    float fw_ = 1920, fh_ = 1080;

    std::unique_ptr<ICameraSource> source_;
    cv::aruco::ArucoDetector       detector_;
    cv::Ptr<cv::CLAHE>             clahe_;  // null if disabled; applied lazily per-crop

    std::vector<std::unique_ptr<IPreprocessor>> preprocessors_;

    cv::Mat H_;
    bool    hasH_ = false;

    // ── Capture thread ────────────────────────────────────────────────────────
    std::thread             captureThread_;
    std::atomic<bool>       captureRunning_{false};
    std::mutex              frameMutex_;
    std::condition_variable frameCv_;
    cv::Mat                 latestFrame_;

    // ── Detection thread ──────────────────────────────────────────────────────
    std::thread                              detectionThread_;
    std::atomic<bool>                        detectionRunning_{false};
    std::atomic<bool>                        statsReset_{false};
    std::unordered_map<int, MarkerState>     markerStates_; // detection thread only

    // ── Result (detection → main thread, guarded by resultMutex_) ────────────
    std::mutex      resultMutex_;
    DetectionResult latestResult_;

    // ── Main-thread-visible state (written only in update()) ──────────────────
    std::vector<RobotPose> robots_;
    cv::Mat                debug_;
    float                  fps_       = 0.f;
    float                  latencyMs_ = 0.f;
};
