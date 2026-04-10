#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include <future>
#include <cmath>
#include <chrono>

// ─── RobotPose ────────────────────────────────────────────────────────────────
// Identical to aruco_tracker.h — circle_demo uses this type directly.

struct RobotPose {
    int   id;
    float x, y;   // world coords (mm) if homography set, else pixels
    float yaw;    // degrees CCW from world +X
    float px, py; // raw pixel centroid
};

// ─── AprilConfig ──────────────────────────────────────────────────────────────
// All fields have working defaults for AprilTag 36h11.
// Override any subset via april_vision_config.json — missing keys fall back
// to the defaults below.
//
// dictId values (cv::aruco enum):
//   DICT_APRILTAG_16h5  = 17  (30 tags,  Hamming-5)
//   DICT_APRILTAG_25h9  = 18  (35 tags,  Hamming-9)
//   DICT_APRILTAG_36h10 = 19  (2320 tags, Hamming-10)
//   DICT_APRILTAG_36h11 = 20  (587 tags,  Hamming-11)  ← default

struct AprilConfig {
    // Camera
    int   camIndex = 0, width = 1920, height = 1080, fps = 30;

    // Detector — OpenCV uses its ArUco engine with AprilTag dictionaries
    int   dictId        = 20;    // DICT_APRILTAG_36h11
    int   winMin        = 3, winMax = 23, winStep = 2;
    float threshC       = 7.0f;
    float minPerimRate  = 0.02f;
    float polyApprox    = 0.05f;
    int   pixPerCell    = 10;
    float cellMargin    = 0.20f;
    float errorCorr     = 0.60f; // AprilTags have strong ECC; slightly lower than ArUco default
    float minOtsuStdDev = 5.0f;
    int   cornerWin     = 5, cornerMaxIter = 30;
    bool  halfResSweep  = true;  // full-frame pass on 0.5× image (~4× fewer pixels)

    // Kalman filter (constant-velocity, per marker)
    float kfProcPos = 1e-4f;
    float kfProcVel = 1e-2f;
    float kfMeas    = 4.0f;
    float kfInitCov = 100.0f;

    // ROI state machine
    float roiPad      = 1.25f;
    float roiGrow     = 1.20f;
    int   roiFailMax  = 5;
    float roiAreaMax  = 0.40f;
    int   globalReset = 30;

    // CLAHE (pre-detection contrast enhancement; set claheClip=0 to disable)
    float claheClip = 2.0f;
    int   claheTile = 8;

    bool debugOverlay = false;

    static AprilConfig fromFile(const std::string& path);
};

inline AprilConfig AprilConfig::fromFile(const std::string& path) {
    AprilConfig c;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        fprintf(stderr, "[april] config '%s' not found — using defaults\n", path.c_str());
        return c;
    }
    auto ri = [&](const char* k, int&   v) { if (!fs[k].empty()) fs[k] >> v; };
    auto rf = [&](const char* k, float& v) { if (!fs[k].empty()) fs[k] >> v; };
    auto rb = [&](const char* k, bool&  v) { int t = (int)v; if (!fs[k].empty()) { fs[k] >> t; v = (bool)t; } };
    ri("cam_index",       c.camIndex);
    ri("cam_width",       c.width);
    ri("cam_height",      c.height);
    ri("cam_fps",         c.fps);
    ri("dict_id",         c.dictId);
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
    rf("roi_pad",         c.roiPad);
    rf("roi_grow",        c.roiGrow);
    ri("roi_fail_max",    c.roiFailMax);
    rf("roi_area_max",    c.roiAreaMax);
    ri("global_reset",    c.globalReset);
    rf("clahe_clip",      c.claheClip);
    ri("clahe_tile",      c.claheTile);
    rb("debug_overlay",   c.debugOverlay);
    return c;
}

// ─── Preprocessor pipeline ────────────────────────────────────────────────────

struct IPreprocessor {
    virtual ~IPreprocessor() = default;
    virtual void process(cv::Mat& gray) = 0;
};

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
        fprintf(stderr, "[april] fisheye calibration loaded from '%s'\n", calibYaml.c_str());
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

// ─── AprilTracker ─────────────────────────────────────────────────────────────
// Drop-in replacement for ArucoTracker.  Public API is identical so that
// circle_demo only needs to change the #include and the two type names.

class AprilTracker {
public:
    explicit AprilTracker(AprilConfig cfg = {}) : cfg_(cfg) {
        auto dict   = cv::aruco::getPredefinedDictionary(cfg_.dictId);
        auto params = cv::aruco::DetectorParameters();
        params.adaptiveThreshWinSizeMin              = cfg_.winMin;
        params.adaptiveThreshWinSizeMax              = cfg_.winMax;
        params.adaptiveThreshWinSizeStep             = cfg_.winStep;
        params.adaptiveThreshConstant                = cfg_.threshC;
        params.minMarkerPerimeterRate                = cfg_.minPerimRate;
        params.polygonalApproxAccuracyRate           = cfg_.polyApprox;
        params.perspectiveRemovePixelPerCell         = cfg_.pixPerCell;
        params.perspectiveRemoveIgnoredMarginPerCell = cfg_.cellMargin;
        params.errorCorrectionRate                   = cfg_.errorCorr;
        params.minOtsuStdDev                         = cfg_.minOtsuStdDev;
        params.minMarkerDistanceRate                 = 0.0;
        params.cornerRefinementMethod                = cv::aruco::CORNER_REFINE_SUBPIX;
        params.cornerRefinementWinSize               = cfg_.cornerWin;
        params.cornerRefinementMaxIterations         = cfg_.cornerMaxIter;
        cv::aruco::RefineParameters refine;
        refine.minRepDistance      = 10.0f;
        refine.errorCorrectionRate = 3.0f;
        refine.checkAllOrders      = true;
        detector_ = cv::aruco::ArucoDetector(dict, params, refine);

        if (cfg_.claheClip > 0)
            preprocessors_.push_back(
                std::make_unique<CLAHEPreprocessor>(cfg_.claheClip, cfg_.claheTile));
    }

    ~AprilTracker() { stopCapture(); }

    bool open() {
        cap_.open(cfg_.camIndex, cv::CAP_AVFOUNDATION);
        if (!cap_.isOpened()) return false;
        cap_.set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
        cap_.set(cv::CAP_PROP_FPS,          cfg_.fps);
        fw_ = (float)cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        fh_ = (float)cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        captureRunning_ = true;
        captureThread_  = std::thread(&AprilTracker::captureLoop, this);
        return true;
    }

    bool update() {
        try {
            {
                std::unique_lock<std::mutex> lk(frameMutex_);
                if (latestFrame_.empty()) return false;
                latestFrame_.copyTo(frame_);
            }

            cv::Mat gray;
            cv::cvtColor(frame_, gray, cv::COLOR_BGR2GRAY);
            for (auto& p : preprocessors_) p->process(gray);

            cv::Mat sweep = gray;
            if (cfg_.halfResSweep)
                cv::resize(gray, sweep, {}, 0.5, 0.5, cv::INTER_AREA);

            ++frameIdx_;
            updateFps();

            // ── Kalman predict ────────────────────────────────────────────────
            for (auto& [id, ms] : markerStates_) {
                if (!ms.kfInit) continue;
                cv::Mat pred  = ms.kf.predict();
                ms.center = ms.predicted = {pred.at<float>(0), pred.at<float>(1)};
                if (ms.state != RoiState::GLOBAL) {
                    float hw = ms.roi.width * 0.5f, hh = ms.roi.height * 0.5f;
                    ms.roi = cv::Rect(
                        (int)(ms.center.x - hw), (int)(ms.center.y - hh),
                        ms.roi.width, ms.roi.height);
                }
            }

            // ── Detection ─────────────────────────────────────────────────────
            bool needGlobal = markerStates_.empty();
            for (auto& [_, ms] : markerStates_)
                if (ms.state == RoiState::GLOBAL) { needGlobal = true; break; }

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
                std::vector<std::vector<cv::Point2f>> cs; std::vector<int> ids;
                std::vector<std::vector<cv::Point2f>> rej;
                detector_.detectMarkers(sweep, cs, ids, rej);
                float scale = cfg_.halfResSweep ? 2.0f : 1.0f;
                for (int j = 0; j < (int)ids.size(); ++j)
                    merge(ids[j], cs[j], scale, {0, 0});
            }

            // ROI crops in parallel — each crop targets a single known marker
            using RoiHits = std::vector<std::pair<int, std::vector<cv::Point2f>>>;
            std::vector<std::future<RoiHits>> futs;
            for (auto& [sid, sms] : markerStates_) {
                if (sms.state == RoiState::GLOBAL) continue;
                futs.push_back(std::async(std::launch::async,
                    [this, &gray, roi = sms.roi, id = sid]() -> RoiHits {
                        RoiHits out;
                        cv::Rect r = roi & cv::Rect(0, 0, gray.cols, gray.rows);
                        if (r.area() < 100) return out;
                        std::vector<std::vector<cv::Point2f>> cs; std::vector<int> ids;
                        std::vector<std::vector<cv::Point2f>> rej;
                        detector_.detectMarkers(gray(r), cs, ids, rej);
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

            // ── ROI state machine update ───────────────────────────────────────
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

            // ── Pose output ───────────────────────────────────────────────────
            robots_.clear();
            debug_ = frame_.clone();

            for (auto& [id, b] : best) {
                auto& c  = b.c;
                auto& ms = markerStates_[id];
                float pcx = ms.kfInit ? ms.center.x : centroid(c, 0);
                float pcy = ms.kfInit ? ms.center.y : centroid(c, 1);
                // AprilTag corner order: 0=bottom-left, 1=bottom-right,
                // 2=top-right, 3=top-left (opposite winding to ArUco).
                // Forward direction = midpoint of top edge (corners 2+3) → centroid.
                cv::Point2f fwd = (c[2] + c[3]) * 0.5f - cv::Point2f(pcx, pcy);

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
                robots_.push_back({id, wx, wy, wyaw, pcx, pcy});

                for (int k = 0; k < 4; ++k)
                    cv::line(debug_, c[k], c[(k+1)%4], {0,255,0}, 3);
                cv::circle(debug_, {(int)pcx,(int)pcy}, 5, {0,0,255}, -1);
                if (cv::norm(fwd) > 1.0f) {
                    cv::Point2f tip(pcx + fwd.x*2, pcy + fwd.y*2);
                    cv::arrowedLine(debug_, {(int)pcx,(int)pcy}, {(int)tip.x,(int)tip.y},
                                   {0,255,255}, 2);
                }
                cv::putText(debug_, std::to_string(id), {(int)c[0].x, (int)c[0].y - 10},
                            cv::FONT_HERSHEY_SIMPLEX, 1.2, {255,255,0}, 2);
            }

            if (cfg_.debugOverlay) drawDebugOverlay();
            else cv::putText(debug_,
                "tags:" + std::to_string(robots_.size()) + (hasH_ ? "  world" : "  px"),
                {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);

            return true;
        } catch (const std::exception& e) {
            throttledErr("[april] ", e.what()); return false;
        } catch (...) {
            throttledErr("[april] unknown exception", ""); return false;
        }
    }

    // Homography: pixel → world (mm)
    void setHomography(const std::vector<cv::Point2f>& pix,
                       const std::vector<cv::Point2f>& world) {
        H_ = cv::findHomography(pix, world);
        hasH_ = !H_.empty();
    }
    bool loadHomography(const std::string& path) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        fs["H"] >> H_;
        hasH_ = !H_.empty();
        return hasH_;
    }
    void saveHomography(const std::string& path) const {
        if (!hasH_) return;
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "H" << H_;
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
    bool     isOpen()     const { return cap_.isOpened(); }
    float    fps()        const { return fps_; }

    static void drawText(cv::Mat& img, const std::string& text, cv::Point org,
                         int fontSize, cv::Scalar color) {
        if (img.empty()) return;
        org.x = std::max(0, std::min(org.x, img.cols - 1));
        org.y = std::max(fontSize, std::min(org.y, img.rows - 1));
        cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX,
                    fontSize / 28.0, color, 2, cv::LINE_AA);
    }

private:
    enum class RoiState { GLOBAL, LOCAL, EXPANDING };

    struct MarkerState {
        RoiState        state = RoiState::GLOBAL;
        cv::Rect        roi;
        cv::Point2f     center, predicted;
        cv::Size2f      bboxSize;
        int             failCount = 0, globalFrames = 0;
        cv::KalmanFilter kf;
        bool            kfInit = false;
    };

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
    }

    void initKalman(cv::KalmanFilter& kf, float x, float y) const {
        kf.init(4, 2, 0, CV_32F);
        kf.transitionMatrix = (cv::Mat_<float>(4,4) <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);
        kf.measurementMatrix = (cv::Mat_<float>(2,4) << 1, 0, 0, 0,  0, 1, 0, 0);
        cv::setIdentity(kf.processNoiseCov,     cv::Scalar(cfg_.kfProcPos));
        kf.processNoiseCov.at<float>(2,2) = cfg_.kfProcVel;
        kf.processNoiseCov.at<float>(3,3) = cfg_.kfProcVel;
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(cfg_.kfMeas));
        cv::setIdentity(kf.errorCovPost,        cv::Scalar(cfg_.kfInitCov));
        kf.statePost = (cv::Mat_<float>(4,1) << x, y, 0.f, 0.f);
    }

    void captureLoop() {
        while (captureRunning_) {
            cv::Mat f;
            if (cap_.read(f) && !f.empty()) {
                std::unique_lock<std::mutex> lk(frameMutex_);
                latestFrame_ = std::move(f);
            }
        }
    }
    void stopCapture() {
        captureRunning_ = false;
        if (captureThread_.joinable()) captureThread_.join();
    }

    void updateFps() {
        auto now = std::chrono::steady_clock::now();
        if (fpsFrames_++ == 0) lastFpsTime_ = now;
        float el = std::chrono::duration<float>(now - lastFpsTime_).count();
        if (el >= 1.0f) { fps_ = fpsFrames_ / el; fpsFrames_ = 0; lastFpsTime_ = now; }
    }

    static float centroid(const std::vector<cv::Point2f>& c, int dim) {
        float s = 0;
        for (auto& pt : c) s += (dim == 0 ? pt.x : pt.y);
        return s / 4.0f;
    }

    void drawDebugOverlay() {
        for (auto& [id, ms] : markerStates_) {
            cv::Scalar col = ms.state == RoiState::LOCAL     ? cv::Scalar(0,255,0)
                           : ms.state == RoiState::EXPANDING ? cv::Scalar(0,255,255)
                                                              : cv::Scalar(0,0,255);
            cv::Rect r = ms.roi & cv::Rect(0, 0, debug_.cols, debug_.rows);
            if (r.area() > 0) cv::rectangle(debug_, r, col, 1);
            if (ms.kfInit) {
                int px = std::clamp((int)ms.predicted.x, 6, debug_.cols-7);
                int py = std::clamp((int)ms.predicted.y, 6, debug_.rows-7);
                cv::line(debug_, {px-6,py},{px+6,py},{255,255,255},1);
                cv::line(debug_, {px,py-6},{px,py+6},{255,255,255},1);
            }
        }
        std::string hud = "FPS:" + std::to_string((int)fps_)
                        + "  tags:" + std::to_string(robots_.size())
                        + (hasH_ ? "  world" : "  px");
        for (auto& [id, ms] : markerStates_)
            hud += "  " + std::to_string(id) + ":"
                + (ms.state==RoiState::LOCAL ? "L" : ms.state==RoiState::EXPANDING ? "E" : "G");
        cv::putText(debug_, hud, {10,28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);
        cv::putText(debug_, "cross=pred  ring=corr  ROI: G=red E=yellow L=green",
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

    AprilConfig cfg_;
    float fw_ = 1920, fh_ = 1080;

    cv::VideoCapture         cap_;
    cv::aruco::ArucoDetector detector_;

    std::vector<std::unique_ptr<IPreprocessor>> preprocessors_;
    std::unordered_map<int, MarkerState>        markerStates_;

    cv::Mat  frame_, debug_, H_;
    bool     hasH_ = false;

    std::vector<RobotPose> robots_;

    int   frameIdx_ = 0, fpsFrames_ = 0;
    float fps_ = 0.0f;
    std::chrono::steady_clock::time_point lastFpsTime_;

    std::thread       captureThread_;
    std::mutex        frameMutex_;
    std::atomic<bool> captureRunning_{false};
    cv::Mat           latestFrame_;
};
