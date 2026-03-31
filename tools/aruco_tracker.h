#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/freetype.hpp>
#include <vector>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include <future>
#include <cmath>
#include <string>

struct RobotPose {
    int   id;
    float x, y;   // world coords (mm) if homography set, else pixels
    float yaw;    // degrees CCW from world +X
    float px, py; // pixel centre
};

class ArucoTracker {
    static constexpr int   FULL_FRAME_INTERVAL = 5;
    static constexpr float ROI_PAD_FRAC        = 0.6f;

public:
    ArucoTracker(int camIndex, float /*markerSizeMm*/)
        : camIdx_(camIndex), hasH_(false), frameIdx_(0)
    {
        auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        // Shared parameters
        auto baseParams = cv::aruco::DetectorParameters();
        baseParams.adaptiveThreshWinSizeMin              = 3;
        baseParams.adaptiveThreshWinSizeMax              = 53;
        baseParams.minMarkerPerimeterRate                = 0.01f;
        baseParams.polygonalApproxAccuracyRate           = 0.04f;
        baseParams.minMarkerDistanceRate                 = 0.0;
        baseParams.perspectiveRemovePixelPerCell         = 10;
        baseParams.perspectiveRemoveIgnoredMarginPerCell = 0.20f;
        baseParams.errorCorrectionRate                   = 0.6f;
        baseParams.minOtsuStdDev                         = 12.0;
        baseParams.cornerRefinementMethod                = cv::aruco::CORNER_REFINE_CONTOUR;

        // Full-frame sweep: finer threshold steps + slightly higher reflection threshold
        auto fullParams = baseParams;
        fullParams.adaptiveThreshWinSizeStep = 2;
        fullParams.adaptiveThreshWinSizeMax  = 35;  // 17 levels instead of 26; ~35% faster
        fullParams.adaptiveThreshConstant    = 24;  // high: good lighting, reject specular highlights
        detectorFull_ = cv::aruco::ArucoDetector(dict, fullParams);

        // ROI crops: coarser step is fine since crops are small
        auto roiParams = baseParams;
        roiParams.adaptiveThreshWinSizeStep = 4;
        roiParams.adaptiveThreshConstant    = 20;  // slightly lower than full: ROI local means are noisier
        detectorROI_ = cv::aruco::ArucoDetector(dict, roiParams);

        clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
    }

    ~ArucoTracker() { stopCapture(); }

    bool open() {
        cap_.open(camIdx_, cv::CAP_AVFOUNDATION);
        if (!cap_.isOpened()) return false;
        cap_.set(cv::CAP_PROP_FRAME_WIDTH,  1920);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap_.set(cv::CAP_PROP_FPS, 30);
        fw_ = (float)cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        fh_ = (float)cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        camMat_ = (cv::Mat_<double>(3,3) <<
            fw_,   0, fw_/2,
              0, fw_, fh_/2,
              0,   0,     1);
        distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        captureRunning_ = true;
        captureThread_  = std::thread(&ArucoTracker::captureLoop, this);

        ft2_ = cv::freetype::createFreeType2();
        const char* fonts[] = {
            "/System/Library/Fonts/Menlo.ttc",
            "/System/Library/Fonts/SFNSMono.ttf",
            "/System/Library/Fonts/Helvetica.ttc",
            nullptr
        };
        for (auto** p = fonts; *p; ++p) {
            try { ft2_->loadFontData(*p, 0); useFt2_ = true; break; }
            catch (...) {}
        }
        return true;
    }

    bool update() {
        try {
            std::unique_lock<std::mutex> lk(frameMutex_);
            if (latestFrame_.empty()) return false;
            latestFrame_.copyTo(frame_);

            cv::Mat gray;
            cv::cvtColor(frame_, gray, cv::COLOR_BGR2GRAY);
            clahe_->apply(gray, gray);

            // Half-resolution image for full-frame sweep: ~4x fewer pixels,
            // and reflection hot spots are averaged across threshold windows.
            cv::Mat grayHalf;
            cv::resize(gray, grayHalf, {}, 0.5, 0.5, cv::INTER_AREA);

            ++frameIdx_;

        bool fullSweep = roiCache_.empty() || (frameIdx_ % FULL_FRAME_INTERVAL == 0);

        struct Best { float perim; std::vector<cv::Point2f> c; };
        std::unordered_map<int, Best> best;

        // scale: multiply detected coords by this factor (2.0 for half-res full sweep, 1.0 for ROI)
        auto ingest = [&](const std::vector<std::vector<cv::Point2f>>& cs,
                          const std::vector<int>& ids, cv::Point2f off, float scale) {
            for (int j = 0; j < (int)ids.size(); ++j) {
                auto c = cs[j];
                for (auto& pt : c) { pt *= scale; pt += off; }
                // Reject before touching best or ROI cache — (int)NaN is UB
                bool ok = true;
                for (auto& pt : c)
                    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
                        pt.x < -fw_ || pt.x > 2*fw_ || pt.y < -fh_ || pt.y > 2*fh_)
                        { ok = false; break; }
                if (!ok) continue;
                float p = 0;
                for (int k = 0; k < 4; ++k) p += (float)cv::norm(c[k] - c[(k+1)%4]);
                if (!std::isfinite(p) || p < 1.0f) continue;
                int id = ids[j];
                if (!best.count(id) || p > best[id].perim)
                    best[id] = {p, std::move(c)};
            }
        };

        if (fullSweep) {
            std::vector<std::vector<cv::Point2f>> cs; std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> rej;
            detectorFull_.detectMarkers(grayHalf, cs, ids, rej);
            ingest(cs, ids, {0, 0}, 2.0f);  // scale back to full-res coords
        } else {
            // Parallel ROI detection — each crop is independent (gray is read-only,
            // detectorROI_ is not modified by detectMarkers).
            using RoiResult = std::vector<std::pair<int, std::vector<cv::Point2f>>>;
            std::vector<std::future<RoiResult>> futs;
            futs.reserve(roiCache_.size());

            for (auto& [id, roi] : roiCache_) {
                futs.push_back(std::async(std::launch::async,
                    [this, &gray, id=id, roi=roi]() -> RoiResult {
                        RoiResult out;
                        cv::Rect r = roi & cv::Rect(0, 0, gray.cols, gray.rows);
                        if (r.area() < 100) return out;
                        std::vector<std::vector<cv::Point2f>> cs;
                        std::vector<int> ids;
                        std::vector<std::vector<cv::Point2f>> rej;
                        detectorROI_.detectMarkers(gray(r), cs, ids, rej);
                        cv::Point2f off((float)r.x, (float)r.y);
                        for (int j = 0; j < (int)ids.size(); ++j) {
                            auto c = cs[j];
                            for (auto& pt : c) pt += off;
                            out.push_back({ids[j], std::move(c)});
                        }
                        return out;
                    }));
            }

            for (auto& f : futs) {
                try {
                    for (auto& [id, c] : f.get()) {
                        // Validate and merge into best (same logic as ingest)
                        bool ok = true;
                        for (auto& pt : c)
                            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
                                pt.x < -fw_ || pt.x > 2*fw_ || pt.y < -fh_ || pt.y > 2*fh_)
                                { ok = false; break; }
                        if (!ok) continue;
                        float p = 0;
                        for (int k = 0; k < 4; ++k) p += (float)cv::norm(c[k] - c[(k+1)%4]);
                        if (!std::isfinite(p) || p < 1.0f) continue;
                        if (!best.count(id) || p > best[id].perim)
                            best[id] = {p, std::move(c)};
                    }
                } catch (...) {}  // task exceptions must not crash the main loop
            }
        }

        roiCache_.clear();
        for (auto& [id, b] : best) {
            auto& c = b.c;
            float xs[] = {c[0].x,c[1].x,c[2].x,c[3].x};
            float ys[] = {c[0].y,c[1].y,c[2].y,c[3].y};
            float minx = *std::min_element(xs,xs+4), maxx = *std::max_element(xs,xs+4);
            float miny = *std::min_element(ys,ys+4), maxy = *std::max_element(ys,ys+4);
            float pw = maxx-minx, ph = maxy-miny;
            cv::Point2f cur((minx+maxx)*0.5f, (miny+maxy)*0.5f);
            float vel = prevCenters_.count(id) ? (float)cv::norm(cur - prevCenters_[id]) : 0.f;
            prevCenters_[id] = cur;
            float pad = std::max(24.0f, std::max(pw,ph) * ROI_PAD_FRAC + vel * 1.5f);
            roiCache_[id] = cv::Rect((int)(minx-pad),(int)(miny-pad),
                                     (int)(pw+2*pad),(int)(ph+2*pad));
        }

        robots_.clear();
        debug_ = frame_.clone();

        for (auto& [id, b] : best) {
            auto& c = b.c;
            float pcx = 0, pcy = 0;
            for (auto& pt : c) { pcx += pt.x; pcy += pt.y; }
            pcx /= 4; pcy /= 4;

            cv::Point2f topMid = (c[0] + c[1]) * 0.5f;
            cv::Point2f fwdImg = topMid - cv::Point2f(pcx, pcy);

            float wx, wy, wyaw;
            if (hasH_) {
                std::vector<cv::Point2f> pxPts = {{pcx,pcy},{pcx+fwdImg.x,pcy+fwdImg.y}};
                std::vector<cv::Point2f> wPts;
                cv::perspectiveTransform(pxPts, wPts, H_);
                wx = wPts[0].x; wy = wPts[0].y;
                cv::Point2f wFwd = wPts[1] - wPts[0];
                wyaw = (float)(std::atan2(wFwd.y, wFwd.x) * 180.0 / M_PI);
            } else {
                wx = pcx; wy = pcy;
                wyaw = (float)(std::atan2(fwdImg.y, fwdImg.x) * 180.0 / M_PI);
            }

            robots_.push_back({id, wx, wy, wyaw, pcx, pcy});

            for (int k = 0; k < 4; ++k)
                cv::line(debug_, c[k], c[(k+1)%4], {0,255,0}, 3);
            cv::circle(debug_, {(int)pcx,(int)pcy}, 5, {0,0,255}, -1);
            if (cv::norm(fwdImg) > 1.0f) {
                cv::Point2f tip(pcx + fwdImg.x*2, pcy + fwdImg.y*2);
                cv::arrowedLine(debug_, {(int)pcx,(int)pcy}, {(int)tip.x,(int)tip.y}, {0,255,255}, 2);
            }

            drawText(debug_, std::to_string(id), {(int)c[0].x,(int)c[0].y-10}, 36, {255,255,0});
        }

        drawText(debug_,
                 "tags:" + std::to_string(robots_.size()) + "  " + (hasH_ ? "world" : "pixels"),
                 {10, 28}, 20, {0,255,0});
        return true;
    } catch (const cv::Exception& e) {
        static auto lastArucoLog = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();
        if (now - lastArucoLog > std::chrono::milliseconds(500)) {
            fprintf(stderr, "[aruco] %s\n", e.what());
            lastArucoLog = now;
        }
        return false;
    } catch (const std::exception& e) {
        static auto lastStdLog = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();
        if (now - lastStdLog > std::chrono::milliseconds(500)) {
            fprintf(stderr, "[aruco] exception: %s\n", e.what());
            lastStdLog = now;
        }
        return false;
    } catch (...) {
        static auto lastAnyLog = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();
        if (now - lastAnyLog > std::chrono::milliseconds(500)) {
            fprintf(stderr, "[aruco] unknown exception\n");
            lastAnyLog = now;
        }
        return false;
    }
    }

    void setHomography(const std::vector<cv::Point2f>& pix,
                       const std::vector<cv::Point2f>& world) {
        H_ = cv::findHomography(pix, world);
        hasH_ = !H_.empty();
    }
    void saveHomography(const std::string& path) const {
        if (!hasH_) return;
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "H" << H_;
    }
    bool loadHomography(const std::string& path) {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        fs["H"] >> H_;
        hasH_ = !H_.empty();
        return hasH_;
    }

    void drawText(cv::Mat& img, const std::string& text, cv::Point org,
                  int fontSize, cv::Scalar color) const {
        if (img.empty()) return;
        org.x = std::max(0, std::min(org.x, img.cols - 1));
        org.y = std::max(fontSize, std::min(org.y, img.rows - 1));
        if (useFt2_) {
            try { ft2_->putText(img, text, org, fontSize, color, -1, cv::LINE_AA, false); }
            catch (...) { cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, fontSize/28.0, color, 2); }
        } else {
            cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, fontSize/28.0, color, 2);
        }
    }

    const std::vector<RobotPose>& robots()    const { return robots_; }
    cv::Mat  debugFrame() const { return debug_; }
    cv::Size frameSize()  const { return {(int)fw_, (int)fh_}; }
    bool     isOpen()     const { return cap_.isOpened(); }

private:
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

    int   camIdx_;
    float fw_ = 1920, fh_ = 1080;
    cv::VideoCapture         cap_;
    cv::aruco::ArucoDetector detectorFull_;
    cv::aruco::ArucoDetector detectorROI_;
    cv::Ptr<cv::CLAHE>       clahe_;
    cv::Mat                  frame_, debug_, camMat_, distCoeffs_, H_;
    bool                     hasH_;
    std::vector<RobotPose>   robots_;
    std::unordered_map<int, cv::Rect>     roiCache_;
    std::unordered_map<int, cv::Point2f>  prevCenters_;
    int                      frameIdx_;

    std::thread       captureThread_;
    std::mutex        frameMutex_;
    std::atomic<bool> captureRunning_{false};
    cv::Mat           latestFrame_;

    cv::Ptr<cv::freetype::FreeType2> ft2_;
    bool useFt2_ = false;
};
