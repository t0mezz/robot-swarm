#pragma once
// BaslerPylonSource — ICameraSource implementation for Basler ace2 GigE cameras.
// Include ONLY via aruco_tracker.h (ICameraSource and ArucoConfig must be in scope).

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <cstdio>

class BaslerPylonSource : public ICameraSource {
public:
    BaslerPylonSource()  { Pylon::PylonInitialize(); }
    ~BaslerPylonSource() {
        if (camera_.IsGrabbing()) camera_.StopGrabbing();
        if (camera_.IsOpen())    camera_.Close();
        Pylon::PylonTerminate();
    }

    bool open(const ArucoConfig& cfg) override {
        try {
            Pylon::CTlFactory& tl = Pylon::CTlFactory::GetInstance();

            Pylon::CDeviceInfo di;
            if (!cfg.baslerSerial.empty())
                di.SetSerialNumber(cfg.baslerSerial.c_str());
            else if (!cfg.baslerIp.empty())
                di.SetIpAddress(cfg.baslerIp.c_str());

            camera_.Attach(tl.CreateDevice(di));
            camera_.Open();

            auto& nm = camera_.GetNodeMap();

            // Zero the ROI offset before resizing: GenICam rejects Width/Height
            // changes that would push offset+size past the sensor bounds, and a
            // nonzero offset left over from a previous run can trigger exactly
            // that if the new width is larger.
            try { Pylon::CIntegerParameter(nm, "OffsetX").SetValue(0); } catch (...) {}
            try { Pylon::CIntegerParameter(nm, "OffsetY").SetValue(0); } catch (...) {}

            Pylon::CIntegerParameter(nm, "Width").SetValue(cfg.width);
            Pylon::CIntegerParameter(nm, "Height").SetValue(cfg.height);

            // Sensor ROI offset — shifts the readout window on the sensor itself
            // (camera.OffsetX.SetValue()/OffsetY per Basler's pylon API), rather
            // than nudging detected pixel coordinates after the fact.
            if (cfg.offsetX != 0 || cfg.offsetY != 0) {
                try {
                    Pylon::CIntegerParameter(nm, "OffsetX").SetValue(cfg.offsetX);
                    Pylon::CIntegerParameter(nm, "OffsetY").SetValue(cfg.offsetY);
                } catch (const Pylon::GenericException& e) {
                    fprintf(stderr, "[basler] failed to set OffsetX/OffsetY (%d,%d): %s\n",
                            cfg.offsetX, cfg.offsetY, e.GetDescription());
                }
            }

            width_  = (int)Pylon::CIntegerParameter(nm, "Width").GetValue();
            height_ = (int)Pylon::CIntegerParameter(nm, "Height").GetValue();
            int offsetX = (int)Pylon::CIntegerParameter(nm, "OffsetX").GetValue();
            int offsetY = (int)Pylon::CIntegerParameter(nm, "OffsetY").GetValue();

            // Set target frame rate so the camera doesn't produce more frames than
            // the detection pipeline can consume. Not all firmware versions support
            // this node, so failures are silently ignored.
            try {
                Pylon::CBooleanParameter(nm, "AcquisitionFrameRateEnable").SetValue(true);
                Pylon::CFloatParameter(nm, "AcquisitionFrameRate").SetValue((double)cfg.fps);
            } catch (...) {}

            converter_.OutputPixelFormat = Pylon::PixelType_BGR8packed;

            // LatestImageOnly: always return the newest frame, discard queued ones.
            camera_.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

            // ResultingFrameRate is the camera's own estimate of what it can
            // actually sustain right now given current exposure/bandwidth —
            // often well below the requested AcquisitionFrameRate, and the
            // most direct answer to "why is my captured fps lower than
            // expected". Not every firmware exposes it, so failure is quiet.
            float resultingFps = -1.f;
            try { resultingFps = (float)Pylon::CFloatParameter(nm, "ResultingFrameRate").GetValue(); }
            catch (...) {}

            printf("[basler] open  serial=%s  %dx%d  offset=(%d,%d)  fps=%d\n",
                   camera_.GetDeviceInfo().GetSerialNumber().c_str(),
                   width_, height_, offsetX, offsetY, cfg.fps);
            if (resultingFps >= 0.f)
                printf("[basler] camera-reported sustainable rate right now: %.1f fps "
                       "(depends on current exposure/bandwidth, not fixed)\n", resultingFps);
            return true;
        } catch (const Pylon::GenericException& e) {
            fprintf(stderr, "[basler] open failed: %s\n", e.GetDescription());
            return false;
        }
    }

    bool read(cv::Mat& frame) override {
        try {
            Pylon::CGrabResultPtr result;
            camera_.RetrieveResult(5000, result, Pylon::TimeoutHandling_Return);
            if (!result || !result->GrabSucceeded()) return false;

            Pylon::CPylonImage img;
            converter_.Convert(img, result);
            // Clone so the Pylon buffer can be released immediately.
            frame = cv::Mat(height_, width_, CV_8UC3, img.GetBuffer()).clone();
            return !frame.empty();
        } catch (const Pylon::GenericException& e) {
            fprintf(stderr, "[basler] grab: %s\n", e.GetDescription());
            return false;
        }
    }

    cv::Size size() const override { return {width_, height_}; }

    float temperature() override {
        try {
            return (float)Pylon::CFloatParameter(
                camera_.GetNodeMap(), "DeviceTemperature").GetValue();
        } catch (...) { return -1.f; }
    }

private:
    Pylon::CBaslerUniversalInstantCamera camera_;
    Pylon::CImageFormatConverter         converter_;
    int width_  = 0;
    int height_ = 0;
};
