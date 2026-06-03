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
            Pylon::CIntegerParameter(nm, "Width").SetValue(cfg.width);
            Pylon::CIntegerParameter(nm, "Height").SetValue(cfg.height);

            width_  = (int)Pylon::CIntegerParameter(nm, "Width").GetValue();
            height_ = (int)Pylon::CIntegerParameter(nm, "Height").GetValue();

            converter_.OutputPixelFormat = Pylon::PixelType_BGR8packed;

            // LatestImageOnly: always return the newest frame, discard queued ones.
            camera_.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

            printf("[basler] open  serial=%s  %dx%d\n",
                   camera_.GetDeviceInfo().GetSerialNumber().c_str(),
                   width_, height_);
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
