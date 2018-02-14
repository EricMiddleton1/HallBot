#pragma once

#include <raspicam/raspicam_cv.h>

#include "VideoDevice.hpp"
#include "DeviceRegistration.hpp"

class RaspicamDevice : public VideoDevice {
public:
  RaspicamDevice(const std::vector<Param>& params);

  bool getFrame(cv::Mat& out) override;
private:
  static DeviceRegistration registration_;
  raspicam::RaspiCam_Cv camera;
};
