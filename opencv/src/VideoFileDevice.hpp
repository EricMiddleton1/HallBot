#pragma once

#include <opencv2/videoio.hpp>

#include "VideoDevice.hpp"
#include "DeviceRegistration.hpp"

class VideoFileDevice : public VideoDevice {
public:
  VideoFileDevice(std::vector<Param>&& params);

  bool getFrame(cv::Mat& out) override;
private:
  static DeviceRegistration registration_;
  cv::VideoCapture cap_;
};
