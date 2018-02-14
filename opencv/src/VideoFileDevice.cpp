#include "VideoFileDevice.hpp"

DeviceRegistration VideoFileDevice::registration_{{"video file",
  [](const std::vector<VideoDevice::Param>& params) {
    return std::make_unique<VideoFileDevice>(params);
  }}};

VideoFileDevice::VideoFileDevice(const std::vector<Param>& params)
  : VideoDevice({"file"}, params) {

  cap_.open(getParam("file"));
  if(!cap_.isOpened()) {
    throw std::runtime_error("VideoFileDevice: Failed to open webcam " + getParam("id"));
  }
}

bool VideoFileDevice::getFrame(cv::Mat& out) {
  cap_ >> out;
  if(out.empty()) {
    cap_.set(cv::CAP_PROP_POS_AVI_RATIO, 0);
    cap_ >> out;
  }

  if(!out.empty()) {
    if(paramExists("max_height")) {
      auto maxHeight = std::stoi(getParam("max_height"));
      if(out.size().height > maxHeight) {
        out = resize(out, maxHeight);
      }
    }
    
    if(paramExists("color_mode") && (getParam("color_mode") == "bw")) {
      cv::Mat gray;
      cv::cvtColor(out, gray, cv::COLOR_RGB2GRAY);
      out = gray;
    }

    return true;
  }
  else {
    return false;
  }
}
