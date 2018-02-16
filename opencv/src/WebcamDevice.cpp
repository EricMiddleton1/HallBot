#include "WebcamDevice.hpp"

DeviceRegistration WebcamDevice::registration_{{"webcam",
  [](std::vector<VideoDevice::Param>&& params) {
    return std::make_unique<WebcamDevice>(std::move(params));
  }}};

WebcamDevice::WebcamDevice(std::vector<Param>&& params)
  : VideoDevice{{"id"}, std::move(params)} {

  cap_.open(std::stoi(getParam("id")));
  if(!cap_.isOpened()) {
    throw std::runtime_error("WebcamDevice: Failed to open webcam " + getParam("id"));
  }
}

bool WebcamDevice::getFrame(cv::Mat& out) {
  cap_ >> out;
  
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
