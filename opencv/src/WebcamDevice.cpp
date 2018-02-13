#include "WebcamDevice.hpp"

DeviceRegistration WebcamDevice::registration_{{"webcam",
  [](const std::vector<VideoDevice::Param>& params) {
    return std::make_unique<WebcamDevice>(params);
  }}};

WebcamDevice::WebcamDevice(const std::vector<Param>& params)
  : VideoDevice({"id"}, params) {

  cap_.open(std::stoi(getParam("id")));
  if(!cap_.isOpened()) {
    throw std::runtime_error("WebcamDevice: Failed to open webcam " + getParam("id"));
  }
}

bool WebcamDevice::getFrame(cv::Mat& out) {
  cap_ >> out;
  
  return !(out.empty());
}
