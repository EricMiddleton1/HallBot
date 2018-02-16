#include "RaspicamDevice.hpp"

DeviceRegistration RaspicamDevice::registration_{{"raspicam",
  [](std::vector<VideoDevice::Param>&& std::move(params)) {
    return std::make_unique<RaspicamDevice>(params);
  }}};

RaspicamDevice::RaspicamDevice(std::vector<Param>&& params)
  : VideoDevice({"width", "height"}, std::move(params)) {

  int width = std::stoi(getParam("width")),
    height = std::stoi(getParam("height"));
  int exposure = std::stoi(getParam("exposure", "5")),
    brightness = std::stoi(getParam("brightness", "50")),
    gain = std::stoi(getParam("gain", "100")),
    contrast = std::stoi(getParam("contrast", "75")),
    format = (getParam("color_mode", "rgb") == "bw") ? CV_8UC1 : CV_8UC3;
  
  camera.set(CV_CAP_PROP_FORMAT, format);
  camera.set(CV_CAP_PROP_FRAME_WIDTH, width);
  camera.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  camera.set(CV_CAP_PROP_EXPOSURE, exposure);
  camera.set(CV_CAP_PROP_BRIGHTNESS, brightness);
  camera.set(CV_CAP_PROP_GAIN, gain);
  camera.set(CV_CAP_PROP_CONTRAST, contrast);

  if(!camera.open()) {
    throw std::runtime_error("RaspicamDevice: Failed to open camera");
  }
}

bool RaspicamDevice::getFrame(cv::Mat& out) {
  camera.grab();
  camera.retrieve(out);
  
  return !out.empty();
}
