#include "VideoDevice.hpp"


VideoDevice::VideoDevice(const std::vector<std::string>& requiredParams,
  std::vector<Param>&& params)
  : IConfigurable{requiredParams, std::move(params)} {
}

cv::Mat VideoDevice::resize(const cv::Mat& in, float height) {
  cv::Mat out;

  auto inSize = in.size();
  float aspectRatio = static_cast<float>(inSize.width) / inSize.height;
  cv::Size newSize(height*aspectRatio, height);

  cv::resize(in, out, newSize, 0, 0, cv::INTER_CUBIC);

  return out;
}
