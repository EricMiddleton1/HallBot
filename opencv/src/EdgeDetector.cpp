#include "EdgeDetector.hpp"

EdgeDetector::EdgeDetector(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{{"low_threshold", "high_threshold", "kernel_size"}, std::move(params)}
  , lowThreshold{std::stoi(getParam("low_threshold"))}
  , highThreshold{std::stoi(getParam("high_threshold"))}
  , kernelSize{std::stoi(getParam("kernel_size"))} {
}

cv::Mat EdgeDetector::process(const cv::Mat& input) const {
  cv::Mat output;

  cv::Canny(input, output, lowThreshold, highThreshold, kernelSize);

  return output;
}
