#pragma once

#include "opencv2/imgproc.hpp"

#include "IConfigurable.hpp"

class EdgeDetector : public IConfigurable {
public:
  EdgeDetector(std::vector<IConfigurable::Param>&& params);

  cv::Mat process(const cv::Mat& input) const;
private:
  int lowThreshold, highThreshold, kernelSize;
};
