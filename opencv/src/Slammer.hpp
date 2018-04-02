#pragma once

#include <opencv2/imgproc.hpp>

#include "IConfigurable.hpp"

#include <chrono>

//ORB-SLAM2
#include <System.h>

class Slammer : public IConfigurable {
public:
  Slammer(std::vector<IConfigurable::Param>&& params);

  void process(const cv::Mat& input);

  cv::Mat draw();

private:
  ORB_SLAM2::System slam;
  std::chrono::steady_clock::time_point startTime;
};
