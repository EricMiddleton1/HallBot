#pragma once

#include <opencv2/imgproc.hpp>

#include "IConfigurable.hpp"

class HoughTransform : public IConfigurable {
public:
  HoughTransform(std::vector<IConfigurable::Param>&& params);

  std::vector<cv::Vec2f> process(const cv::Mat& input) const;

  static void drawLines(const std::vector<cv::Vec2f>& lines, cv::Mat& img);
private:
  float rho, theta, threshold, minVerticalAngle, minHorizontalAngle;
};
