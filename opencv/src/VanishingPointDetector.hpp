#pragma once

#include <opencv2/imgproc.hpp>

#include "IConfigurable.hpp"

#include "GRANSAC.hpp"
#include "IntersectModel.hpp"

class VanishingPointDetector : public IConfigurable {
public:
  VanishingPointDetector(std::vector<IConfigurable::Param>&& params);

  float process(const std::vector<cv::Vec2f>& lines, cv::Vec2f& output);

  bool enabled() const;
private:
  bool m_enabled;
  int threshold, iterations;
  GRANSAC::RANSAC<IntersectModel> estimator;
};
