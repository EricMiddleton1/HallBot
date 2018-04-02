#include "VanishingPointDetector.hpp"

#include <cmath>

VanishingPointDetector::VanishingPointDetector(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"threshold", "iterations"}, std::move(params) }
  , m_enabled{(getParam("enabled", "true") == "true") ? true : false}
  , threshold{std::stoi(getParam("threshold"))}
  , iterations{std::stoi(getParam("iterations"))} {
  
  if(m_enabled) {
    estimator.Initialize(threshold, iterations);
  }
}

bool VanishingPointDetector::enabled() const {
  return m_enabled;
}

float VanishingPointDetector::process(const std::vector<cv::Vec2f>& lines,
  cv::Vec2f& output) {
  
  if(lines.size() < 2) {
    return 0.f;
  }

  std::vector<Line> ransac_lines(lines.size());
  
  for(size_t i = 0; i < lines.size(); ++i) {
    float r = lines[i][0], theta = lines[i][1];
    
    float sin_theta = std::sin(theta), cos_theta = std::cos(theta);
    float x0 = r*cos_theta, y0 = r*sin_theta, alpha = 1000.;
    cv::Vec2f pt1{x0-alpha*sin_theta, y0+alpha*cos_theta},
      pt2{x0+alpha*sin_theta, y0-alpha*cos_theta};

    float a = pt2[1] - pt1[1];
    float b = pt1[0] - pt2[0];
    float c = b*pt1[1] + a*pt1[0];

    ransac_lines[i] = {a, b, c};
  }

  estimator.Estimate(ransac_lines);

  auto* bestModel = estimator.GetBestModel();
  if(bestModel == nullptr) {
    return 0.f;
  }
  else {
    auto intersectionPoint = bestModel->getIntersectionPoint();
    output[0] = intersectionPoint.x;
    output[1] = intersectionPoint.y;

    return 1.f;
  }
}
