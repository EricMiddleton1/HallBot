#include "HoughTransform.hpp"

#include <cmath>

HoughTransform::HoughTransform(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"rho", "theta", "threshold", "min_vertical_angle",
      "min_horizontal_angle"}, std::move(params) }
  , rho{std::stof(getParam("rho"))}
  , theta{std::stof(getParam("theta"))*static_cast<float>(CV_PI)/180.f}
  , threshold{std::stof(getParam("threshold"))}
  , minVerticalAngle{std::stof(getParam("min_vertical_angle"))*
      static_cast<float>(CV_PI)/180.f}
  , minHorizontalAngle{std::stof(getParam("min_horizontal_angle"))*
      static_cast<float>(CV_PI)/180.f} {
}

std::vector<cv::Vec2f> HoughTransform::process(const cv::Mat& input) const {
  std::vector<cv::Vec2f> lines;
  
  //cvtColor(edges, hough, COLOR_GRAY2BGR);
	//threshold = 90
  cv::HoughLines(input, lines, rho, theta, threshold, 0, 0 );

  for(int i = 0; i < static_cast<int>(lines.size()); ++i) {
    auto theta = lines[i][1];
    
    //Remove lines that are too horizontal or vertical
    if( (theta < minVerticalAngle) || (theta > CV_PI - minVerticalAngle)
        || (std::abs(theta - CV_PI/2) < minHorizontalAngle) ) {
      lines.erase(lines.begin() + i);
      --i;
    }
  }

  return lines;
}

void HoughTransform::drawLines(const std::vector<cv::Vec2f>& lines, cv::Mat& img) {
  for(auto& line : lines) {
    auto r = line[0], t = line[1];
    float cos_t = std::cos(t), sin_t = std::sin(t);
    float x0 = r*cos_t, y0 = r*sin_t, alpha = 1000.f;

    cv::line(img, {cvRound(x0 - alpha*sin_t), cvRound(y0 + alpha*cos_t)},
      {cvRound(x0 + alpha*sin_t), cvRound(y0 - alpha*cos_t)}, cv::Scalar(255, 0, 0),
      2, cv::LINE_AA);
  }
}
