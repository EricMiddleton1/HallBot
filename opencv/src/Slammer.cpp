#include "Slammer.hpp"

#include <cmath>
#include <iostream>

Slammer::Slammer(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"vocab_file", "config_file"}, std::move(params) }
  , slam{getParam("vocab_file"), getParam("config_file"), ORB_SLAM2::System::MONOCULAR,
      (getParam("visualizer", "false") == "true") ? true : false}
  , startTime{std::chrono::steady_clock::now()} {
}

void Slammer::process(const cv::Mat& input) {
  auto time = std::chrono::steady_clock::now() - startTime;
  double t = std::chrono::duration_cast<std::chrono::microseconds>(time).count()/1000000.;
  
  slam.TrackMonocular(input, t);
}

cv::Mat Slammer::draw() {
  return slam.DrawFrame();
}
