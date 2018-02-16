#pragma once

#include <string>
#include <vector>
#include <functional>
#include <utility>
#include <exception>

#include <opencv2/imgproc.hpp>

#include "IConfigurable.hpp"

class VideoDevice : public IConfigurable {
public:
  VideoDevice(const std::vector<std::string>& requiredParams,
    std::vector<Param>&& params);

  virtual bool getFrame(cv::Mat& out) = 0;

protected:
  static cv::Mat resize(const cv::Mat& in, float height);
};
