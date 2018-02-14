#include "VideoDevice.hpp"


VideoDevice::VideoDevice(const std::vector<std::string>& requiredParams,
  const std::vector<Param>& params)
  : params_(params) {

  for(const auto& param : requiredParams) {
    if(!paramExists(param)) {
      throw ParamMissingException(param);
    }
  }
}

bool VideoDevice::paramExists(const std::string& key) const {
  return paramIndex(key) != -1;
}

const std::string& VideoDevice::getParam(const std::string& key) const {
  auto index = paramIndex(key);
  if(index == -1) {
    throw ParamMissingException(key);
  }
  else {
    return params_[index].second;
  }
}

std::string VideoDevice::getParam(const std::string& key,
  const std::string& defaultValue) const {
  auto index = paramIndex(key);
  if(index == -1) {
    return defaultValue;
  }
  else {
    return params_[index].second;
  }
}

int VideoDevice::paramIndex(const std::string& key) const {
  auto found = std::find_if(params_.begin(), params_.end(),
    [&key](const auto& param) {
      return param.first == key;
    });

  if(found == params_.end()) {
    return -1;
  }
  else {
    return found - params_.begin();
  }
}

cv::Mat VideoDevice::resize(const cv::Mat& in, float height) {
  cv::Mat out;

  auto inSize = in.size();
  float aspectRatio = static_cast<float>(inSize.width) / inSize.height;
  cv::Size newSize(height*aspectRatio, height);

  cv::resize(in, out, newSize, 0, 0, cv::INTER_CUBIC);

  return out;
}

ParamMissingException::ParamMissingException(const std::string& key)
  : key_{key}
  , msg_{std::string("Missing Parameter '") + key + "'"} {
}

const std::string& ParamMissingException::key() const {
  return key_;
}

const char* ParamMissingException::what() const noexcept {
  return msg_.c_str();
}
