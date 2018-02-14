#pragma once

#include <string>
#include <vector>
#include <functional>
#include <utility>
#include <exception>

#include <opencv2/imgproc.hpp>

class VideoDevice {
public:
  using Param = std::pair<std::string, std::string>;

  VideoDevice(const std::vector<std::string>& requiredParams,
    const std::vector<Param>& params);

  virtual bool getFrame(cv::Mat& out) = 0;

protected:
  bool paramExists(const std::string& key) const;
  const std::string& getParam(const std::string& key) const;
  std::string getParam(const std::string& key, const std::string& defaultValue) const;

  static cv::Mat resize(const cv::Mat& in, float height);

private:
  int paramIndex(const std::string& key) const;

  std::vector<Param> params_;
};

class ParamMissingException : public std::exception {
public:
  ParamMissingException(const std::string& key);

  const std::string& key() const;

  const char* what() const noexcept override;
private:
  std::string key_;
  std::string msg_;
};
