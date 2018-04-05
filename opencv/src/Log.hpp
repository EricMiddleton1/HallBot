#pragma once

#include "IConfigurable.hpp"

#include "VRPN.hpp"

#include <opencv2/imgproc.hpp>

#include <string>
#include <fstream>

class Log : public IConfigurable {
public:
  Log(std::vector<IConfigurable::Param>&& params);

  void add(const VRPN::TrackerData& td);
  void add(const cv::Mat& frame);
  
  void write();
private:
  std::string m_name;
  std::ofstream m_file;
  int m_counter;

  VRPN::TrackerData m_td;
  cv::Mat m_frame;
};
