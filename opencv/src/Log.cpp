#include "Log.hpp"

#include <sstream>
#include <iomanip>

#include <opencv2/opencv.hpp>

Log::Log(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"name"}, std::move(params) }
  , m_name{getParam("name")}
  , m_file{std::string("logs/") + m_name + ".log"}
  , m_counter{0} {

  m_file << "frame, x, y, z, pitch, yaw, roll\n";
}

void Log::add(const VRPN::TrackerData& td) {
  m_td = td;
}

void Log::add(const cv::Mat& frame) {
  m_frame = frame;
}

void Log::write() {
  std::stringstream ss;
  ss << m_name << "_" << std::setw(6) << std::setfill('0') << m_counter
    << ".png";
  
  m_file << ss.str() << ", " << m_td << std::endl;
  cv::imwrite(std::string("logs/") + ss.str(), m_frame);

  ++m_counter;
}
