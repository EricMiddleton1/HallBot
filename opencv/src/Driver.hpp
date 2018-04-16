#pragma once

#include "IConfigurable.hpp"
#include "iRobot.hpp"

#include <vector>
#include <memory>
#include <deque>

#include <opencv2/core/core.hpp>


class Driver : public IConfigurable {
public:
  enum class Mode {
    Initializing,
    Tracking,
    Retracing
  };

  Driver(std::vector<IConfigurable::Param>&& params);

  void setRobot(std::shared_ptr<iRobot>& bot);

  void start();
  void stop();
  bool running() const;

  Mode mode() const;
  void mode(Mode m);

  void pose(const cv::Vec2f& pos, float angle);

  void update();

private:
  struct Movement {
    float left, right;
    float distance;
  };

  void stopMode();
  void startMode();
  bool startRetrace();
  void setMotion(float left, float right);

  std::shared_ptr<iRobot> m_bot;

  Mode m_mode;
  bool m_running;

  cv::Vec2f m_pos;
  float m_angle;

  int m_speedInit, m_speedTracking, m_speedRetracing;

  std::deque<Movement> m_motion;
  float m_movementDistance;
};
