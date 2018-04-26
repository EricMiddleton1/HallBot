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

  enum class Side {
    Left,
    Right
  };

  Driver(std::vector<IConfigurable::Param>&& params);

  void setRobot(std::shared_ptr<iRobot>& bot);

  void start();
  void stop();
  bool running() const;

  Mode mode() const;
  void mode(Mode m);

  //Physical Units
  float hallwayWidth() const;
  void hallwayWidth(float width);

  //Physical Units
  float posInHallway() const;
  void posInHallway(float pos);

  //Angle of hallway [-180, 180]
  float hallwayAngle() const;
  void hallwayAngle(float angle);

  void setDistanceToHallwayEnd(float dist);

  void turn(Side side);

  void update();

  void draw(cv::Mat& image, float scale) const;

private:
  enum class MoveState {
    Forward,
    Turn
  };

  struct Movement {
    float left, right;
    float distance;
  };

  void stopMode();
  void startMode();
  void startTurn(float angle); //Angle in hallway reference frame
  void stopTurn();
  bool startRetrace();
  void setMotion(float left, float right);

  std::shared_ptr<iRobot> m_bot;

  Mode m_mode;
  bool m_running;

  float m_hallwayWidth;
  float m_hallwayPos;
  float m_hallwayAngle;
  float m_distToHallwayEnd;
  float m_forcedHallwayWidth;

  MoveState m_moveState;
  float m_desiredAngle;
  int m_turnDir;

  int m_speedInit, m_speedTracking, m_speedRetracing;
  float m_turnIntensity, m_zigzag_angle, m_wallTurnDistance;

  std::deque<Movement> m_motion;
  float m_movementDistance;
	float m_prevLeft, m_prevRight;

  bool m_display;
};
