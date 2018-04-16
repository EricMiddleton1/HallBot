#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <deque>

#include <opencv2/core/core.hpp>

#include <boost/asio.hpp>

#include "SerialPort.hpp"
#include "PeriodicTimer.hpp"

#include "IConfigurable.hpp"

class iRobot : public IConfigurable {
public:
  enum class State {
    Initializing = 0,
    Tracking,
    Retracing
  };

  iRobot(std::vector<IConfigurable::Param>&& params);
  ~iRobot();

  void setWheels(float left, float right);

  cv::Vec2f getPosition() const;
  float getAngle() const;

  void setCameraPose(const cv::Vec2f&, float angle);
  void setCameraAngle(float angle);

  float getCameraScale() const;

  void setState(State);
  State getState() const;

  bool getButtonPress() const;

  bool retraceStep();

private:
  const int SENSOR_UPDATE_RATE = 100; //milliseconds
  
  struct Movement {
    float left, right;
    uint32_t startTime, stopTime;
  };

  enum class Command {
    Start = 128,
    FullMode = 132,
    Drive = 137,
    DriveDirect = 145,
    LEDs = 139,
    Sensors = 142,
  };

  struct Sensors {
    uint8_t ir;

    union {
      struct {
        uint8_t play : 1;
        uint8_t unused1 : 1;
        uint8_t advance : 1;
        uint8_t unused0 : 5;
      } button;
      uint8_t button_packed;
    };

    int16_t distance;
    int16_t angle;
  };

  void start();
  void recvHandler(std::vector<uint8_t> data);
  void sensorUpdate();
  bool parseSensorStream(Sensors& sensors);
  void processSensorUpdate(const Sensors& sensors);
  void setWheelsDirect(int16_t left, int16_t right);
  uint32_t getTime() const;

  static int16_t parseInt16(const std::vector<uint8_t>& buffer, int start);

  static float dist(const cv::Vec2f& p1, const cv::Vec2f& p2);

  boost::asio::io_service ioService;
  std::unique_ptr<boost::asio::io_service::work> ioWork;
  SerialPort port;

  std::vector<uint8_t> sensorStream;

  cv::Vec2f pos;
  float angle;
  float distAccum, cameraScale;

  bool buttonState;
  mutable bool buttonPress;

  State state;
  int drivingSpeeds[3];
  
  std::deque<Movement> motion;
  std::chrono::steady_clock::time_point startTime;
  uint32_t retraceMovementDone;

  mutable std::mutex mutex;

  PeriodicTimer sensorTimer;

  std::thread asyncThread;
};
