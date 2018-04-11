#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <deque>

#include <opencv2/core/core.hpp>

#include <boost/asio.hpp>

#include "SerialPort.hpp"

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

  bool retraceStep();

private:
  struct Movement {
    int16_t left, right;
    uint32_t startTime, stopTime;
  };

  enum class Command {
    Start = 128,
    Baud = 129,

    SafeMode = 131,
    FullMode = 132,

    Drive = 137,
    DriveDirect = 145,

    LEDs = 139,

    Sensors = 142,
    SensorStream = 148
  };
  enum class SensorPacket {
    BumpsAndDrops = 7,
    Wall = 8,
    CliffLeft = 9,
    CliffFrontLeft = 10,
    CliffFrontRight = 11,
    CliffRight = 12,
    VirtualWall = 13,
    Overcurrents = 14,
    IRByte = 16,
    Buttons = 18,
    Distance = 19,
    Angle = 20,
    ChargingState = 21,
    Voltage = 22,
    Current = 23,
    BatteryTemperature = 24,
    BatteryCharge = 25,
    BatteryCapacity = 26,
    WallSignal = 27,
    CliffLeftSignal = 28,
    CliffFrontLeftSignal = 29,
    CliffFrontRightSigna = 30,
    CliffRightSignal = 31,
    UserDigitalInputs = 32,
    UserAnalogInput = 33,
    ChargingSourcesAvailable = 34,
    OIMode = 35,
    SongNumber = 36,
    SongPlaying = 37,
    NumberOfStreamPackets = 38,
    Velocity = 39,
    Radius = 40,
    RightVelocity = 41,
    LeftVelocity = 42
  };
  const uint8_t SensorStreamStart = 19;

  void start();
  void recvHandler(std::vector<uint8_t> data);
  bool parseSensorStream(const std::vector<uint8_t>&, int& start);
  bool processSensorUpdate();
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

  State state;
  int drivingSpeeds[3];
  
  std::deque<Movement> motion;
  std::chrono::steady_clock::time_point startTime;
  uint32_t retraceMovementDone;

  mutable std::mutex mutex;

  std::thread asyncThread;
};
