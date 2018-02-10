#pragma once

#include <string>
#include <mutex>

#include <boost/asio.hpp>

#include "SerialPort.hpp"

class iRobot {
public:
  struct Position {
    float x, y;
  };

  iRobot(boost::asio::io_service& ioService, const std::string& comPort);

  void start();

  void setWheels(int left, int right);

  Position getPosition() const;
  float getAngle() const;

private:
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

  void recvHandler(std::vector<uint8_t> data);
  bool parseSensorStream(const std::vector<uint8_t>&, int& start);
  bool processSensorUpdate();

  static int16_t parseInt16(const std::vector<uint8_t>& buffer, int start);

  boost::asio::io_service& ioService;
  SerialPort port;

  std::vector<uint8_t> sensorStream;

  Position pos;
  float angle;

  mutable std::mutex mutex;
};
