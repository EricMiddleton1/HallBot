#include "iRobot.hpp"

#include <iostream>
#include <numeric>
#include <cmath>

iRobot::iRobot(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"port", "initial_speed", "tracking_speed", "retracing_speed"},
      std::move(params) }
  , ioWork{std::make_unique<boost::asio::io_service::work>(ioService)}
  , port{ioService, getParam("port"), 57600, [this](std::vector<uint8_t> data) {
      recvHandler(data);
    }}
  , pos{0.f, 0.f}
  , angle{0.f}
  , state{State::Initializing}
  , drivingSpeeds{std::stoi(getParam("initial_speed")),
      std::stoi(getParam("tracking_speed")), std::stoi(getParam("retracing_speed"))}
  , asyncThread{ [this](){
      ioService.run();
      std::cout << "[Error] SerialPort: Thread close" << std::endl;
    }} {

  start();
}

iRobot::~iRobot() {
  ioWork.reset();
  asyncThread.join();
}

void iRobot::start() {
  //Start OpenInterface, set mode to full
  //Request sensor stream (Distance, Angle)
  port.send({static_cast<uint8_t>(Command::Start),
    static_cast<uint8_t>(Command::FullMode),
    static_cast<uint8_t>(Command::SensorStream), 2,
    static_cast<uint8_t>(SensorPacket::Distance),
    static_cast<uint8_t>(SensorPacket::Angle)});
}

void iRobot::setWheels(float left, float right) {
  auto speed = drivingSpeeds[static_cast<int>(state)];
  int iLeft = speed*left, iRight = speed*right;

  iLeft = std::min(500, std::max(-500, iLeft));
  iRight = std::min(500, std::max(-500, iRight));

  port.send({static_cast<uint8_t>(Command::DriveDirect),
    static_cast<uint8_t>(iRight >> 8), static_cast<uint8_t>(iRight & 0xFF),
    static_cast<uint8_t>(iLeft >> 8), static_cast<uint8_t>(iLeft & 0xFF)});
}

iRobot::Position iRobot::getPosition() const {
  std::lock_guard<std::mutex> lock(mutex);
  return pos;
}

float iRobot::getAngle() const {
  std::lock_guard<std::mutex> lock(mutex);
  return angle;
}

iRobot::State iRobot::getState() const {
  return state;
}

void iRobot::setState(State s) {
  state = s;
}

void iRobot::recvHandler(std::vector<uint8_t> data) {
  int start = 0;
  while(parseSensorStream(data, start)) {
    sensorStream.clear();
  }
}

bool iRobot::parseSensorStream(const std::vector<uint8_t>& data, int& start) {
  if(sensorStream.empty()) {
    for(; (start < static_cast<int>(data.size())) && data[start] != SensorStreamStart;
      ++start) {
      std::cout << "[Warning] iRobot: Rejecting byte '" << static_cast<int>(data[start])
        << "'" << std::endl;
    }
  }
  
  int available = data.size() - start;
  if(available <= 0) {
    return false;
  }

  if(sensorStream.size() < 2) {
    int needed = 2 - sensorStream.size();
    if(available <= needed) {
      sensorStream.insert(sensorStream.end(), data.begin() + start, data.end());
      return false;
    }
    else {
      sensorStream.insert(sensorStream.end(), data.begin() + start,
        data.begin() + start + needed);
      start += needed;
      available -= needed;
    }
  }
  
  auto packetSize = sensorStream[1] + 3;
  int needed = packetSize - sensorStream.size(),
    toRead = std::min(needed, available);

  sensorStream.insert(sensorStream.end(), data.begin() + start,
    data.begin() + start + toRead);
  start += toRead;

  return toRead == needed;
}

bool iRobot::processSensorUpdate() {
  if(sensorStream.size() != 9) {
    std::cerr << "[Error] Invalid sensor stream size (expected 9, actually "
      << sensorStream.size() << ")" << std::endl;
    return false;
  }

  auto checksum = std::accumulate(sensorStream.begin(), sensorStream.end(),
    static_cast<uint8_t>(0));

  if(checksum != 0) {
    std::cerr << "[Error] Invalid checksum value (" << checksum << ")"
      << std::endl;
  }

  auto dist = parseInt16(sensorStream, 3);
  auto ang_change = parseInt16(sensorStream, 6);

  {
    std::lock_guard<std::mutex> lock(mutex);

    angle += ang_change * M_PI / 180.;
    pos.x += dist * std::cos(angle);
    pos.y += dist * std::sin(angle);
  }
  return true;
}

int16_t iRobot::parseInt16(const std::vector<uint8_t>& buffer, int index) {
  return (buffer[index] << 8) | buffer[index + 1];
}
