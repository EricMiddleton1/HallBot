#include "iRobot.hpp"

#include <iostream>
#include <numeric>
#include <cmath>

iRobot::iRobot(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"port"},
      std::move(params) }
  , ioWork{std::make_unique<boost::asio::io_service::work>(ioService)}
  , port{ioService, getParam("port"), 57600, [this](std::vector<uint8_t> data) {
      recvHandler(data);
    }}
  , drivenDistance{0.f}
  , pos{0.f, 0.f}
  , angle{0.f}
  , distAccum{0.f}
  , cameraScale{1.f}
  , buttonState{false}
  , buttonPress{false}
  , sensorTimer{ioService, std::chrono::milliseconds(SENSOR_UPDATE_RATE), [this]() {
      sensorUpdate();
    }}
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
  port.send({static_cast<uint8_t>(Command::Start),
    static_cast<uint8_t>(Command::FullMode)});
}

bool iRobot::getButtonPress() const {
  if(!port.connected()) {
    return true;
  }
  std::lock_guard<std::mutex> lock(mutex);
  
  bool pressCopy = buttonPress;
  buttonPress = false;

  return pressCopy;
}

void iRobot::setWheels(int left, int right) {
  left = std::min(500, std::max(-500, left));
  right = std::min(500, std::max(-500, right));
  
  port.send({static_cast<uint8_t>(Command::DriveDirect),
    static_cast<uint8_t>(right >> 8), static_cast<uint8_t>(right & 0xFF),
    static_cast<uint8_t>(left >> 8), static_cast<uint8_t>(left & 0xFF)});
}

float iRobot::getDistance() const {
  std::unique_lock<std::mutex> lock(mutex);

  float distCopy = drivenDistance;
  drivenDistance = 0.f;

  return distCopy;
}

cv::Vec2f iRobot::getPosition() const {
  std::lock_guard<std::mutex> lock(mutex);
  return pos;
}

float iRobot::getAngle() const {
  std::lock_guard<std::mutex> lock(mutex);
  return angle;
}

float iRobot::getCameraScale() const {
  std::lock_guard<std::mutex> lock(mutex);
	return cameraScale;
}

void iRobot::setCameraPose(const cv::Vec2f& p, float a) {
  static cv::Vec2f lastCameraPos = {0.f, 0.f};

  std::lock_guard<std::mutex> lock(mutex);

  if(port.connected()) {
    //Compute/update camera scale factor
    float cameraDist = dist(lastCameraPos, p);
    if(cameraDist != 0.f) {
      //Scale to convert from ORB-SLAM units to meters
      float newCamScale = distAccum / cameraDist / 1000.f;
      float alpha = (cameraScale == 0.f) ? 1.f : 0.1f;
      cameraScale = alpha*newCamScale + (1.f-alpha)*cameraScale;
    }
    distAccum = 0.f;
  }
  else {
    //Assume scale=1 when no encoder data is available
    cameraScale = 1.f;
  }

  pos = cameraScale * p;
  angle = a;
  lastCameraPos = p;
}

void iRobot::recvHandler(std::vector<uint8_t> data) {
  sensorStream.insert(sensorStream.end(), data.begin(), data.end());
  
  Sensors sensors;
  while(parseSensorStream(sensors)) {
    processSensorUpdate(sensors);
  }
}

void iRobot::sensorUpdate() {
  //Flush receive buffer
  sensorStream.clear();

  //Request sensor packet for group 2 (values 17-20)
  port.send({static_cast<uint8_t>(Command::Sensors), 2});
}


bool iRobot::parseSensorStream(Sensors& sensors) {
  //Sensor group 2: 6 bytes
  const int PACKET_SIZE = 6;
  if(sensorStream.size() < PACKET_SIZE) {
    return false;
  }
  else {
    sensors.ir = sensorStream[0];
    sensors.button_packed = sensorStream[1];
    sensors.distance = parseInt16(sensorStream, 2);
    sensors.angle = parseInt16(sensorStream, 4);

    sensorStream.erase(sensorStream.begin(), sensorStream.begin() + PACKET_SIZE);

    return true;
  }
}

void iRobot::processSensorUpdate(const Sensors& sensors) {
  {
    std::lock_guard<std::mutex> lock(mutex);

    distAccum += sensors.distance;
    drivenDistance += sensors.distance;

    angle += sensors.angle * M_PI / 180.;
    pos[0] += sensors.distance * std::cos(angle);
    pos[1] += sensors.distance * std::sin(angle);

    if(sensors.button.play != buttonState) {
      buttonState = sensors.button.play;
      if(buttonState) {
        buttonPress = true;
      }
    }
  }
/*
  std::cout << "[Info] iRobot Sensors: buttons="
    << static_cast<int>(sensors.button_packed) << "button_advance="
    << static_cast<int>(sensors.button.advance) << ", button_play="
    << static_cast<int>(sensors.button.play) << ", distance="
    << sensors.distance << ", angle=" << sensors.angle << std::endl;
    */
}

int16_t iRobot::parseInt16(const std::vector<uint8_t>& buffer, int index) {
  return (buffer[index] << 8) | buffer[index + 1];
}

float iRobot::dist(const cv::Vec2f& p1, const cv::Vec2f& p2) {
  float dx = p1[0]-p2[0], dy = p1[1]-p2[1];
  return std::sqrt(dx*dx + dy*dy);
}
