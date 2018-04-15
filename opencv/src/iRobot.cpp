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
    }}
  , startTime{std::chrono::steady_clock::now()}
  , retraceMovementDone{0} {

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
   
  setWheelsDirect(iLeft, iRight);

  auto curTime = getTime();

  if(state == State::Tracking) {
    if(!motion.empty() && (motion.back().left != iLeft) &&
      (motion.back().right != iRight)) {
      //Mark that previous motion is done
      motion.back().stopTime = curTime;
      
      //Push new motion onto stack
      motion.push_back({left, right, curTime, curTime});

      std::cout << "[Info] iRobot: Ending previous movement and pushing new movement "
        "onto motion stack" << std::endl;
    }
    else if(motion.empty()) {
      std::cout << "[Info] iRobot: Pushing first new movement onto motion stack"
        << std::endl;
      motion.push_back({left, right, curTime, curTime});
    }
  }
}

void iRobot::setWheelsDirect(int16_t left, int16_t right) {
  std::cout << "[Info] iRobot: setting wheels to (" << left << ", " << right << ")"
    << std::endl;
  
  port.send({static_cast<uint8_t>(Command::DriveDirect),
    static_cast<uint8_t>(right >> 8), static_cast<uint8_t>(right & 0xFF),
    static_cast<uint8_t>(left >> 8), static_cast<uint8_t>(left & 0xFF)});
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
	return cameraScale;
}

void iRobot::setCameraPose(const cv::Vec2f& p, float a) {
  static cv::Vec2f lastCameraPos = {0.f, 0.f};

  std::lock_guard<std::mutex> lock(mutex);

  if(port.connected()) {
    //Compute/update camera scale factor
    float cameraDist = dist(lastCameraPos, p);
    if(cameraDist != 0.f) {
      float newCamScale = distAccum / cameraDist;
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

iRobot::State iRobot::getState() const {
  return state;
}

void iRobot::setState(State s) {
  auto oldState = state;
  state = s;

  if(state != oldState) {
    if(state == State::Retracing) {
      //End current movement
      setWheels(0, 0);
      if(!motion.empty()) {
        motion.back().stopTime = getTime();
      }
    }
    else if(state == State::Tracking) {
      if(!motion.empty() && retraceMovementDone != 0) {
        auto dt = static_cast<int64_t>(retraceMovementDone) - getTime();
        
        if(dt > 0) {
          motion.back().stopTime = motion.back().startTime + dt;
        }
        else {
          motion.pop_back();
        }
      }
      retraceMovementDone = 0;
    }
  }
}

bool iRobot::retraceStep() {
  if(state != State::Retracing || motion.empty()) {
    if(motion.empty()) {
      std::cout << "[Error] iRobot: Retracing stack empty" << std::endl;
    }
    return false;
  }

  auto curTime = getTime();
 
  if(retraceMovementDone == 0) {
    retraceMovementDone = curTime + (motion.back().stopTime - motion.back().startTime);
    setWheels(-motion.back().left, -motion.back().right);

    std::cout << "[Info] Retracing movement (" << motion.back().left << ", "
      << motion.back().right << ") for " << retraceMovementDone-curTime << "ms"
      << std::endl;
  }
  else if(curTime >= retraceMovementDone) {
    std::cout << "[Info] Retracing movement done" << std::endl;

    motion.pop_back();
    
    if(motion.empty()) {
      setWheelsDirect(0, 0);
      std::cout << "[Error] iRobot: Retracing stack empty" << std::endl;
      return false;
    }
    else {
      retraceMovementDone = curTime + (motion.back().stopTime - motion.back().startTime);
      setWheels(-motion.back().left, -motion.back().right);
      std::cout << "[Info] Retracing movement (" << motion.back().left << ", "
        << motion.back().right << ") for " << retraceMovementDone-curTime << "ms"
        << std::endl;

    }
  }

  return true;
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

  auto dist = parseInt16(sensorStream, 3) / 1000.f;
  auto ang_change = parseInt16(sensorStream, 6);

  {
    std::lock_guard<std::mutex> lock(mutex);

    distAccum += dist;

    angle += ang_change * M_PI / 180.;
    pos[0] += dist * std::cos(angle);
    pos[1] += dist * std::sin(angle);
  }
  return true;
}

uint32_t iRobot::getTime() const {
  auto time = std::chrono::steady_clock::now() - startTime;
  return std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
}

int16_t iRobot::parseInt16(const std::vector<uint8_t>& buffer, int index) {
  return (buffer[index] << 8) | buffer[index + 1];
}

float iRobot::dist(const cv::Vec2f& p1, const cv::Vec2f& p2) {
  float dx = p1[0]-p2[0], dy = p1[1]-p2[1];
  return std::sqrt(dx*dx + dy*dy);
}
