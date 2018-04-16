#include "Driver.hpp"

#include <string>
#include <iostream>

Driver::Driver(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"init_speed", "tracking_speed", "retracing_speed"},
      std::move(params) }
  , m_mode{Mode::Initializing}
  , m_running{false}
  , m_pos{{0.f, 0.f}}
  , m_angle{0.f}
  , m_speedInit{std::stoi(getParam("init_speed"))}
  , m_speedTracking{std::stoi(getParam("tracking_speed"))}
  , m_speedRetracing{std::stoi(getParam("retracing_speed"))}
  , m_movementDistance{0.f} {
}

void Driver::setRobot(std::shared_ptr<iRobot>& bot) {
  m_bot = bot;
}

void Driver::start() {
  if(!m_running && m_bot) {
    m_running = true;
    startMode();
  }
}

void Driver::stop() {
  m_running = false;

  if(m_bot) {
    setMotion(0.f, 0.f);
  }
}

bool Driver::running() const {
  return  m_running;
}

Driver::Mode Driver::mode() const {
  return m_mode;
}

void Driver::mode(Mode m) {
  if(m != m_mode) {
    stopMode();
    m_mode = m;
    startMode();
  }
}

void Driver::pose(const cv::Vec2f& pos, float angle) {
  m_pos = pos;
  m_angle = angle;
}

void Driver::update() {
  if(m_running) {
    //Accumulate distance traveled
    m_movementDistance += m_bot->getDistance();

    if(m_mode == Mode::Retracing) {
      std::cout << "[Info] Driver: Distance " << -m_movementDistance << " of total "
        << m_motion.back().distance << " retraced" << std::endl;

      if((-m_movementDistance) >= m_motion.back().distance) {
        std::cout << "[Info] Driver: Retrace complete for movement ("
          << m_motion.back().left << ", " << m_motion.back().right << ")" << std::endl;
        m_motion.pop_back();
        startRetrace();
      }
    }
    else {
      std::cout << "[Info] Driver: Accumulated distance " << m_movementDistance
        << std::endl;
    }
  }
}

void Driver::stopMode() {
  std::cout << "[Info] Driver: Ending mode " << static_cast<int>(m_mode) << std::endl;

  if(m_mode == Mode::Retracing && !m_motion.empty()) {
    float distRemaining = m_motion.back().distance + m_movementDistance;
    if(distRemaining <= 0.f) {
      m_motion.pop_back();
    }
    else {
      m_motion.back().distance = distRemaining;
    }
  }
  else if(!m_motion.empty()) {
    m_motion.back().distance += m_movementDistance;
  }
  m_movementDistance = 0.f;
}

void Driver::startMode() {
  std::cout << "[Info] Driver: Starting mode " << static_cast<int>(m_mode) << std::endl;
  
  switch(m_mode) {
    case Mode::Initializing:
      setMotion(1.f, 1.f);
    break;

    case Mode::Tracking:
      setMotion(1.f, 1.f);
    break;

    case Mode::Retracing:
      startRetrace();
    break;
  }
}

bool Driver::startRetrace() {
  if(m_motion.empty()) {
    stop();

    return false;
  }
  else {
    auto& movement = m_motion.back();

    std::cout << "[Info] Driver: Starting retrace of movement ("
      << movement.left << ", " << movement.right << ")" << std::endl;

    m_movementDistance = 0.f;
    setMotion(movement.left, movement.right);

    return true;
  }
}

void Driver::setMotion(float left, float right) {
  int speed = 0;

  switch(m_mode) {
    case Mode::Initializing:
      speed = m_speedInit;
    break;

    case Mode::Tracking:
      speed = m_speedTracking;
    break;

    case Mode::Retracing:
      speed = -m_speedRetracing;
    break;
  }

  if(m_mode != Mode::Retracing && (left > 0 || right > 0)) {
    if(m_motion.empty() || (m_motion.back().left != left) ||
      (m_motion.back().right != right)) {
      
      if(!m_motion.empty()) {
        m_motion.back().distance += m_movementDistance;
      }

      m_motion.push_back({left, right, 0.f});

      std::cout << "[Info] Starting new motion movement (" << left << ", " << right
        << ")" << std::endl;
    }
  }
  
  std::cout << "[Info] Driver: Setting robot wheels to (" << speed*left << ", "
    << speed*right << ")" << std::endl;

  m_bot->setWheels(speed*left, speed*right);
}
