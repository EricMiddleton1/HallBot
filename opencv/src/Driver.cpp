#include "Driver.hpp"

#include <string>
#include <iostream>

Driver::Driver(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"init_speed", "tracking_speed", "retracing_speed", "turn_intensity",
      "wall_turn_distance", "zigzag_angle"},
      std::move(params) }
  , m_mode{Mode::Initializing}
  , m_running{false}
  , m_hallwayWidth{0.f}
  , m_hallwayPos{0.f}
  , m_hallwayAngle{0.f}
  , m_moveState{MoveState::Forward}
  , m_desiredAngle{0.f}
  , m_turnDir{0}
  , m_speedInit{std::stoi(getParam("init_speed"))}
  , m_speedTracking{std::stoi(getParam("tracking_speed"))}
  , m_speedRetracing{std::stoi(getParam("retracing_speed"))}
  , m_turnIntensity{std::stof(getParam("turn_intensity"))}
  , m_zigzag_angle{std::stof(getParam("zigzag_angle"))*3.141592654f/180.f}
  , m_wallTurnDistance{std::stof(getParam("wall_turn_distance"))}
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

float Driver::hallwayWidth() const {
  return m_hallwayWidth;
}

void Driver::hallwayWidth(float width) {
  m_hallwayWidth = width;
}

float Driver::posInHallway() const {
  return m_hallwayPos;
}

void Driver::posInHallway(float pos) {
  m_hallwayPos = pos;
}

float Driver::hallwayAngle() const {
  return m_hallwayAngle;
}

void Driver::hallwayAngle(float angle) {
  m_hallwayAngle = angle;
}

void Driver::update() {
  if(m_running) {
    //Accumulate distance traveled
    m_movementDistance += m_bot->getDistance();

        auto distFromLeft = m_hallwayWidth/2.f - m_hallwayPos,
          distFromRight = m_hallwayWidth - distFromLeft;

        std::cout << "[Info] Distance from left, right: " << distFromLeft << ", "
          << distFromRight << std::endl;

    if(m_mode == Mode::Retracing) {
/*      std::cout << "[Info] Driver: Distance " << -m_movementDistance << " of total "
        << m_motion.back().distance << " retraced" << std::endl;
*/
      if((-m_movementDistance) >= m_motion.back().distance) {
        std::cout << "[Info] Driver: Retrace complete for movement ("
          << m_motion.back().left << ", " << m_motion.back().right << ")" << std::endl;
        m_motion.pop_back();
        startRetrace();
      }
    }
    else if(m_mode == Mode::Tracking) {
      if(m_moveState == MoveState::Turn) {
        auto curAngle = m_bot->getAngle();
/*
        std::cout << "[Info] Current angle, desired angle: "
          << curAngle*180.f/3.14159f << ", " << m_desiredAngle*180.f/3.14159f
          << std::endl;
        */
        if((m_turnDir > 0 && curAngle >= m_desiredAngle) ||
            (m_turnDir <= 0 && curAngle <= m_desiredAngle)) {
          stopTurn();
        }
      }
      else if(m_hallwayWidth != 0.f) {
        auto angleInHallway = m_bot->getAngle() - m_hallwayAngle;
        auto distFromLeft = m_hallwayWidth/2.f - m_hallwayPos,
          distFromRight = m_hallwayWidth - distFromLeft;
/*
        std::cout << "[Info] Distance from left, right: " << distFromLeft << ", "
          << distFromRight << std::endl;
					*/
/*
        std::cout << "[Info] Angle in hallway: " << angleInHallway*180.f/3.14159f
          << std::endl;
*/
        if(angleInHallway > 0 && distFromLeft <= m_wallTurnDistance) {
          //Getting too close to left wall, turn right
          startTurn(-m_zigzag_angle);

          std::cout << "[Info] Driver: Too close to left wall, turning right"
            << std::endl;
        }
        else if(angleInHallway <= 0 && distFromRight <= m_wallTurnDistance) {
          //Getting too close to right wall, turn left
          startTurn(m_zigzag_angle);

          std::cout << "[Info] Driver: Too close to right wall, turning left"
            << std::endl;
        }
      }
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

void Driver::startTurn(float angle) {
  m_moveState = MoveState::Turn;
  m_desiredAngle = m_hallwayAngle + angle;
  
  auto delta = m_desiredAngle - m_bot->getAngle();
  m_turnDir = (delta > 0) ? 1 : -1;

  std::cout << "[Info] Driver: Starting turn from angle " << 180.f*m_bot->getAngle()/3.14f
    << " to " << 180.f*m_desiredAngle/3.14f << std::endl;

  setMotion(1.f - m_turnDir*m_turnIntensity, 1.f + m_turnDir*m_turnIntensity);
}

void Driver::stopTurn() {
  std::cout << "[Info] Driver: Stopping turn" << std::endl;

  setMotion(1.f, 1.f);
  m_moveState = MoveState::Forward;
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
