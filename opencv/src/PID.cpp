#include "PID.hpp"

PID::PID(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"kp", "ki", "kd"}, std::move(params) }
  , kp  {std::stof(getParam("kp"))}
  , ki  {std::stof(getParam("ki"))}
  , kd  {std::stof(getParam("kd"))}
  , integral  {0.f}
  , lastError {0.f}
  , setpoint  {0.f} {

}

void PID::set(float _setpoint) {
  setpoint = _setpoint;
}

float PID::update(float curpoint, float dt) {
  float error = setpoint - curpoint;

  integral += error*dt;

  float actuation = kp*error + ki*integral + kd*(error - lastError)/dt;

  lastError = error;

  return actuation;
}
