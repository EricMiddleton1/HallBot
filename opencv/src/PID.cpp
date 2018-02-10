#include "PID.hpp"

PID::PID(float _kp, float _ki, float _kd)
  : kp  {_kp}
  , ki  {_ki}
  , kd  {_kd}
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
