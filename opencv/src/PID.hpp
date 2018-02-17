#pragma once

#include "IConfigurable.hpp"

class PID : public IConfigurable{
public:
  PID(std::vector<IConfigurable::Param>&& params);

  void set(float setpoint);
  float update(float curPoint, float dt);
  
private:
  float kp, ki, kd;
  float integral, lastError;
  float setpoint;
};
