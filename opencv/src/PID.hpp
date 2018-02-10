#pragma once

class PID {
public:
  PID(float kp, float ki, float kd);

  void set(float setpoint);
  float update(float curPoint, float dt);
  
private:
  float kp, ki, kd;
  float integral, lastError;
  float setpoint;
};
