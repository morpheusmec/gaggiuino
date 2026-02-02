#include <Arduino.h>
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// originally from https://github.com/tadelv/steelduino

class PIDController
{
public:
  PIDController(float kp, float ki, float kd);
  float calculate(float setpoint, float process_variable, float dt);
  void reset();

  void setKp(float kp);
  void setKi(float ki);
  void setKd(float kd);
  void getParams(float *kp, float *ki, float *kd);

      private : float kp_; // Proportional gain
  float ki_;         // Integral gain
  float kd_;         // Derivative gain
  float integral_;   // Integral of the error
  float prev_error_; // Previous error
};

#endif // PID_CONTROLLER_H