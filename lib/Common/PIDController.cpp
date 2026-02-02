#include "PIDController.h"

// originally from https://github.com/tadelv/steelduino

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0)
{
}

float PIDController::calculate(float setpoint, float process_variable, float dt)
{

  float error = setpoint - process_variable;

  // Proportional term
  float proportional = kp_ * error;

  // Integral term
  integral_ = integral_ + (error * dt);

  // Derivative term
  float derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  // Calculate the control output
  float control_output = (kp_ * error)  + (ki_ * integral_) + (kd_ * derivative);

  return control_output;
}

void PIDController::reset() {
  prev_error_ = 0;
  integral_ = 0;
}

void PIDController::setKp(float kp)
{
  kp_ = kp;
}

void PIDController::setKi(float ki)
{
  ki_ = ki;
}

void PIDController::setKd(float kd)
{
  kd_ = kd;
}

void PIDController::getParams(float *kp, float *ki, float *kd)
{
  *kp = kp_;
  *ki = ki_;
  *kd = kd_;
}
