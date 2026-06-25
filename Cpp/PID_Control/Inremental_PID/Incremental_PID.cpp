#include "Incremental_PID.h"

INCREMENT_PID::INCREMENT_PID(float Kp_, float Ki_, float Kd_, float out_min_, float out_max_)
  : Kp(Kp_), Ki(Ki_), Kd(Kd_),
    out_min(out_min_), out_max(out_max_)
{
}

void INCREMENT_PID::begin()
{
  last_time_us = micros();
}

void INCREMENT_PID::setTunings(float Kp_, float Ki_, float Kd_)
{
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void INCREMENT_PID::resetOutput(float value)
{
  output = value;
  prev_output = value;
  delta_ouput = 0.0f;

  error = 0.0f;
  prev_error = 0.0f;
  prev_prev_error = 0.0f;
}

float INCREMENT_PID::clampValue(float value) 
{
  if (value < out_min) return out_min;
  if (value > out_max) return out_max;

  return value;
}

void INCREMENT_PID::calcDt()
{
  tic_us = micros();
  current_time_us = micros();
  const float dt_us = (current_time_us - last_time_us);
  dt = (float)dt_us * 1e-6f;

  // Protect against zero / tiny dt
  if (dt <= 1e-9f) 
  {
    const float toc_us = micros();
    latency_ms = (float)((uint32_t)(toc_us - tic_us)) * 1e-3f;
    last_time_us = current_time_us;
  } 
}

void INCREMENT_PID::calculateOutput()
{
  float output_calc = prev_output + delta_ouput;
  output = clampValue(output_calc);
}

void INCREMENT_PID::calculateDeltaOutput()
{
  const float P = Kp * (error - prev_error);
  const float I = Ki * error * dt;
  const float D = Kd * ( ( error - (2*prev_error) + prev_prev_error ) / (dt) );
  delta_ouput = P + I + D;
}

float INCREMENT_PID::getControlOutput(float setpoint, float measurement) 
{
  calcDt();

  error = setpoint - measurement;

  calculateDeltaOutput();
  calculateOutput();

  prev_prev_error = prev_error;
  prev_error = error;
  prev_output = output;

  last_time_us = current_time_us;

  toc_us = micros();
  latency_ms = (toc_us - tic_us) * 1e-3f;

  return output;
}



