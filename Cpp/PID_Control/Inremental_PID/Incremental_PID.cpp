#include "Incremental_PID.h"

INCREMENT_PID::INCREMENT_PID(float Kp, float Ki, float Kd, float integral_reset, float out_min, float out_max)
  : Kp(Kp), Ki(Ki), Kd(Kd),
    integral(0.0f),
    prev_meas(0.0f),
    last_us(micros()),
    integral_reset(integral_reset),
    out_min(out_min), out_max(out_max),
{
}

void INCREMENT_PID::begin()
{
}

float INCREMENT_PID::clampValue(float value, float min_clamp, float max_clamp) 
{
  if (value < min_clamp) return min_clamp;
  if (value > max_clamp) return max_clamp;

  return value;
}

void INCREMENT_PID::reset() 
{
  integral = 0.0f;
  prev_meas = 0.0f;
  prev_P = 0.0f;
  prev_I = 0.0f;
  prev_D = 0.0f;
  last_us = micros();
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
    prev_meas = measurement;
  } 
}

void INCREMENT_PID::checkIntegralReset()
{
  if (integral_reset >= 0.0f && fabsf(error) <= integral_reset) {
    integral = 0.0f;
  }
}

void INCREMENT_PID::calculateOutput()
{
  float output_calc = prev_output + delta_ouput;
  output = clampValue(output_calc, out_min, out_max);
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

  prev_error = error;
  if (PREV_PREV_ERROR_FLAG)
  {
    prev_prev_error = prev_error;
  }
  PREV_PREV_ERROR_FLAG = true;

  calculateOutput();
  prev_output = output;

  last_time_us = current_time_us;
  toc_us = micros();
  latency_ms = (toc_us - tic_us) * 1e-3f;

  return output;
}



