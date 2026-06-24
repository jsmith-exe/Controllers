#pragma once
#include <Arduino.h>
#include <utility>
#include <math.h>

class INCREMENT_PID 
{
public:
    INCREMENT_PID(float Kp, float Ki, float Kd, float integral_reset, float out_min, float out_max);
    void begin();
    void reset();
    void calcDt(); 
    void checkIntegralReset();
    void calculateOutput();
    void calculateDeltaOutput();
    float getControlOutput(float setpoint, float measurement);

    float latency_ms;

    float output;
    float prev_output;
    float delta_ouput;

private:
  float clampValue(float value, float min_clamp, float max_clamp);

  float prev_meas;
  float last_time_us;

  float error = 0.0;
  float prev_error = 0.0;
  float prev_prev_error = 0.0;
  bool PREV_PREV_ERROR_FLAG = false;

  float prev_output = 0.0;

  float tic_us = 0.0;
  float current_time_us = 0.0;
  float toc_us = 0.0;
  float last_time_us = 0.0;

  float integral_reset;
  float out_min, out_max;

  float dt;
};