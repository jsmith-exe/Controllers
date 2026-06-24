#pragma once
#include <Arduino.h>
#include <utility>
#include <math.h>

class INCREMENT_PID 
{
public:
    INCREMENT_PID(float Kp_, float Ki_, float Kd_, float out_min_, float out_max_);
    void begin();
    void calcDt(); 
    void calculateOutput();
    void calculateDeltaOutput();
    float getControlOutput(float setpoint, float measurement);

    float latency_ms;

    float output;
    float prev_output;
    float delta_ouput;

    float Kp;
    float Ki;
    float Kd;
    float integral_reset;
    int out_min;
    int out_max;

private:
  float clampValue(float value);

  float error = 0.0;
  float prev_error = 0.0;
  float prev_prev_error = 0.0;
  bool PREV_PREV_ERROR_FLAG = false;

  float tic_us = 0.0;
  float current_time_us = 0.0;
  float toc_us = 0.0;
  float last_time_us = 0.0;

  float dt;
};