#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "pid.h"

PID::PID(float* Input, float* Output, float* Setpoint, float* terms) {
  previous_error = 0.0;
  integral = 0.0;

  PID_input = Input;
  PID_output = Output;
  PID_setpoint = Setpoint;

  Kp = &terms[0];
  Ki = &terms[1];
  Kd = &terms[2];

  AntiWindup = &terms[3];
}

void PID::Compute() {
  unsigned long now = micros();
  float delta_time = (now - last_time) / 1000000.0;
  float error = *PID_setpoint - *PID_input;
  integral = constrain(integral + error * delta_time, -*AntiWindup, *AntiWindup);
  float derivative = (*PID_input - previous_error) / delta_time;

  *PID_output = *Kp * error + *Ki * integral + *Kd * derivative;

  previous_error = *PID_input;
  last_time = now;
}

void PID::IntegralReset() {
  integral = 0.0;
}
