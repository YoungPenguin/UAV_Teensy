#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Tustin_pid.h"

Tustin_PID::Tustin_PID(float* Input, float* Setpoint, float* terms) {
  previous_error = 0.0;
  previous_D = 0.0;
  previous_I = 0.0;

  PID_input = Input;
  PID_setpoint = Setpoint;
  Kp = &terms[0];
  f1 = &terms[1];
  f2 = &terms[2];
  K  = &terms[3];
  K1 = &terms[4];
  K2 = &terms[5];
  K3 = &terms[6];

  AntiWindup = &terms[3];
}

float Tustin_PID::Compute() {
  float error = *Kp * *PID_setpoint - *PID_input;
  float integral = constrain(*f1 * error + previous_error * *f1 + previous_I, -*AntiWindup, *AntiWindup);
  float derivative = integral * *K1 + previous_I * *K2 - *K3 * previous_D;

  return derivative;

  previous_error = error;
  previous_I = integral;
  previous_D = derivative;
}

void Tustin_PID::IntegralReset() {
  integral = 0.0;
}
