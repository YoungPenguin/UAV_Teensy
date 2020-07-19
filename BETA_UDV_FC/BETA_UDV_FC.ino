/*
   hello there stranger!
   I see you have found the flight controller, i must say you have very good tast.
   This is the main file, here you can input the flightmodes you have implementet
   down in the switch case. Or ofc just use it as is!
   Hope you enjoy it
*/

// Arduino standard library imports
#include <Wire.h>
#include <EEPROM.h>

// Custom imports
#include "MadgwickAHRS.h"
#include "controller.h"
#include "pid.h"
#include "helpers.h"

Madgwick filter;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);

  filter.begin(400); // sample freq for prop shield. 400Hz is max in hybrid mode

  DDRB |= B00100000;
  PORTB &= B11011111;

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;


  all_ready = true;
}

void loop() {
  if (!all_ready) return;

  uint32_t startCycleCPU;
  startCycleCPU = ARM_DWT_CYCCNT;

  switch (flightMode) {
    case DISARMED:
      StopAll();
      PORTB &= B11011111;
      break;
    case RATE_MODE:
      if (THROTTLE > 1000) { // disable stabilization if throttle is very low
        updateMotorsMix(); // Frame specific motor mix
      } else {
        StopAll();
      }
      PORTB |= B00100000;
      break;
    case ACRO_MODE:
      if (THROTTLE > 1000) { // disable stabilization if throttle is very low
        updateMotorsMix(); // Frame specific motor mix
      } else {
        StopAll();
      }
      PORTB |= B00100000;
      break;
    default: // random input go to fail-safe
      StopAll();
      break;
  }

  cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  while (cycles < loop_time) {
    cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  }
}

void StopAll() {
  for (uint8_t i = 0; i < MOTORS; i++) {
    MotorOut[i] = 1000;
  }
  reset_PID_integrals();
}

void reset_PID_integrals() {
  yaw_command_pid.IntegralReset();
  pitch_command_pid.IntegralReset();
  roll_command_pid.IntegralReset();

  yaw_motor_pid.IntegralReset();
  pitch_motor_pid.IntegralReset();
  roll_motor_pid.IntegralReset();

  altitude_hold_baro_pid.IntegralReset();
  altitude_hold_sonar_pid.IntegralReset();
}
