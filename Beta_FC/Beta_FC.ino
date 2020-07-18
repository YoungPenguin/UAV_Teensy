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
#include "controller.h"
#include "PID.h"
#include "SerialCom.h"
#include "StoreData.h"
//#include <BatteryMonitor_current.h>
//#include <Receiver_teensy3_HW_PPM.h>
//#include <FrameType_QuadX.h>

PID yaw_command_pid;
PID pitch_command_pid;
PID roll_command_pid;

PID yaw_motor_pid;
PID pitch_motor_pid;
PID roll_motor_pid;

void StopAll() {
  yaw_command_pid.IntegralReset();
  pitch_command_pid.IntegralReset();
  roll_command_pid.IntegralReset();

  yaw_motor_pid.IntegralReset();
  pitch_motor_pid.IntegralReset();
  roll_motor_pid.IntegralReset();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);

  DDRB |= B00100000;
  PORTB &= B11011111;

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  readEEPROM();

  // Initialize PID objects with data from EEPROM
  yaw_command_pid = PID(&headingError, &YawCommandPIDSpeed, &headingSetpoint, (float*) &CONFIG.data.PID_YAW_c);
  pitch_command_pid = PID(&kinematicsAngle[YAXIS], &PitchCommandPIDSpeed, &commandPitch, (float*) &CONFIG.data.PID_PITCH_c);
  roll_command_pid = PID(&kinematicsAngle[XAXIS], &RollCommandPIDSpeed, &commandRoll, (float*) &CONFIG.data.PID_ROLL_c);

  yaw_motor_pid = PID(&gyro[ZAXIS], &YawMotorSpeed, &YawCommandPIDSpeed, (float*) &CONFIG.data.PID_YAW_m);
  pitch_motor_pid = PID(&gyro[YAXIS], &PitchMotorSpeed, &PitchCommandPIDSpeed, (float*) &CONFIG.data.PID_PITCH_m);
  roll_motor_pid = PID(&gyro[XAXIS], &RollMotorSpeed, &RollCommandPIDSpeed, (float*) &CONFIG.data.PID_ROLL_m);

  initializeESC();
  initializeReceiver();

  sensors.initializeGyro();
  sensors.initializeAccel();

  all_ready = true;
}

void loop() {
  if (!all_ready) return;

  uint32_t startCycleCPU;
  startCycleCPU = ARM_DWT_CYCCNT;

  switch (flightflag) {
    case DISARMED:
      for (uint8_t i = 0; i < MOTORS; i++) {
        MotorOut[i] = CONFIG.data.minimumArmedThrottle;
      }
      updateMotors();
      PORTB &= B11011111;
      break;
    case RATE_MODE:
      updateMotorsMix();
      updateMotors();
      PORTB |= B00100000;
      break;
    case ACRO_MODE:

      PORTB |= B00100000;
      break;
    default: // random input go to fail-safe

      break;
  }

  cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  while (cycles < loop_time) {
    cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  }
}
