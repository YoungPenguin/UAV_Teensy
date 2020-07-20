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
#include <Servo.h>

// Custom imports
#include "controller.h"
#include "Tustin_pid.h"
#include "helpers.h"
#include "MadgwickAHRS.h"

Servo Propeller[MOTORS];

// Flight modes
#define DISARMED      0
#define RATE_MODE     1
#define ACRO_MODE     2
#define ATTITUDE_MODE 3

// Primary channel definitions
#define ROLL        0
#define PITCH       1
#define THROTTLE    2
#define YAW         3

// PID pseudo definitions
#define P  0 // Proportional
#define I  1 // Integral
#define D  2 // Derivative
#define AW 3 // Anti-Windup

volatile int cycles;
uint8_t flightMode = 0;
bool all_ready = false;

// FlightController commands definitions
float commandYaw, commandYawAttitude, commandPitch, commandRoll, commandThrottle;

// Heading related variables
float headingError = 0.0;
float headingSetpoint = 0.0;

// PID variables
float YawCommandPIDSpeed, PitchCommandPIDSpeed, RollCommandPIDSpeed;
float YawMotor;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, heading;

Madgwick filter;

float data[12] = {ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, heading};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);

  for (uint8_t j = 0; j < MOTORS; j++) {
    for (uint8_t i = (0 + 2); i < (MOTORS + 2); i++) {
      Propeller[j].attach(MotorOut[i]);
    }
    Propeller[j].writeMicroseconds(1000);
  }


  DDRB |= B00100000;
  PORTB &= B11011111;

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  filter.begin(400);

  all_ready = true;
}

void loop() {
  if (!all_ready) return;

  uint32_t startCycleCPU;
  startCycleCPU = ARM_DWT_CYCCNT;

  magOn ? filter.update(gx, gy, gz, ax, ay, az, mx, my, mz) : filter.updateIMU(gx, gy, gz, ax, ay, az);

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();



  switch (flightMode) {
    case DISARMED:
      StopAll();
      PORTB &= B11011111;
      break;
    case RATE_MODE:
      if (THROTTLE > 1000) { // disable stabilization if throttle is very low
        // updateMotorsMix(); // Frame specific motor mix
      } else {
        StopAll();
      }
      PORTB |= B00100000;
      break;
    case ACRO_MODE:
      if (THROTTLE > 1000) { // disable stabilization if throttle is very low
        //updateMotorsMix(); // Frame specific motor mix
      } else {
        StopAll();
      }
      PORTB |= B00100000;
      break;
    default: // random input go to fail-safe
      StopAll();
      break;
  }
  //  helpers.dataVector(data);
  cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  while (cycles < loop_time) {
    cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  }
}

void StopAll() {
  for (uint8_t i = 0; i < MOTORS; i++) {
    Propeller[i].writeMicroseconds(1000);
  }
}

void Write2ESC() {
  for (uint8_t i = 0; i < MOTORS; i++) {
    Propeller[i].writeMicroseconds(MotorOut[i]);
  }
}
