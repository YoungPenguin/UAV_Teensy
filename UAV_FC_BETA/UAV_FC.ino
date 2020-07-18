/*
   hello there stranger!
   I see you have found the flight controller, i must say you have very good tast.
   This is the main file, here you can input the flightmodes you have implementet
   down in the switch case. Or ofc just use it as is!
   Hope you enjoy it
*/

#include <Servo.h>
//#include <NXPMotionSense.h>
//#include <MadgwickAHRS.h>
#include <Wire.h>
#include <EEPROM.h>

#include "helpers.h"


//NXPMotionSense imu;
//Madgwick filter; // using the madgwick algo - change to mahony if you have problems with drift

#define loop_time 450000 // 2.5ms @ 180Mhz 
#define pinCount 6

Servo Propeller[pinCount];

volatile int cycles;
volatile int flightflag = 0;

float f1[3], f2[3], K1[3], K2[3], K3[3];

helpers helper;

float alpha[3] = {0.0, 0.0, 0.00};
float tau_D[3] = {0.0, 0.0, 0.0};
float tau_I[3] = {0.0, 0.0, 0.0};
float kp[3] = {0.0, 0.0, 0.0};
float T = 0.0025;
float K[3] = {0, 0, 0}; // calculate this value from the pre-warping ... or if you belive in GOD the use 2/T



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(1);
  
  DDRB |= B00100000;
  PORTB &= B11011111;

  for (int thisProp = 0; thisProp < pinCount; thisProp++) {
    Propeller[thisProp].writeMicroseconds(1000);
  }
  //  imu.begin();
  //  filter.begin(400); // sample freq for prop shield. 400Hz is max in hybrid mode

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  /*Calculate tustin values so you dont have to*/
  for (int i = 0; i < 3; i++) {
    f1[i] = (tau_I[i] * K[i] + 1 ) / (tau_I[i] * K[i]);
    f2[i] = (1 - tau_I[i] * K[i]) / (tau_I[i] * K[i]);
    K1[i] = (tau_D[i] * K[i] + 1) / (1 + alpha[i] * tau_D[i] * K[i]);
    K2[i] = (1 - tau_D[i] * K[i]) / (1 + alpha[i] * tau_D[i] * K[i]);
    K3[i] = (1 - alpha[i] * tau_D[i] * K[i]) / (1 + alpha[i] * tau_D[i] * K[i]);
  }
  /*Calculate tustin values*/

  //  while (!(imu.available())); // to make sure the hardware it rdy 2 go

}

void loop() {
  uint32_t startCycleCPU;
  startCycleCPU = ARM_DWT_CYCCNT;
  // put your main code here, to run repeatedly:



  switch (flightflag) {
    case 0: // dis-armed (default)
      // flightMode0();
      PORTB &= B11011111;
      break;
    case 1: // Armed
      //flightMode1();
      PORTB |= B00100000;
      break;
    case 2: // better safe the sorry
      // failsafe();
      break;
    default: // random input go to fail-safe
      //failsafe();
      break;
  }

  cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  while (cycles < loop_time) {
    cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  }
}
