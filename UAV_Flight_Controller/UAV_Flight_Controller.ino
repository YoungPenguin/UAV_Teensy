/*
   hello there stranger!
   I see you have found the flight controller, i must say you have very good tast.
   This is the main file, here you can input the flightmodes you have implementet
   down in the switch case. Or ofc just use it as is!
   Hope you enjoy it
*/

#include <Servo.h>
#include <NXPMotionSense.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <EEPROM.h>
// #include "SparkFunMPL3115A2.h"

NXPMotionSense imu;
Madgwick filter; // using the madgwick algo - change to mahony if you have problems with drift

/* functions */
void Transmitter();
float Yaw_counter(float yaw_difference);
void PC_input();
void Dof3PID();
int flightmodes();

void MotorMixHex();
// void MotorMixQuad();

/* Add differet fligth modes */
void flightMode0(); // dis-armed
void flightMode1(); // armed
void failsafe(); // fligthmode 2 = failsafe

/******************** Values to edit ************************/
#define loop_time 450000 //45000=2.5ms @ 180Mhz // 36000=2ms
#define pinCount 6
// total number of motors
Servo Propeller[pinCount];
float pwm_[pinCount] = {0.0, 0.0, 0.0};

bool mag = true; // dafault state for magnetometer On/Off = true/false
bool dataOn = true; // dafault state for data On/Off = true/false

/* [roll, pitch, yaw] */
float alpha[3] = {0.02, 0.02, 0.02};
float tau_D[3] = {0.083, 0.083, 0.083};
float tau_I[3] = {1, 1, 1};
float kp[3] = {1.8, 1.8, 1.8};
float T = 0.0025;
float K[3] = {800, 800, 800};


/************************/
float f1[3], f2[3], K1[3], K2[3], K3[3];

unsigned long counter[6];
byte last_CH_state[5];
int input_pin[5];


/******************* inizilazation ******************/
float PID_output[3];
float error[3];
float old_error[3];
float Iterm[3];
float Dterm[3];
float old_I[3];
float old_D[3];

int throttle = 0;
int flightflag = 0;

/*Controller inputs*/
float desired_angle[3]      = {0.0, 0.0, 0.0};
float yaw_desired_angle_set = 0.0;
float total_yaw             = 0.0;

/*PC input with serial check PC_input for the commands*/
int Serial_input[4] = {0, 0, 0, 0};
int data_flag = 0;
float roll, pitch, yaw, yaw_previous, yaw_difference, last_yaw;
float roll_val, pitch_val, yaw_val;
float old_roll, old_pitch, old_yaw;
float old_Lead[3];

/*Prop shield varibles*/
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

// timing the code
volatile int cycles;


void setup() {

  Serial.begin(9600);
  Serial.setTimeout(1); // the timeout for serial.read is standard @ 1000 - we don't want that

  for (int thisPin = 14; thisPin < 18; thisPin++) {
    pinMode(thisPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(thisPin), blink, CHANGE);
  }

  pinMode(22, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(22), blink, CHANGE); //resset pin

  /* LED 13 used for flight mode indicator */
  DDRB |= B00100000;
  PORTB &= B11011111;

  Propeller[0].attach(3);
  Propeller[1].attach(7);
  Propeller[2].attach(6);
  Propeller[3].attach(4);
  Propeller[4].attach(5);
  Propeller[5].attach(2);

  for (int thisProp = 0; thisProp < pinCount; thisProp++) {
    Propeller[thisProp].writeMicroseconds(1000);
  }

  imu.begin();
  filter.begin(400); // sample freq for prop shield. 400Hz is max in hybrid mode

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  /*Calculate tustin values*/
  f1[0] = (tau_I[0] * K[0] + 1 ) / (tau_I[0] * K[0]);
  f2[0] = (1 - tau_I[0] * K[0]) / (tau_I[0] * K[0]);
  K1[0] = (tau_D[0] * K[0] + 1) / (1 + alpha[0] * tau_D[0] * K[0]);
  K2[0] = (1 - tau_D[0] * K[0]) / (1 + alpha[0] * tau_D[0] * K[0]);
  K3[0] = (1 - alpha[0] * tau_D[0] * K[0]) / (1 + alpha[0] * tau_D[0] * K[0]);

  f1[1] = (tau_I[0] * K[0] + 1 ) / (tau_I[0] * K[0]);
  f2[1] = (1 - tau_I[0] * K[0]) / (tau_I[0] * K[0]);
  K1[1] = (tau_D[1] * K[1] + 1) / (1 + alpha[1] * tau_D[1] * K[1]);
  K2[1] = (1 - tau_D[1] * K[1]) / (1 + alpha[1] * tau_D[1] * K[1]);
  K3[1] = (1 - alpha[1] * tau_D[1] * K[1]) / (1 + alpha[1] * tau_D[1] * K[1]);

  f1[2] = (tau_I[2] * K[2] + 1 ) / (tau_I[2] * K[2]);
  f2[2] = (1 - tau_I[2] * K[2]) / (tau_I[2] * K[2]);
  K1[2] = (tau_D[2] * K[0] + 1) / (1 + alpha[2] * tau_D[2] * K[2]);
  K2[2] = (1 - tau_D[2] * K[2]) / (1 + alpha[2] * tau_D[2] * K[2]);
  K3[2] = (1 - alpha[2] * tau_D[2] * K[2]) / (1 + alpha[2] * tau_D[2] * K[2]);


  /*Calculate tustin values*/

  while (!(imu.available())); // to make sure the hardware it rdy 2 go
}

void loop() {
  uint32_t startCycleCPU;
  startCycleCPU = ARM_DWT_CYCCNT;
  data_flag = 0;

  // Read the motion sensors
  imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

  mag ? filter.update(gx, gy, gz, ax, ay, az, mx, my, mz) : filter.updateIMU(gx, gy, gz, ax, ay, az); // switching between including/excluting magnetometer data

  flightflag = flightmodes(flightflag, input_pin[0], input_pin[1], input_pin[2], input_pin[3]); // switching between arming and dis-arming

  switch (flightflag) {
    case 0: // dis-armed (default)
      flightMode0();
      PORTB &= B11011111;
      break;
    case 1: // Armed
      flightMode1();
      PORTB |= B00100000;
      break;
    case 2: // better safe the sorry
      failsafe();
      break;
    default: // random input go to fail-safe
      failsafe();
      break;
  }

  cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  while (cycles < loop_time) {
    cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
    if (data_flag == 0 && dataOn)data_vector();
  }
}

///////////////////////// just some handy functions /////////////////////////////////////////
int anti_windup(float a, float b, float c) {
  a = (a < b) ? b : a;
  a = (a > c) ? c : a;
  return a;
}

void stopAll() {
  for (int thisProp = 0; thisProp < pinCount; thisProp++) {
    Propeller[thisProp].writeMicroseconds(1000);
    int thisInput = anti_windup(thisProp, 0, 3);
    Serial_input[thisInput] = 0;
  }
  for (int i = 0; i < 3; i++) {
    Iterm[i] = 0;
    Dterm[i] = 0;
  }
  total_yaw        = 0;
  desired_angle[2] = 0;
}
