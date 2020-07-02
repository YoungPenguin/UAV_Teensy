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
Madgwick filter; // using the madgwick algo - change to mahony if you have problem with drift
// zMPL3115A2 myPressure;
// float Height;

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

// void flightMode3(); // Altitude
// void flightMode4(); // GPS hold

#define loop_time 450000 //45000=2.5ms @ 180Mhz // 36000=2ms
#define pinCount 6

// total number of motors
Servo Propeller[pinCount];
float pwm_[pinCount] = {0.0, 0.0, 0.0};

bool mag = true;

unsigned long counter[6];
byte last_CH_state[5];
int input_pin[5];
int throttle = 0;
int flightflag = 0;

/*{Kp, Ki, Kd}*/
float roll_pid_values[3]    = {1.3, 0.02, 28.0};
float pitch_pid_values[3]   = {1.3, 0.04, 28.0};
float yaw_pid_values[3]     = {7.0, 0.5, 10.0};

/*Controller inputs*/
float desired_angle[3]      = {0.0, 0.0, 0.0};
float yaw_desired_angle_set = 0.0;
float total_yaw             = 0.0;

/*PC input with serial check PC_input for the commands*/
int Serial_input[4] = {0, 0, 0, 0};

int data_flag = 0;

float roll, pitch, yaw, yaw_previous, yaw_difference, last_yaw;

float PID_output[3];
float error[3];
float old_error[3];
float D_term[3]     = {0, 0, 0};
float roll_pid_i = 0;
float pitch_pid_i = 0;
float yaw_pid_i = 0;

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
  // imu.setSeaPressure(98900);

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  /*
    myPressure.begin(); // Get sensor online
    myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setOversampleRate(1); // Set Oversample to the recommended 128
    myPressure.enableEventFlags(); // Enable all three pressure and temp event flags
  */
  while (!(imu.available())); // to make sure the hardware it rdy 2 go
}
void loop() {
  uint32_t startCycleCPU;
  startCycleCPU = ARM_DWT_CYCCNT;
  data_flag = 0;

  // Read the motion sensors
  imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
  // Update the SensorFusion filter

  mag ? filter.update(gx, gy, gz, ax, ay, az, mx, my, mz) : filter.updateIMU(gx, gy, gz, ax, ay, az); // switching between including/excluting magnetometer data
  
  flightflag = flightmodes(flightflag, input_pin[0], input_pin[1], input_pin[2], input_pin[3]);

  switch (flightflag) {
    case 0:
      flightMode0();
      PORTB &= B11011111;
      break;
    case 1:
      flightMode1();
      PORTB |= B00100000;
      break;
    case 2:
      failsafe();
      break;
    /*    case 3:
          flightMode3();
          float pressure = myPressure.readPressure();
          float temperature = 25 + 273.15; // = myPressure.readTemp() + 273.15;
          Height = -(log(pressure / 100900) * 8.3143 * temperature) / (0.28401072);
          break;
    */
    default:
      failsafe();
      break;
  }

  cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  while (cycles < loop_time) {
    cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
    // if (data_flag == 0)data_vector();
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

  roll_pid_i       = 0;
  pitch_pid_i      = 0;
  yaw_pid_i        = 0;
  total_yaw        = 0;
  desired_angle[2] = 0;
}
