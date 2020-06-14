/*
         UAV-configuration:
           3-cw  4-ccw
        2-ccw        5-cw
           1-cw  6-ccw
              (front)
*/

#include <Servo.h>
#include <NXPMotionSense.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <EEPROM.h>

NXPMotionSense imu;
Madgwick filter;

/* functions */
void Transmitter();
void MotorMix_HEX(float input, float roll_PID, float pitch_PID, float yaw_PID); // replace the motor mix with the UAV configuration you are working with
float Yaw_counter(float yaw_difference);

/*Add differet fligth modes*/
void flightMode0(); // dis-armed
void flightMode1(); // armed
// void flightMode2(); // GPS hold

// total number of motors
Servo Propeller[6];
float pwm_[6];

#define T 0.004
#define pinCount 6

int flightMode = 0;

unsigned long counter[6];
byte last_CH_state[5];
int input_pin[5];

float roll_pid_values[3] = {2.3, 0.04, 0.0};
float pitch_pid_values[3] = {2.3, 0.04, 0.0};
float yaw_pid_values[3] = {0, 0.0, 0.0};

float desired_angle[3] = {0, 0, 0};
float yaw_desired_angle_set = 0;

float roll_PID, roll_error, roll_previous_error;
float pitch_PID, pitch_error, pitch_previous_error;
float yaw_PID, yaw_error, yaw_previous_error;

float pid_i_out[3] = {0, 0, 0};

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, yaw, yaw_previous, yaw_difference;
float total_yaw = 0;

// timing the code
volatile int cycles;

void setup() {

  Serial.begin(9600);

  for (int thisPin = 14; thisPin < 18; thisPin++) {
    pinMode(thisPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(thisPin), blink, CHANGE);
  }

  pinMode(22, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(22), blink, CHANGE); //resset pin

  DDRB |= B00100000;  //D13 as output
  PORTB &= B11011111; //D13 set to LOW

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
  filter.begin(100);
  // imu.setSeaPressure(98900);

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

}
void loop() {
  uint32_t startCycleCPU;
  startCycleCPU = ARM_DWT_CYCCNT;
  switch (flightMode) {
    case 0:
      flightMode0();
      break;
    case 1:
      flightMode1();
      break;
    //    case 2:
    //      flightMode2();
    //      break;
    default:
      stopAll();
      break;
  }
  cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;

  while (cycles < 720000) {
    cycles = (ARM_DWT_CYCCNT - startCycleCPU) - 1;
  }
}

///////////////////////// just some handy functions /////////////////////////////////////////

int anti_windup(float a, float b, float c) {
  if (a < b) {
    a = b;
  }
  if (a > c) {
    a = c;
  }
  return a;
}
void stopAll() {
  for (int thisProp = 0; thisProp < pinCount; thisProp++) {
    Propeller[thisProp].writeMicroseconds(1000);
  }
  pid_i_out[0] = 0;
  pid_i_out[1] = 0;
  pid_i_out[2] = 0;
  total_yaw = 0;
  desired_angle[2] = 0;
}