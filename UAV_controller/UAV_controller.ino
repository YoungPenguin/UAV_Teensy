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
void Transmitter();
void motormix(float input, float roll_PID, float pitch_PID, float yaw_PID);
float Yaw_counter(float yaw_difference);

#define T 0.004
#define pinCount 6

float pwm_[6];
int fligthMode = 0;
Servo Propeller[6];
unsigned long counter[6];
byte last_CH_state[5];
int input_pin[5];
float roll_pid_values[3] = {2.3, 0.04, 0.0};
float pitch_pid_values[3] = {2.3, 0.04, 0.0};
float yaw_pid_values[3] = {0, 0.0, 0.0};

float desired_angle[3] = {0, 0, 0}; 
float yaw_desired_angle_set = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, yaw, yaw_previous, yaw_difference;
float roll_smooth, pitch_smooth, yaw_smooth;
float total_yaw = 0;

volatile int cycles;

float roll_PID, roll_error, roll_previous_error;
float pitch_PID, pitch_error, pitch_previous_error;
float yaw_PID, yaw_error, yaw_previous_error;

float pid_i_out[3] = {0, 0, 0};


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

  if (fligthMode == 0) {
    stopAll();
    if ((input_pin[0] < 1000) && (input_pin[1] < 1100) && (input_pin[2] > 1700) && (input_pin[3] < 1300) && (imu.available())) {
      fligthMode = 1;
      digitalWrite(13, HIGH);
    }

  }

  if (fligthMode == 1) { //(start==1)
    if ((input_pin[0] < 1000) && (input_pin[1] > 1700) && (input_pin[2] > 1700) && (input_pin[3] > 1700)) {
      fligthMode = 0;
      digitalWrite(13, LOW);
    }
    if (input_pin[0] > 1000) {
      // Read the motion sensors
      imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
      // Update the SensorFusion filter
      filter.updateIMU(gx, gy, gz, ax, ay, az);
      roll = filter.getRoll();
      pitch = filter.getPitch();
      yaw = filter.getYaw();

      yaw_difference = (yaw_previous - yaw);
      yaw_previous = yaw;
      total_yaw = Yaw_counter(yaw_difference);

      desired_angle[2] = map(input_pin[3], 1000, 2000, -2, 2);
      desired_angle[2] = desired_angle[2] / 10;
      desired_angle[2]  += desired_angle[2];

      desired_angle[0] = map(input_pin[1], 1000, 2000, -10, 10);
      desired_angle[1]  = map(input_pin[2], 1000, 2000, -10, 10);

      roll_error = roll - desired_angle[0];
      pitch_error = pitch - desired_angle[1];
      yaw_error = total_yaw - desired_angle[2];


      pid_i_out[0] += roll_pid_values[1] * T * roll_error;
      pid_i_out[1] += pitch_pid_values[1] * T * pitch_error;
      pid_i_out[2] += yaw_pid_values[1] * T * yaw_error;

      pid_i_out[0] = anti_windup(pid_i_out[0], -3, 3);
      pid_i_out[1] = anti_windup(pid_i_out[1], -3, 3);
      pid_i_out[2] = anti_windup(pid_i_out[2], -3, 3);

      roll_PID  = roll_pid_values[0] * (roll_error + pid_i_out[0] + roll_pid_values[2] * ((roll_error - roll_previous_error) / T));
      pitch_PID = pitch_pid_values[0] * (pitch_error + pid_i_out[0] + pitch_pid_values[2] * ((pitch_error - pitch_previous_error) / T));
      yaw_PID   = yaw_pid_values[0] * (yaw_error + pid_i_out[0] + yaw_pid_values[2] * ((yaw_error - yaw_previous_error) / T));


      pitch_previous_error = pitch_error;
      roll_previous_error = roll_error;
      yaw_previous_error = yaw_error;

      roll_PID = anti_windup(roll_PID, -400, 400);
      pitch_PID = anti_windup(pitch_PID, -400, 400);
      yaw_PID = anti_windup(yaw_PID, -400, 400);

      MotorMix_HEX(input_pin[0], roll_PID, pitch_PID, yaw_PID);

    }
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
