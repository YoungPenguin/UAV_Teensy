
/*
         UAV-configuration:
           3-cw  4-ccw

        2-ccw        5-cw

           1-cw  6-ccw
              (front)
*/

#include <Servo.h>
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>
#include <MadgwickAHRS.h>
#include "Filter.h"
#include <util/crc16.h>

NXPMotionSense imu;
Madgwick filter;

float pwm_[6];

Servo Propeller[6];
unsigned long counter[6];
byte last_CH_state[5];
int input_pin[5];
int pinCount = 6;
float roll_pid_values[3] = {1.3, 0.04, 28};
float pitch_pid_values[3] = {1.3, 0.04, 32};
float yaw_pid_values[3] = {7, 0.05, 10};
float desired_angle[3] = {0, 0, 0}; //This is the angle in which we whant the
float yaw_desired_angle_set = 0;
float yaw_desired_angle = 0; //This is the angle in which we whant the
float pid_i_out[3] = {0, 0, 0};

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, yaw, yaw_previous, yaw_difference;
float total_yaw = 0;
float loop_time; //sæt med ordentlig test
bool first_time = 0;
float difference = 0;
float main_loop_timer = 0;

elapsedMillis MPL = 0;

void setup() {
  Serial.begin(115200);
  for (int thisPin = 14; thisPin < 18; thisPin++) {
    pinMode(thisPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(thisPin), blink, CHANGE);
  }

  pinMode(22, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(22), blink, CHANGE); //resset pin

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
}
void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  if (input_pin[0] < 1000) {
    stopAll();
    pid_i_out[0] = 0;
    pid_i_out[1] = 0;
    pid_i_out[2] = 0;
    total_yaw = 0;
    yaw_desired_angle = 0;
  }


  if ((input_pin[0] > 1100) && (imu.available())) { //(start==1)
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    roll = roll - 2.2;
    pitch = pitch - 2.74;

    yaw_difference = (yaw_previous - yaw);
    yaw_previous = yaw;

    if (yaw_difference < -20)
    {
      yaw = 1;
    }
    else if (yaw_difference > 20)
    {
      yaw = -1;
    }
    else {
      yaw = yaw_difference;
    }

    
    total_yaw += yaw;
    desired_angle[0] = map(input_pin[1], 1000, 2000, -10, 10);
    desired_angle[1]  = map(input_pin[2], 1000, 2000, -10, 10);

    yaw_desired_angle_set = map(input_pin[3], 1000, 2000, -2, 2);
    yaw_desired_angle_set = yaw_desired_angle_set / 10;
    desired_angle[2]  += yaw_desired_angle_set;

    roll_error = roll - desired_angle[0];
    pitch_error = pitch - desired_angle[1];
    yaw_error = total_yaw - desired_angle[2];


    pid_i_out[0] += (roll_pid_values[1] * roll_error);
    pid_i_out[1] += (pitch_pid_values[1] * pitch_error);
    pid_i_out[2] += (yaw_pid_values[1] * yaw_error);

    pid_i_out[0] = anti_windup(pid_i_out[0], -200, 200);
    pid_i_out[1] = anti_windup(pid_i_out[1], -200, 200);
    pid_i_out[2] = anti_windup(pid_i_out[2], -200, 200);

    roll_PID = roll_pid_values[0] * roll_error + pid_i_out[0] + roll_pid_values[2] * ((roll_error - roll_previous_error) );
    pitch_PID = pitch_pid_values[0] * pitch_error + pid_i_out[1] + pitch_pid_values[2] * ((pitch_error - pitch_previous_error));
    yaw_PID = yaw_pid_values[0] * yaw_error + pid_i_out[3] + yaw_pid_values[2] * ((yaw_error - yaw_previous_error));

    pitch_previous_error = pitch_error;
    roll_previous_error = roll_error;
    yaw_previous_error = yaw_error;

    roll_PID = anti_windup(roll_PID, -400, 400);
    pitch_PID = anti_windup(pitch_PID, -400, 400);
    yaw_PID = anti_windup(yaw_PID, -400, 400);

    pwm_[0] = input_pin[0] - roll_PID - 1.732 * pitch_PID - yaw_PID;
    pwm_[1] = input_pin[0] - 0.5 * roll_PID + yaw_PID;
    pwm_[2] = input_pin[0] - roll_PID + 1.732 * pitch_PID - yaw_PID;
    pwm_[3] = input_pin[0] + roll_PID + 1.732 * pitch_PID + yaw_PID;
    pwm_[4] = input_pin[0] + 0.5 * roll_PID - yaw_PID;
    pwm_[5] = input_pin[0] + roll_PID - 1.732 * pitch_PID + yaw_PID; // roll, pitch, yaw

    pwm_[0] = anti_windup(pwm_[0], 1000, 2000);
    pwm_[1] = anti_windup(pwm_[1], 1000, 2000);
    pwm_[2] = anti_windup(pwm_[2], 1000, 2000);
    pwm_[3] = anti_windup(pwm_[3], 1000, 2000);
    pwm_[4] = anti_windup(pwm_[4], 1000, 2000);
    pwm_[5] = anti_windup(pwm_[5], 1000, 2000);

    for (int Prop_PWM = 0; Prop_PWM < 6; Prop_PWM++) {
      Propeller[Prop_PWM].writeMicroseconds(pwm_[Prop_PWM]);
    }

  }



  //maintain_loop_time();
  difference = micros() - main_loop_timer;

  while (difference < 1000) {
      Serial.print(millis()); //time in micros
      Serial.print(",");
      Serial.print(roll);
      Serial.print(",");
      Serial.print(pitch);
      Serial.print(",");
      Serial.print(yaw);
      Serial.print(",");
      Serial.print(ax);
      Serial.print(",");
      Serial.print(ay);
      Serial.print(",");
      Serial.print(az);
      Serial.print(",");
      Serial.print(gx);
      Serial.print(",");
      Serial.print(gy); //pitch acceleration
      Serial.print(",");
      Serial.print(gz);
      Serial.print(",");
      Serial.print(pitch_PID);
      Serial.print(",");
      Serial.print(roll_PID);
      Serial.print(",");
      Serial.print(yaw_PID);
      Serial.print(",");
      Serial.print(pwm_5);
      Serial.print(",");
      Serial.print(pitch_desired_angle);
      Serial.print(",");
      Serial.print(roll_desired_angle);
      Serial.print(",");
      Serial.print(yaw_desired_angle);
      Serial.print(",");
      Serial.print(pitch_error);
      Serial.print(",");
      Serial.print(pwm_1);
      Serial.print(",");
      Serial.print(pwm_2);
      Serial.print(",");
      Serial.print(pwm_3);
      Serial.print(",");
      Serial.print(pwm_4);
      Serial.print(",");
      Serial.print(pwm_5);
      Serial.print(",");
      Serial.println(pwm_6);

    difference = micros() - main_loop_timer;
  }

  main_loop_timer = micros();


}

void blink() {
  counter[5] = micros();
  ///////////////////////////////////////Channel 1
  if (GPIOB_PDIR & 2) { //pin 17 (1B 16)
    if (last_CH_state[0] == 0) {
      last_CH_state[0] = 1;        //Store the current state into the last state for the next loop
      counter[0] = counter[5]; //Set counter_1 to current value.
    }
  }
  else if (last_CH_state[0] == 1) {
    last_CH_state[0] = 0;                     //Store the current state into the last state for the next loop
    input_pin[0] = counter[5] - counter[0]; //We make the time difference. Channel 1 is current_time - timer_1.
  }

  ///////////////////////////////////////Channel 2
  if (GPIOB_PDIR & 1) { //pin 16
    if (last_CH_state[1] == 0) {
      last_CH_state[1] = 1;
      counter[1] = counter[5];
    }
  }
  else if (last_CH_state[1] == 1) {
    last_CH_state[1] = 0;
    input_pin[1] = counter[5] - counter[1];
    input_pin[1] = input_pin[1] - 100;
  }
  ///////////////////////////////////////Channel 3
  if (GPIOC_PDIR & 1) {//pin 15, D2                  pin D10 - B00000100
    if (last_CH_state[2] == 0) {
      last_CH_state[2] = 1;
      counter[2] = counter[5];
    }
  }
  else if (last_CH_state[2] == 1) {
    last_CH_state[2] = 0;
    input_pin[2] = counter[5] - counter[2];
    input_pin[2] = input_pin[2] - 122;                //outcomment for step
  }
  ///////////////////////////////////////Channel 4
  if (GPIOD_PDIR & 2) { //pin 14
    if (last_CH_state[3] == 0)  {
      last_CH_state[3] = 1;
      counter[3] = counter[5];
    }
  }
  else if (last_CH_state[3] == 1) {
    last_CH_state[3] = 0;
    input_pin[3] = counter[5] - counter[3];
  }
  ///////////////////////////////////////Channel 5
  if (GPIOC_PDIR & 2) { //pin 22 (1B 16)

    if (last_CH_state[4] == 0) {
      last_CH_state[4] = 1;        //Store the current state into the last state for the next loop
      counter[4] = counter[5]; //Set counter_1 to current value.
    }
  }
  else if (last_CH_state[4] == 1) {
    last_CH_state[4] = 0;                     //Store the current state into the last state for the next loop
    input_pin[4] = counter[5] - counter[4]; //We make the time difference. Channel 1 is current_time - timer_1.
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
}
