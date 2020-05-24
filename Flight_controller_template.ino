
/*
 * UAV Flight controller template:
 * 
 * Target: teensy 3.6
 * 
 * The main program file is called: Flight_controller_template
 * 
 * Functionality and usability:
 *    When using this template you should note:
 *      * Edit the number for propellers accordengly
 *      * Implement the correct motor mixing algorythem
 *      * Check you have placed the imu in the correct direction
 *      * Depending on how you would like to control, edit the input channels 
 *      * Input your own computed Kp, Ki, and Kd values 
 *      * PLEASE TAKE OFF THE PORPELLERS THE FIRST TIME
 * 
 * Last modification:
 *    on $ 25.may.2020
 * 
 * Copyright (c) 2020 Denmark DTU.
 * All rights reserved.
 * 
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

Servo prop__1, prop__2, prop__3, prop__4, prop__5, prop__6;
unsigned long counter_1, counter_2, counter_3, counter_4, counter_5, current_count;
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state, last_CH5_state;

int input_YAW = 0;      // channel 4 
int input_PITCH = 0;    // channel 3 
int input_ROLL = 0;     // channel 2
int input_THROTTLE;     // channel 1
int input_RESET = 0;    // channel 5

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz; 
float roll, pitch, yaw, yaw_previous, yaw_difference;
float roll_smooth, pitch_smooth, yaw_smooth;
float total_yaw=0;
float loop_time; //sæt med ordentlig test
bool first_time=0;

float roll_PID, pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6, roll_error, roll_previous_error;

float roll_kp = 0;
float roll_ki = 0;
float roll_kd = 0;
float roll_desired_angle = 0; //This is the angle in which we whant the

float pitch_kp = 0;
float pitch_ki = 0;
float pitch_kd = 0;
float pitch_desired_angle = 0; //This is the angle in which we whant the

float yaw_kp = 0;
float yaw_ki = 0;
float yaw_kd = 0;

// this is needed to stare in a neutrul position
float yaw_desired_angle = 0; 
float yaw_desired_angle_set = 0;
float difference = 0;
float main_loop_timer = 0;
elapsedMillis MPL = 0;


void setup() {
  Serial.begin(9600);
  pinMode(14, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14), blink, CHANGE);
  pinMode(15, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(15), blink, CHANGE);
  pinMode(16, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(16), blink, CHANGE);
  pinMode(17, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(17), blink, CHANGE);
  pinMode(22, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(22), blink, CHANGE); //resset pin

  prop__1.attach(3);
  prop__2.attach(7);
  prop__3.attach(6);
  prop__4.attach(4);
  prop__5.attach(5);
  prop__6.attach(2);    
  
  prop__1.writeMicroseconds(1000);
  prop__2.writeMicroseconds(1000);
  prop__3.writeMicroseconds(1000);
  prop__4.writeMicroseconds(1000);
  prop__5.writeMicroseconds(1000);
  prop__6.writeMicroseconds(1000);
  
  imu.begin();
  filter.begin(100);
 // imu.setSeaPressure(98900);    
}

void loop() {
  if ((input_THROTTLE > 1100) && (imu.available())) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    /* Please make sure you have positioned the sensor accordingly */
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    yaw_difference = (yaw_previous - yaw);
    yaw_previous = yaw;
   
    if(yaw_difference < -20) {
     yaw = 1;
    } else if(yaw_difference > 20){
     yaw = -1;
    } else {
    yaw = yaw_difference;
    }
    total_yaw += yaw;

    /* Depending on your experience level flysing multiroter set the input range accordenly*/
    roll_desired_angle = map(input_ROLL, 1000, 2000, -10, 10);
    pitch_desired_angle = map(input_PITCH, 1000, 2000, -10, 10);
    yaw_desired_angle_set = map(input_YAW, 1000, 2000, -2, 2);
    yaw_desired_angle_set = yaw_desired_angle_set/10;
    yaw_desired_angle = yaw_desired_angle + yaw_desired_angle_set;

    roll_error = roll - roll_desired_angle;
    pitch_error = pitch - pitch_desired_angle;
    yaw_error = total_yaw - yaw_desired_angle;

    roll_pid_i += (roll_ki*roll_error); 
    pitch_pid_i += (pitch_ki*pitch_error);  
    yaw_pid_i += (yaw_ki*yaw_error);

    roll_pid_i = anti_windup(roll_pid_i, -200, 200);
    pitch_pid_i = anti_windup(pitch_pid_i, -200, 200);
    yaw_pid_i = anti_windup(yaw_pid_i, -200, 200);

    roll_PID = roll_kp * roll_error + roll_pid_i + roll_kd * ((roll_error - roll_previous_error) );
    pitch_PID = pitch_kp * pitch_error + pitch_pid_i + pitch_kd * ((pitch_error - pitch_previous_error));
    yaw_PID = yaw_kp * yaw_error + yaw_pid_i + yaw_kd * ((yaw_error - yaw_previous_error));

    pitch_previous_error = pitch_error; 
    roll_previous_error = roll_error;
    yaw_previous_error = yaw_error;

    roll_PID = anti_windup(roll_PID, -400, 400);
    pitch_PID = anti_windup(pitch_PID, -400, 400);
    yaw_PID = anti_windup(yaw_PID, -400, 400);

/*
 * Depending on what type of multiroter UAV you have implement the corresponding motor mixing algo 
 * An Example is listed below for a hex x copter 
 * 
    pwm_1 = input_THROTTLE - roll_PID - 1.732 * pitch_PID - yaw_PID;
    pwm_2 = input_THROTTLE - 0.5 * roll_PID + yaw_PID;
    pwm_3 = input_THROTTLE - roll_PID + 1.732 * pitch_PID - yaw_PID;
    pwm_4 = input_THROTTLE + roll_PID + 1.732 * pitch_PID + yaw_PID;
    pwm_5 = input_THROTTLE + 0.5 * roll_PID - yaw_PID;
    pwm_6 = input_THROTTLE + roll_PID - 1.732 * pitch_PID + yaw_PID; // roll, pitch, yaw

*/

  pwm_1 = anti_windup(pwm_1, 1000, 2000);
  pwm_2 = anti_windup(pwm_2, 1000, 2000);
  pwm_3 = anti_windup(pwm_3, 1000, 2000);
  pwm_4 = anti_windup(pwm_4, 1000, 2000);
  pwm_5 = anti_windup(pwm_5, 1000, 2000);
  pwm_6 = anti_windup(pwm_6, 1000, 2000);

  prop__1.writeMicroseconds(pwm_1);
  prop__2.writeMicroseconds(pwm_2);
  prop__3.writeMicroseconds(pwm_3);
  prop__4.writeMicroseconds(pwm_4);
  prop__5.writeMicroseconds(pwm_5);
  prop__6.writeMicroseconds(pwm_6);
  } else {
      stopAll();
      roll_pid_i = 0; 
      pitch_pid_i =0;  
      yaw_pid_i = 0;
      total_yaw = 0;
      yaw_desired_angle=0;
    }
    maintain_loop_time();
}

/*For interrupt pint if you choose to use a RC transmitter */

void blink() {
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if (GPIOB_PDIR & 2) { //pin 17 (1B 16)    
    if (last_CH1_state == 0) {                         
      last_CH1_state = 1;        //Store the current state into the last state for the next loop
      counter_1 = current_count; //Set counter_1 to current value.
    }
  }
  else if (last_CH1_state == 1) {                           
    last_CH1_state = 0;                     //Store the current state into the last state for the next loop
    input_THROTTLE = current_count - counter_1; //We make the time difference. Channel 1 is current_time - timer_1.
  }

  ///////////////////////////////////////Channel 2
  if (GPIOB_PDIR & 1) { //pin 16
    if (last_CH2_state == 0) {
      last_CH2_state = 1;
      counter_2 = current_count;
    }
  }
  else if (last_CH2_state == 1) {
    last_CH2_state = 0;
    input_ROLL = current_count - counter_2;
     input_ROLL = input_ROLL -100;
  }
  ///////////////////////////////////////Channel 3 
  if (GPIOC_PDIR & 1) {//pin 15, D2                  pin D10 - B00000100
    if (last_CH3_state == 0) {
      last_CH3_state = 1;
      counter_3 = current_count;
    }
  }
  else if (last_CH3_state == 1) {
    last_CH3_state = 0;
    input_PITCH = current_count - counter_3;
     input_PITCH = input_PITCH -122;
  }
  ///////////////////////////////////////Channel 4
  if (GPIOD_PDIR & 2) { //pin 14
    if (last_CH4_state == 0)  {
      last_CH4_state = 1;
      counter_4 = current_count;
    }
  }
  else if (last_CH4_state == 1) {
    last_CH4_state = 0;
    input_YAW = current_count - counter_4;
  }
  ///////////////////////////////////////Channel 5
  if (GPIOC_PDIR & 2) { //pin 22 (1B 16)
     
    if (last_CH5_state == 0) {                         
      last_CH5_state = 1;        //Store the current state into the last state for the next loop
      counter_5 = current_count; //Set counter_1 to current value.
    }
  }
  else if (last_CH5_state == 1) {                           
    last_CH5_state = 0;                     //Store the current state into the last state for the next loop
    input_RESET = current_count - counter_5; //We make the time difference. Channel 1 is current_time - timer_1.
  }
}

/* Now just some handy functions */
int anti_windup(float a, float b, float c) {
  if (a < b) {
    a = b;
  } else if (a > c) {
    a = c;
  }
  return a;
}

void stopAll() {
  prop__1.writeMicroseconds(1000);
  prop__2.writeMicroseconds(1000);
  prop__3.writeMicroseconds(1000);
  prop__4.writeMicroseconds(1000);
  prop__5.writeMicroseconds(1000);
  prop__6.writeMicroseconds(1000);
}

void maintain_loop_time () {
  difference = micros() - main_loop_timer;
  while (difference < 1000) {
    //Serial.println("Quick enough, loop faster than 1 ms ");
    difference = micros() - main_loop_timer;
  }
  main_loop_timer = micros();
}
