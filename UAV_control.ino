/*
 *       UAV-configuration:
 *         3-cw  4-ccw
 *      
 *      2-ccw        5-cw
 *     
 *         1-cw  6-ccw
 *            (front)
 */

#include <Servo.h>
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>
#include <util/crc16.h>

NXPMotionSense imu;
NXPSensorFusion filter;

Servo prop__1;
Servo prop__2;
Servo prop__3;
Servo prop__4;
Servo prop__5;
Servo prop__6;

unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

int input_YAW = 0;      // channel 4 of the receiver and pin D12 of arduino
int input_PITCH = 0;    // channel 3 of the receiver and pin D9 of arduino
int input_ROLL = 0;     // channel 2 of the receiver and pin D8 of arduino
int input_THROTTLE; // channel 1 of the receiver and pin D10 of arduino

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz; 
float roll, pitch, yaw, yaw_previous, yaw_difference;
float total_yaw = 0;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6, roll_error, roll_previous_error;
float roll_pid_p = 0;
float roll_pid_i = 0;
float roll_pid_d = 0;

///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp = 1.33;       
double roll_ki = 0.043;     
double roll_kd =36;//.2;       
float roll_desired_angle = 0; //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p = 0;
float pitch_pid_i = 0;
float pitch_pid_d = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp = 1.33;    
double pitch_ki = 0.043;   
double pitch_kd = 37;//.22;    
float pitch_desired_angle = 0; //This is the angle in which we whant the

//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p = 0;
float yaw_pid_i = 0;
float yaw_pid_d = 0;
///////////////////////////////YAW PID CONSTANTS///////////////////
double yaw_kp = 7;       
double yaw_ki = 0.05;    
double yaw_kd = 10;//.22;        
float yaw_desired_angle = 0; //This is the angle in which we whant the
float yaw_desired_angle_set = 0;

float difference = 0;
float main_loop_timer = 0;

elapsedMillis MPL = 0;

void setup() {

  Serial.begin(9600);

  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14), blink, CHANGE);
  attachInterrupt(digitalPinToInterrupt(15), blink, CHANGE);
  attachInterrupt(digitalPinToInterrupt(16), blink, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), blink, CHANGE);

  DDRB |= B00100000;  //D13 as output
  PORTB &= B11011111; //D13 set to LOW

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

////////////////////////////////////////////////////////////////////////////////////
 
  imu.begin();

  filter.begin(100);

  //sets local sea level pressure
  //you can get this from your closest airport from
  //     https://aviationweather.gov/metar
  imu.setSeaPressure(98900);


}
void loop() {
  
  /////////////////////////////I M U/////////////////////////////////////
  timePrev = time; 
  time = millis(); 
  elapsedTime = (time - timePrev) / 1000;
  //////////////////////////////////////Gyro read/////////////////////////////////////

  Serial.print(input_THROTTLE);
  Serial.print(" ");
  Serial.print(input_ROLL);
  Serial.print(" ");
  Serial.print(input_PITCH);
  Serial.print(" ");
  Serial.print(input_YAW);
  Serial.print(" ");
  Serial.println();
  
  if (input_THROTTLE > 1100) {
    
  if(imu.available()){
   imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
     // Update the SensorFusion filter
   filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

     // print the heading, pitch and roll
   roll = -1*filter.getRoll();  
   pitch = -1*(filter.getPitch()); 
   yaw = filter.getYaw();

    if(MPL > 100){
       imu.readAltitude();
       MPL = 0;
       }

   yaw_difference = (yaw_previous - yaw);
   yaw_previous = yaw;

   if(yaw_difference < -20) {
     yaw = 1;
  }
  else if(yaw_difference > 20) {
    yaw = -1;
  }
  else {
    yaw = yaw_difference;
  }
  total_yaw += yaw;

  roll_desired_angle = map(input_ROLL, 1000, 2000, -10, 10);
  pitch_desired_angle = map(input_PITCH, 1000, 2000, -10, 10);
  yaw_desired_angle_set = map(input_YAW, 1000, 2000, -2, 2);
  yaw_desired_angle_set = yaw_desired_angle_set/10;
  
  yaw_desired_angle = yaw_desired_angle + yaw_desired_angle_set;

  /*///////////////////////////P I D///////////////////////////////////*/  
  roll_error = roll - roll_desired_angle;
  pitch_error = pitch - pitch_desired_angle;
  yaw_error = total_yaw - yaw_desired_angle;

  roll_pid_p = roll_kp * roll_error;
  pitch_pid_p = pitch_kp * pitch_error;
  yaw_pid_p = yaw_kp * yaw_error;

  roll_pid_i += (roll_ki*roll_error); 
  pitch_pid_i += (pitch_ki*pitch_error);  
  yaw_pid_i += (yaw_ki*yaw_error);

  roll_pid_i =anti_windup(roll_pid_i, -400, 400);
  pitch_pid_i =anti_windup(pitch_pid_i, -400, 400);
  yaw_pid_i =anti_windup(yaw_pid_i, -400, 400);
 
  roll_pid_d = roll_kd * ((roll_error - roll_previous_error) );
  pitch_pid_d = pitch_kd * ((pitch_error - pitch_previous_error));
  yaw_pid_d = yaw_kd * ((yaw_error - yaw_previous_error));

  roll_PID = roll_kp * roll_error + roll_pid_i + roll_kd * ((roll_error - roll_previous_error) );
  pitch_PID = pitch_kp * pitch_error + pitch_pid_i + pitch_kd * ((pitch_error - pitch_previous_error));
  yaw_PID = yaw_kp * yaw_error + yaw_pid_i + yaw_kd * ((yaw_error - yaw_previous_error));

   pitch_previous_error = pitch_error; 
   roll_previous_error = roll_error;
   yaw_previous_error = yaw_error;
  
  /*///////////////////////////P I D///////////////////////////////////*/

  roll_PID = anti_windup(roll_PID, -400, 400);
  pitch_PID = anti_windup(pitch_PID, -400, 400);
  yaw_PID = anti_windup(yaw_PID, -400, 400);

  /* hex-x config */
  pwm_1 = input_THROTTLE - roll_PID - 1.732 * pitch_PID - yaw_PID;
  pwm_2 = input_THROTTLE - 1 / 2 * roll_PID + yaw_PID;
  pwm_3 = input_THROTTLE - roll_PID + 1.732 * pitch_PID - yaw_PID;
  pwm_4 = input_THROTTLE + roll_PID + 1.732 * pitch_PID + yaw_PID;
  pwm_5 = input_THROTTLE + 1 / 2 * roll_PID - yaw_PID;
  pwm_6 = input_THROTTLE + roll_PID - 1.732 * pitch_PID + yaw_PID; // roll, pitch, yaw
  
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
  }
  }
  if (input_THROTTLE < 1000) {
    Serial.println("Stop all");
   stopAll();
   roll_pid_i = 0; 
  pitch_pid_i =0;  
  yaw_pid_i = 0;
  }
 maintain_loop_time();
}

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
  }

  ///////////////////////////////////////Channel 3
  
  if (GPIOC_PDIR & 1) {//pin 15, D2               
    if (last_CH3_state == 0) {
      last_CH3_state = 1;
      counter_3 = current_count;
    }
  }
  else if (last_CH3_state == 1) {
    last_CH3_state = 0;
    input_PITCH = current_count - counter_3;
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
}

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
  pwm_1 = 1000;
  pwm_2 = 1000;
  pwm_3 = 1000;
  pwm_4 = 1000;
  pwm_5 = 1000;
  pwm_6 = 1000;
  prop__1.writeMicroseconds(pwm_1);
  prop__2.writeMicroseconds(pwm_2);
  prop__3.writeMicroseconds(pwm_3);
  prop__4.writeMicroseconds(pwm_4);
  prop__5.writeMicroseconds(pwm_5);
  prop__6.writeMicroseconds(pwm_6);
}

void maintain_loop_time () {
  difference = micros() - main_loop_timer;
  while (difference < 5000) {
    difference = micros() - main_loop_timer;
  }
  //Serial.println(difference);
  main_loop_timer = micros();
}
