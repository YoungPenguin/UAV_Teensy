/*
         UAV-configuration:
           3-cw  4-ccw

        2-ccw        5-cw

           1-cw  6-ccw
              (front)

*/
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
#include <util/crc16.h>

NXPMotionSense imu;
Madgwick filter;

Servo prop__1;
Servo prop__2;
Servo prop__3;
Servo prop__4;


unsigned long counter_1, counter_2, counter_3, counter_4, counter_5, current_count;

byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state, last_CH5_state;

int input_YAW = 0;      // channel 4 of the receiver and pin D12 of arduino
int input_PITCH = 0;    // channel 3 of the receiver and pin D9 of arduino
int input_ROLL = 0;     // channel 2 of the receiver and pin D8 of arduino
int input_THROTTLE; // channel 1 of the receiver and pin D10 of arduino
int input_RESET = 0;     // channel 2 of the receiver and pin D8 of arduino

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float roll, pitch, yaw, yaw_previous, yaw_difference;
float roll_smooth, pitch_smooth, yaw_smooth;
float total_yaw = 0;
float loop_time; //sæt med ordentlig test
bool first_time = 0;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6, roll_error, roll_previous_error;
float roll_pid_p = 0;
float roll_pid_i = 0;
float roll_pid_d = 0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
float roll_kp = 1.3;         //3.55, (1.2)
float roll_ki = 0.04;       //0.003, (0.038)
float roll_kd = 10;//.2;         //2.05, 15, 36, (29)
float roll_desired_angle = 0; //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p = 0;
float pitch_pid_i = 0;
float pitch_pid_d = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
float pitch_kp = 1.3;       //1.33 , (1.25), 0.55
float pitch_ki = 0.04;       //0.043
float pitch_kd = 10.0;//      //32
float pitch_desired_angle = 0; //This is the angle in which we whant the
float pitch_old_I, pitch_old_D, roll_old_I, roll_old_D;
//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p = 0;
float yaw_pid_i = 0;
float yaw_pid_d = 0;
///////////////////////////////YAW PID CONSTANTS///////////////////
float yaw_kp = 7;       //3.55, 0.3, 150
float yaw_ki = 0.05;       //0.003, 0.0, 200.5
float yaw_kd = 10;//.22;        //2.05, 15.0, 100
float yaw_desired_angle = 0; //This is the angle in which we whant the
float yaw_desired_angle_set = 0;

float difference = 0;
float main_loop_timer = 0;




float PIDF_Tustin(float kp, float K1, float K2, float f, float error, float & old_error, float & old_I, float & old_Lead, float & PIDF_OUT);


// set values
#define T 0.0025
#define alpha 0.1
float tauD[3] = {0.04, 0.04, 0.04};
float tauI[3] = {0.08, 0.08, 0.08};
float kp[3] = {1.3, 1.3, 1.3};
float K = 2 / T;
float K1[3];
float K2[3];
float f[3];
float previous_I[3] = {0.0, 0.0, 0.0};
float previous_D[3] = {0.0, 0.0, 0.0};

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

  DDRB |= B00100000;  //D13 as output
  PORTB &= B11011111; //D13 set to LOW

  prop__1.attach(5);
  prop__2.attach(4);
  prop__3.attach(2);
  prop__4.attach(3);
  // blå(5) blå(4)
  // hvid(3) hvid(2)

  prop__1.writeMicroseconds(1000);
  prop__2.writeMicroseconds(1000);
  prop__3.writeMicroseconds(1000);
  prop__4.writeMicroseconds(1000);

  imu.begin();
  filter.begin(100);
  // imu.setSeaPressure(98900);






  // compute in setup
  K1[0] = (-2 * alpha * tauD[0] + T ) / (2 * alpha * tauD[0] + T);
  K1[1] = (-2 * alpha * tauD[1] + T) / (2 * alpha * tauD[1] + T);
  K1[2] = (-2 * alpha * tauD[2] + T) / (2 * alpha * tauD[2] + T);

  K2[0] = (2 * tauD[0] * kp[0]) / (2 * alpha * tauD[0] + T);
  K2[1] = (2 * tauD[1] * kp[1]) / (2 * alpha * tauD[1] + T);
  K2[2] = (2 * tauD[2] * kp[2]) / (2 * alpha * tauD[2] + T);

  f[0] = (T / 2) * (kp[0] / tauI[0]);
  f[1] = (T / 2) * (kp[1] / tauI[1]);
  f[2] = (T / 2) * (kp[2] / tauI[2]);


}
void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  Serial.println(roll_error);

  if (input_THROTTLE < 1000) {
    //   Serial.println("Stop all");
    stopAll();
    roll_pid_i = 0;
    pitch_pid_i = 0;
    yaw_pid_i = 0;
    total_yaw = 0;
    yaw_desired_angle = 0;
  }
  //input_THROTTLE> 1100
  if ((input_THROTTLE > 1100) && (imu.available())) { //(start==1)
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    roll = roll + 2.1;
    pitch = pitch;

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

    roll_desired_angle = map(input_ROLL, 1000, 2000, -10, 10);
    pitch_desired_angle = map(input_PITCH, 1000, 2000, -10, 10);


    yaw_desired_angle_set = map(input_YAW, 1000, 2000, -5, 5);
    yaw_desired_angle_set = yaw_desired_angle_set / 10;
    yaw_desired_angle = yaw_desired_angle + yaw_desired_angle_set;

    /*///////////////////////////P I D///////////////////////////////////*/
    roll_error = 0;//roll - roll_desired_angle;
    pitch_error = pitch - pitch_desired_angle;
    yaw_error = 0;//total_yaw - yaw_desired_angle;

    yaw_pid_i += (yaw_ki * yaw_error);
    yaw_PID = yaw_kp * yaw_error + yaw_pid_i + yaw_kd * ((yaw_error - yaw_previous_error));



    roll_PID = PIDF_Tustin(kp[0], K1[0], K2[0], f[0], roll_error, roll_previous_error, previous_I[0], previous_D[0]);
    pitch_PID = PIDF_Tustin(kp[1], K1[1], K2[1], f[1], pitch_error, pitch_previous_error, previous_I[1], previous_D[1]);
    yaw_PID = PIDF_Tustin(kp[2], K1[2], K2[2], f[2], yaw_error, yaw_previous_error, previous_I[2], previous_D[2]);


    roll_PID = anti_windup(roll_PID, -400, 400);
    pitch_PID = anti_windup(pitch_PID, -400, 400);
    yaw_PID = anti_windup(yaw_PID, -400, 400);


    pwm_1 = input_THROTTLE  + roll_PID - pitch_PID + yaw_PID;
    pwm_2 = input_THROTTLE  -  roll_PID - pitch_PID - yaw_PID;
    pwm_3 = input_THROTTLE  -  roll_PID + pitch_PID + yaw_PID;
    pwm_4 = input_THROTTLE  +  roll_PID + pitch_PID - yaw_PID;

    pwm_1 = anti_windup(pwm_1, 1000, 2000);
    pwm_2 = anti_windup(pwm_2, 1000, 2000);
    pwm_3 = anti_windup(pwm_3, 1000, 2000);
    pwm_4 = anti_windup(pwm_4, 1000, 2000);

    prop__1.writeMicroseconds(pwm_1);
    prop__2.writeMicroseconds(pwm_2);
    prop__3.writeMicroseconds(pwm_3);
    prop__4.writeMicroseconds(pwm_4);

  }



  //maintain_loop_time();
  difference = micros() - main_loop_timer;

  while (difference < 2500) {
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.print(pitch_PID);
    Serial.print(", ");
    Serial.println(gy);

    difference = micros() - main_loop_timer;
  }

  main_loop_timer = micros();


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
    input_ROLL = input_ROLL;
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
    input_PITCH = input_PITCH;              //outcomment for step
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
  prop__1.writeMicroseconds(1000);
  prop__2.writeMicroseconds(1000);
  prop__3.writeMicroseconds(1000);
  prop__4.writeMicroseconds(1000);

}
