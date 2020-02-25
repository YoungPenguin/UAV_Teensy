 /*
 *       UAV-configuration:
 *         3-cw  4-ccw
 *      
 *      2-ccw        5-cw
 *     
 *         1-cw  6-ccw
 *            (front)
 */

#include <Wire.h>
#include <Servo.h>

Servo prop__1;
Servo prop__2;
Servo prop__3;
Servo prop__4;
Servo prop__5;
Servo prop__6;

unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

int input_YAW;      // channel 4 of the receiver and pin D12 of arduino
int input_PITCH;    // channel 3 of the receiver and pin D9 of arduino
int input_ROLL;     // channel 2 of the receiver and pin D8 of arduino
int input_THROTTLE; // channel 1 of the receiver and pin D10 of arduino

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error = 0;                       //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;       //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

//Acc Variables
int acc_error = 0;                          //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180 / 3.141592654;       //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;         //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;             //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;

int i;
int mot_activated = 0;
long activate_count = 0;
long des_activate_count = 0;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_1, pwm_2, pwm_3, pwm_4, pwm_5, pwm_6, roll_error, roll_previous_error;
float roll_pid_p = 0;
float roll_pid_i = 0;
float roll_pid_d = 0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp = 0.7;         //3.55
double roll_ki = 0.006;       //0.003
double roll_kd = 1.2;         //2.05
float roll_desired_angle = 0; //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p = 0;
float pitch_pid_i = 0;
float pitch_pid_d = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp = 0.72;        //3.55
double pitch_ki = 0.006;       //0.003
double pitch_kd = 1.22;        //2.05
float pitch_desired_angle = 0; //This is the angle in which we whant the
float yaw_desired_angle = 0;

void setup() {

  pinMode(14, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(14), blink, CHANGE);
  pinMode(15, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(15), blink, CHANGE);
  pinMode(16, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(16), blink, CHANGE);
  pinMode(17, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(17), blink, CHANGE);

  DDRB |= B00100000;  //D13 as output
  PORTB &= B11011111; //D13 set to LOW

  prop__1.attach(5);
  prop__2.attach(3);
  prop__3.attach(4);
  prop__4.attach(7);
  prop__5.attach(2);
  prop__6.attach(6);

  prop__1.writeMicroseconds(1000);
  prop__2.writeMicroseconds(1000);
  prop__3.writeMicroseconds(1000);
  prop__4.writeMicroseconds(1000);
  prop__5.writeMicroseconds(1000);
  prop__6.writeMicroseconds(1000);

////////////////////////////////////////////////////////////////////////////////////
  Wire.begin();                 //begin the wire comunication
  Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true); //end the transmission

  Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);             //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);             //Set the register bits as 00010000 (100dps full scale)
  Wire.endTransmission(true);   //End the transmission with the gyro

  Wire.beginTransmission(0x68); //Start communication with the address found during search.
  Wire.write(0x1C);             //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);             //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  Serial.begin(9600);
  delay(1000);
  time = millis(); //Start counting time in milliseconds

  if (gyro_error == 0)
  {
    for (int i = 0; i < 200; i++)
    {
      Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
      Wire.write(0x43);             //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 4, true); //We ask for just 4 registers

      Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
      Gyr_rawY = Wire.read() << 8 | Wire.read();

      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX / 32.8);
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY / 32.8);
      if (i == 199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x / 200;
        Gyro_raw_error_y = Gyro_raw_error_y / 200;
        gyro_error = 1;
      }
    }
  } 

  if (acc_error == 0)
  {
    for (int a = 0; a < 200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B); 
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);

      Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0; 
      Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0;
      Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0;

      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));

      if (a == 199)
      {
        Acc_angle_error_x = Acc_angle_error_x / 200;
        Acc_angle_error_y = Acc_angle_error_y / 200;
        acc_error = 1;
      }
    }
  } //end of acc error calculation
} //end of setup loop

void loop()
{

  /////////////////////////////I M U/////////////////////////////////////
  timePrev = time; 
  time = millis(); 
  elapsedTime = (time - timePrev) / 1000;

  //////////////////////////////////////Gyro read/////////////////////////////////////
  Wire.beginTransmission(0x68); 
  Wire.write(0x43);             
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);         
  Gyr_rawX = Wire.read() << 8 | Wire.read(); 
  Gyr_rawY = Wire.read() << 8 | Wire.read();
   /*---X---*/
  Gyr_rawX = (Gyr_rawX / 32.8) - Gyro_raw_error_x;
  /*---Y---*/
  Gyr_rawY = (Gyr_rawY / 32.8) - Gyro_raw_error_y;

  /*---X---*/
  Gyro_angle_x = Gyr_rawX * elapsedTime;
  /*---X---*/
  Gyro_angle_y = Gyr_rawY * elapsedTime;

  //////////////////////////////////////Acc read/////////////////////////////////////
  Wire.beginTransmission(0x68);   
  Wire.write(0x3B);               
  Wire.endTransmission(false);   
  Wire.requestFrom(0x68, 6, true); 

  Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0; 
  Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0;
  Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0;

  /*---X---*/
  Acc_angle_x = (atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_x;
  /*---Y---*/
  Acc_angle_y = (atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_y;

  //////////////////////////////////////Total angle and filter/////////////////////////////////////
  /*---X axis angle---*/
  Total_angle_x = 0.98 * (Total_angle_x + Gyro_angle_x) + 0.02 * Acc_angle_x;
  /*---Y axis angle---*/
  Total_angle_y = 0.98 * (Total_angle_y + Gyro_angle_y) + 0.02 * Acc_angle_y;

  roll_desired_angle = map(input_ROLL, 1000, 2000, -20, 20);
  pitch_desired_angle = map(input_PITCH, 1000, 2000, -20, 20);
  yaw_desired_angle = map(input_YAW, 1000, 2000, -20, 20);

  /*///////////////////////////P I D///////////////////////////////////*/

  Serial.print("Input Throttle: ");
  Serial.print(input_THROTTLE);
  Serial.print(" ");

  Serial.print("Input Pitch: ");
  Serial.print(input_PITCH);
  Serial.print(" ");

  Serial.print("Input Roll: ");
  Serial.print(input_ROLL);
  Serial.print(" ");

  Serial.print("Input yaw: ");
  Serial.print(input_YAW);
  Serial.print(" ");
  
  Serial.println();
  
  roll_error = Total_angle_y - roll_desired_angle;
  pitch_error = Total_angle_x - pitch_desired_angle;

  roll_pid_p = roll_kp * roll_error;
  pitch_pid_p = pitch_kp * pitch_error;

  if (-3 < roll_error < 3) {
    roll_pid_i = roll_pid_i + (roll_ki * roll_error);
  }
  if (-3 < pitch_error < 3) {
    pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);
  }

  roll_pid_d = roll_kd * ((roll_error - roll_previous_error) / elapsedTime);
  pitch_pid_d = pitch_kd * ((pitch_error - pitch_previous_error) / elapsedTime);

  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;

  /*///////////////////////////P I D///////////////////////////////////*/

  anti_windup(roll_PID, -400, 400);
  anti_windup(pitch_PID, -400, 400);

  /* hex-x config */
  pwm_1 = input_THROTTLE - roll_desired_angle - 1.732 * pitch_desired_angle - yaw_desired_angle;
  pwm_2 = input_THROTTLE - 1 / 2 * roll_desired_angle + yaw_desired_angle;
  pwm_3 = input_THROTTLE - roll_desired_angle + 1.732 * pitch_desired_angle - yaw_desired_angle;
  pwm_4 = input_THROTTLE + roll_desired_angle + 1.732 * pitch_desired_angle + yaw_desired_angle;
  pwm_5 = input_THROTTLE + 1 / 2 * roll_PID - yaw_desired_angle;
  pwm_6 = input_THROTTLE + roll_desired_angle - 1.732 * pitch_desired_angle + yaw_desired_angle;

  anti_windup(pwm_1, 1000, 2000);
  anti_windup(pwm_2, 1000, 2000);
  anti_windup(pwm_3, 1000, 2000);
  anti_windup(pwm_4, 1000, 2000);
  anti_windup(pwm_5, 1000, 2000);
  anti_windup(pwm_6, 1000, 2000);

  if (input_THROTTLE > 1020) {
    prop__1.writeMicroseconds(pwm_1);
    prop__2.writeMicroseconds(pwm_2);
    prop__3.writeMicroseconds(pwm_3);
    prop__4.writeMicroseconds(pwm_4);
    prop__5.writeMicroseconds(pwm_5);
    prop__6.writeMicroseconds(pwm_6);
  
  }
  if (input_THROTTLE < 1000) {
   stopAll();
  }
}

void blink() {

  current_count = micros();
  ///////////////////////////////////////Channel 1
  if (GPIOD_PDIR & 2) { //pin 34, bit position 26
     
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
  if (GPIOC_PDIR & 1) { //pin 35, bit position 8                  D9 -- B00000010
    if (last_CH2_state == 0) {
      last_CH2_state = 1;
      counter_2 = current_count;
    }
  }
  else if (last_CH2_state == 1) {
    last_CH2_state = 0;
    input_PITCH = current_count - counter_2;
  }

  ///////////////////////////////////////Channel 3
  
  if (GPIOB_PDIR & 1) {//pin 36, bit position 9                    pin D10 - B00000100
    if (last_CH3_state == 0) {
      last_CH3_state = 1;
      counter_3 = current_count;
    }
  }
  else if (last_CH3_state == 1) {
    last_CH3_state = 0;
    input_ROLL = current_count - counter_3;
  }


  ///////////////////////////////////////Channel 4
  if (GPIOB_PDIR & 2) { //pin 37, bit position 10                                                     pin D12  -- B00010000
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

void anti_windup(float a, float b, float c) {
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

void Test_print() {
  Serial.println("pwm_1 pwm_2 pwm_3 pwm_4 pwm_5 pwm_6");
  Serial.print(pwm_1);
  Serial.print("  -  ");
  Serial.print(pwm_2);
  Serial.print("  -  ");
  Serial.print(pwm_3);
  Serial.print("  -  ");
  Serial.print(pwm_4);
  Serial.print("  -  ");
  Serial.print(pwm_5);
  Serial.print("  -  ");
  Serial.println(pwm_6);
}
