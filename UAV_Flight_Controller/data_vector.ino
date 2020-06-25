/* 
 *  Hello there, 
 *  I see you have found your way to the data vector
 *  Here you can put the data you want to print
 *  The data_flag in the buttom is just so you only get 
 *  1 data vector per loop :)
*/

void data_vector() {
  Serial.print(micros());
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
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.print(",");
  Serial.print(PID_output[0]);
  Serial.print(",");
  Serial.print(PID_output[1]);
  Serial.print(",");
  Serial.println(PID_output[2]);
  data_flag = 1;
}
