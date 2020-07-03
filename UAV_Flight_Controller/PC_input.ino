/*
   Hey you!
   You have found the custom Serial input protocol
   This is very self-explanitory but here is a table of the commands avilable:

   Action  |  Command  | Input range
   ---------------------------------------
   Roll    |  R+/-###  | +/- 20 deg
   Pitch   |  P+/-###  | +/- 20 deg
   Yaw     |  Y+/-###  | +/- 360 deg
   Throttle|  T+###    | 0 =< # < 1000 us
   Mag     |  M+/-     | On/Off = +/-
   Data    |  D+/-     | On/Off = +/-
   ---------------------------------------

   If you don't follow this, all inputs will just be set to 0

   So year you can just make your own or change
   the way you give input to this

*/

char string[5];
int val  = 0;
int sign = 1;

void PC_input(int & a, int & b, int & c, int & d) {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    string[0] = Serial.read(); // action letter
    string[1] = Serial.read(); //sign

    /* the value itself */
    for (int i = 2; i < 6; i++) {
      string[i] = Serial.read();
    }

    int temp100 = (int)string[2] - 48;
    int temp10  = (int)string[3] - 48;
    int temp1   = (int)string[4] - 48;

    sign = (string[1] == '-') ? -1 : 1;
    val = ((temp100 * 100) + (temp10 * 10) + temp1) * sign;

    switch (string[0]) {
      case 'R':
        a = ((val > -20) && (val < 20)) ? val : 0;
        break;
      case 'P':
        b = ((val > -20) && (val < 20)) ? val : 0;
        break;
      case 'Y':
        c = ((val > -360) && (val < 360)) ? val : 0;
        break;
      case 'T':
        d = ((val >= 0) && (val < 1000)) ? val : 0;
        break;
      case 'M':
        mag = (sign == 1) ? true : false;
        break;
      case 'D':
        data_flag = (sign == 1) ? 0 : 1;
        break;
      default:
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        break;
    }
  }
}
