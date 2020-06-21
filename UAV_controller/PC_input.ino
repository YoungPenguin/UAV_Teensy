void PC_input() {
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
        Serial_input[0] = ((val > -20) && (val < 20)) ? val : 0;
        break;
      case 'P':
        Serial_input[1] = ((val > -20) && (val < 20)) ? val : 0;
        break;
      case 'Y':
        Serial_input[2] = ((val > -360) && (val < 360)) ? val : 0;
        break;
      case 'T':
        Serial_input[3] = ((val >= 0) && (val < 1000)) ? val : 0;
        break;
      default:
        for (int thisInput = 0; thisInput < 4; thisInput++) {
          Serial_input[thisInput] = 0;
        }
        break;
    }
  }
}
