void PC_input() {
  // send data only when you receive data:
  if (Serial.available() > 0) {

    string[0] = Serial.read();
    string[1] = Serial.read();
    for (int i = 2; i < 6; i++) {
      string[i] = Serial.read();
    }
    int temp100 = (int)string[2] - 48;
    int temp10 = (int)string[3] - 48;
    int temp1 = (int)string[4] - 48;
    if (string[1] == '-') {
      sign = -1;
    } else {
      sign = 1;
    }
    val = ((temp100 * 100) + (temp10 * 10) + temp1) * sign;
    if (val > -500 && val < 500) {
      switch (string[0]) {
        case 'R':
          Serial_input[0] = val;
          Serial.println("yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy");
          break;
        case 'P':
          Serial_input[1] = val;
          break;
        case 'Y':
          Serial_input[2] = val;
          break;
        case 'T':
          Serial_input[3] = val;
          break;
      }
    }
  }
}
