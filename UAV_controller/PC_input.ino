void PC_input() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    int availableBytes = Serial.available();
    for (int i = 0; i < availableBytes; i++)
    {
      string[i] = Serial.read();
    }
    int temp100 = (int)string[1] - 48;
    int temp10 = (int)string[2] - 48;
    int temp1 = (int)string[3] - 48;
    val = (temp100 * 100) + (temp10 * 10) + temp1;
  }
  switch (string[0]) {
    case 'T':
      Serial_input[0] = val;
      break;
    case 'R':
      Serial_input[1] = val;
      break;
    case 'P':
      Serial_input[2] = val;
      break;
    case 'Y':
      Serial_input[3] = val;
      break;
  }
}
