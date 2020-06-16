void flightMode0() {
  if ((input_pin[0] < 1000) && (input_pin[1] < 1100) && (input_pin[2] > 1700) && (input_pin[3] < 1300) && (imu.available())) {
    flightMode = 1;
    digitalWrite(13, HIGH);
  }

  stopAll();
}
