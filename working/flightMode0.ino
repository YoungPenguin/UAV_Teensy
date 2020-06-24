void flightMode0() {
  if (input_pin[0] < 1000) {
    //   Serial.println("Stop all");
    stopAll();
    roll_pid_i = 0;
    pitch_pid_i = 0;
    yaw_pid_i = 0;
    total_yaw = 0;
    desired_angle[2] = 0;
  }
}
