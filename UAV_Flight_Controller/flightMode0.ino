/*
 * This flight mode i just the default Dis-armed mode
 * Nothing to see here, just put your own motor varibles  
 * to zeroe here and you will be fine
*/

void flightMode0() {
  if (input_pin[0] < 1000) {
    stopAll();
    roll_pid_i = 0;
    pitch_pid_i = 0;
    yaw_pid_i = 0;
    total_yaw = 0;
    desired_angle[2] = 0;
  }
}
