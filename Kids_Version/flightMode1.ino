/*
   This is the first real flight model
   This flight mode include only stabilized
   control of yaw, pitch, and roll.
   
   The Lead term is in the feedback path!!!!!!!!!!!!!
   
   CAREFULL IT'S ARMED
*/

void flightMode1() {
  if ((input_pin[0] > 1100)) { //(start==1)
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    roll_val  = roll * K1[0] + old_roll * K2[0] - K3[0] * old_Lead[0];
    pitch_val = pitch * K1[1] + old_pitch * K2[1] - K3[1] * old_Lead[1];
    yaw_val   = yaw * K1[2] + old_yaw * K2[2] - K3[2] * old_Lead[2];

    yaw_difference = (yaw_previous - yaw_val);
    total_yaw = Yaw_counter(yaw_difference);
    yaw_previous = yaw_val;

    desired_angle[0]  = (input_pin[1] - 1500.0) / 100.0;
    desired_angle[1]  = (input_pin[2] - 1500.0) / 100.0;

    desired_angle[2]  = (input_pin[3] - 1500.0) / 5000.0;
    desired_angle[2] += last_yaw;
    last_yaw = desired_angle[2];

    last_yaw = desired_angle[2];

    PC_input(Serial_input[0], Serial_input[1], Serial_input[2], Serial_input[3]); // serial inputs

    /* Switch for the serial input - Gain full manual control when switch 7 is set */
    error[0] = (input_pin[4] < 1500) ? (roll_val - (desired_angle[0] + Serial_input[0])) : (roll_val - (desired_angle[0]));
    error[1] = (input_pin[4] < 1500) ? (pitch_val - (desired_angle[1] + Serial_input[1])) : (pitch_val - (desired_angle[1]));
    error[2] = (input_pin[4] < 1500) ? (total_yaw - (desired_angle[2] + Serial_input[2])) : (total_yaw - (desired_angle[2]));
    /* Switch for the serial input - Gain full manual control when switch 7 is set */

    Dof3PID();

    throttle = (input_pin[4] < 1500) ? Serial_input[3] + input_pin[0] : input_pin[0];

    MotorMixHex();

  }
}
