void flightMode1() {
  if ((input_pin[0] < 1000) && (input_pin[1] > 1700) && (input_pin[2] > 1700) && (input_pin[3] > 1700)) {
    flightMode = 0;
    digitalWrite(13, LOW);
  }
  if (!(input_pin[0] > 1000)) {
    stopAll();
  }
  PC_input();
  // Read the motion sensors
  imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
  // Update the SensorFusion filter
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  yaw_difference = (yaw_previous - yaw);
  yaw_previous = yaw;
  total_yaw = Yaw_counter(yaw_difference);

  desired_angle[0]  = (input_pin[1] - 1500.0) / 100.0;
  desired_angle[1]  = (input_pin[2] - 1500.0) / 100.0;
  desired_angle[2]  = (input_pin[3] - 1500.0) / 5000.0;
  desired_angle[2] += last_yaw;

  last_yaw = desired_angle[2];

  /* Switch for the serial input */
  if (input_pin[4] < 1500) {
    error[0] = roll - (desired_angle[0] + Serial_input[0]);
    error[1] = pitch - (desired_angle[1] + Serial_input[1]);
    error[2] = total_yaw - (desired_angle[2] + Serial_input[2]);
  } else {
    error[0] = roll - (desired_angle[0]);
    error[1] = pitch - (desired_angle[1]);
    error[2] = total_yaw - (desired_angle[2]);
  }
  /*Switch for the serial input */

  /*3x PID*/
  P_term[0] = roll_pid_values[0] * error[0];
  P_term[1] = pitch_pid_values[0] * error[1];
  P_term[2] = yaw_pid_values[0] * error[2];

  I_term[0] = old_I_term[0] + roll_pid_values[1] * error[0] * T;
  I_term[1] = old_I_term[1] + pitch_pid_values[1] * error[1] * T;
  I_term[2] = old_I_term[2] + yaw_pid_values[1] * error[2] * T;

  D_term[0] = roll_pid_values[2] * (error[0] - old_error[0]) / T;
  D_term[1] = pitch_pid_values[2] * (error[1] - old_error[1]) / T;
  D_term[2] = yaw_pid_values[2] * (error[2] - old_error[2]) / T;

  PID_output[0] = P_term[0] + I_term[0] + D_term[0];
  PID_output[1] = P_term[1] + I_term[1] + D_term[1];
  PID_output[2] = P_term[2] + I_term[2] + D_term[2];

  old_error[0] = error[0];
  old_error[1] = error[1];
  old_error[2] = error[2];
  old_I_term[0] = I_term[0];
  old_I_term[1] = I_term[1];
  old_I_term[2] = I_term[2];

  PID_output[0] = anti_windup(PID_output[0], -400, 400);
  PID_output[1] = anti_windup(PID_output[1], -400, 400);
  PID_output[2] = anti_windup(PID_output[2], -400, 400);
  /*3x PID*/

  MotorMix_HEX(input_pin[0], PID_output[0], PID_output[1], PID_output[2]);


}
