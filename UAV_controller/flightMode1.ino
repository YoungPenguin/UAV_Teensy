void flightMode1() {
  if ((input_pin[0] < 1000) && (input_pin[1] > 1700) && (input_pin[2] > 1700) && (input_pin[3] > 1700)) {
    flightMode = 0;
    PORTB &= B11011111;
  }

  if (!(input_pin[0] > 1000)) {
    stopAll();
  }

  PC_input();
  // Read the motion sensors
  imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
  // Update the SensorFusion filter
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz); // madgwick
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  yaw_difference = (yaw_previous - yaw);
  yaw_previous = yaw;
  total_yaw = Yaw_counter(yaw_difference);

  /* Don't use the map function, this will give a bad input resolution. Do this instead*/
  desired_angle[0]  = (input_pin[1] - 1500.0) / 100.0;
  desired_angle[1]  = (input_pin[2] - 1500.0) / 100.0;
  desired_angle[2]  = (input_pin[3] - 1500.0) / 5000.0;
  desired_angle[2] += last_yaw;

  last_yaw = desired_angle[2];

  /* Switch for the serial input - Gain full manual control when switch 7 is set */
  error[0] = (input_pin[4] < 1500) ? (roll - (desired_angle[0] + Serial_input[0])) : (roll - (desired_angle[0]));
  error[1] = (input_pin[4] < 1500) ? (pitch - (desired_angle[1] + Serial_input[1])) : (pitch - (desired_angle[1]));
  error[2] = (input_pin[4] < 1500) ? (total_yaw - (desired_angle[2] + Serial_input[2])) : (total_yaw - (desired_angle[2]));
  /* Switch for the serial input - Gain full manual control when switch 7 is set */

  3DofPID();

  throttle = (input_pin[4] < 1500) ? Serial_input[3] + input_pin[0] : input_pin[0];
  MotorMix_HEX(throttle, PID_output[0], PID_output[1], PID_output[2]);
}
