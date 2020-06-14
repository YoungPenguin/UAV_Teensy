void flightMode1() {
  if ((input_pin[0] < 1000) && (input_pin[1] > 1700) && (input_pin[2] > 1700) && (input_pin[3] > 1700)) {
    flightMode = 0;
    digitalWrite(13, LOW);
  }
  if (input_pin[0] > 1000) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    yaw_difference = (yaw_previous - yaw);
    yaw_previous = yaw;
    total_yaw = Yaw_counter(yaw_difference);

    desired_angle[2] = map(input_pin[3], 1000, 2000, -2, 2);
    desired_angle[2] = desired_angle[2] / 10;
    desired_angle[2]  += desired_angle[2];

    desired_angle[0] = map(input_pin[1], 1000, 2000, -10, 10);
    desired_angle[1]  = map(input_pin[2], 1000, 2000, -10, 10);

    roll_error = roll - desired_angle[0];
    pitch_error = pitch - desired_angle[1];
    yaw_error = total_yaw - desired_angle[2];

    pid_i_out[0] += roll_pid_values[1] * T * roll_error;
    pid_i_out[1] += pitch_pid_values[1] * T * pitch_error;
    pid_i_out[2] += yaw_pid_values[1] * T * yaw_error;

    pid_i_out[0] = anti_windup(pid_i_out[0], -3, 3);
    pid_i_out[1] = anti_windup(pid_i_out[1], -3, 3);
    pid_i_out[2] = anti_windup(pid_i_out[2], -3, 3);

    roll_PID  = roll_pid_values[0] * (roll_error + pid_i_out[0] + roll_pid_values[2] * ((roll_error - roll_previous_error) / T));
    pitch_PID = pitch_pid_values[0] * (pitch_error + pid_i_out[0] + pitch_pid_values[2] * ((pitch_error - pitch_previous_error) / T));
    yaw_PID   = yaw_pid_values[0] * (yaw_error + pid_i_out[0] + yaw_pid_values[2] * ((yaw_error - yaw_previous_error) / T));

    pitch_previous_error = pitch_error;
    roll_previous_error = roll_error;
    yaw_previous_error = yaw_error;

    roll_PID = anti_windup(roll_PID, -400, 400);
    pitch_PID = anti_windup(pitch_PID, -400, 400);
    yaw_PID = anti_windup(yaw_PID, -400, 400);

    MotorMix_HEX(input_pin[0], roll_PID, pitch_PID, yaw_PID);

  }
}
