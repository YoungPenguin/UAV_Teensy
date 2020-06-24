void flightMode1() {

  if ((input_pin[0] > 1100) && (imu.available())) { //(start==1)
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    yaw_difference = (yaw_previous - yaw);
    total_yaw = Yaw_counter(yaw_difference);
    yaw_previous = yaw;

    desired_angle[0]  = (input_pin[1] - 1500.0) / 100.0;
    desired_angle[1]  = (input_pin[2] - 1500.0) / 100.0;

    desired_angle[2]  = (input_pin[3] - 1500.0) / 5000.0;
    desired_angle[2] += last_yaw;
    last_yaw = desired_angle[2];

    last_yaw = desired_angle[2];

    PC_input(Serial_input[0], Serial_input[1], Serial_input[2], Serial_input[3]); // serial inputs
    
    /* Switch for the serial input - Gain full manual control when switch 7 is set */
    error[0] = 0;//(input_pin[4] < 1500) ? (roll - (desired_angle[0] + Serial_input[0])) : (roll - (desired_angle[0]));
    error[1] = (input_pin[4] < 1500) ? (pitch - (desired_angle[1] + Serial_input[1])) : (pitch - (desired_angle[1]));
    error[2] = 0;//(input_pin[4] < 1500) ? (total_yaw - (desired_angle[2] + Serial_input[2])) : (total_yaw - (desired_angle[2]));
    /* Switch for the serial input - Gain full manual control when switch 7 is set */

    Dof3PID();

    throttle = (input_pin[4] < 1500) ? Serial_input[3] + input_pin[0] : input_pin[0];
    pwm_[0] = throttle  - PID_output[0] - 1.732 * PID_output[1] - PID_output[2];
    pwm_[1] = throttle  - 0.5 * PID_output[0] + PID_output[2];
    pwm_[2] = throttle  - PID_output[0] + 1.732 * PID_output[1] - PID_output[2];
    pwm_[3] = throttle  + PID_output[0] + 1.732 * PID_output[1] + PID_output[2];
    pwm_[4] = throttle  + 0.5 * PID_output[0] - PID_output[2];
    pwm_[5] = throttle  + PID_output[0] - 1.732 * PID_output[1] + PID_output[2]; // roll, pitch, yaw

    pwm_[0] = anti_windup(pwm_[0], 1000, 2000);
    pwm_[1] = anti_windup(pwm_[1], 1000, 2000);
    pwm_[2] = anti_windup(pwm_[2], 1000, 2000);
    pwm_[3] = anti_windup(pwm_[3], 1000, 2000);
    pwm_[4] = anti_windup(pwm_[4], 1000, 2000);
    pwm_[5] = anti_windup(pwm_[5], 1000, 2000);

    for (int Prop_PWM = 0; Prop_PWM < 6; Prop_PWM++) {
      Propeller[Prop_PWM].writeMicroseconds(pwm_[Prop_PWM]);
    }

  }


}
