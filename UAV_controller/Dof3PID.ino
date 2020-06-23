void Dof3PID() {
  /* 3x PID - [roll, pitch, yaw] = [0,1,2] */
 
    PID_output[0] = roll_pid_values[0] * error[0] + roll_pid_i + roll_pid_values[2] * ((error[0] - old_error[0]) );
    PID_output[1] = pitch_pid_values[0] * error[1] + pitch_pid_i + pitch_pid_values[2] * ((error[1] - old_error[1]));
    PID_output[2] = yaw_pid_values[0] * error[2] + yaw_pid_i + yaw_pid_values[2] * ((error[2] - old_error[2]));

    old_error[1] = error[1];
    old_error[0] = error[0];
    old_error[2] = error[2];

    PID_output[0] = anti_windup(PID_output[0], -400, 400);
    PID_output[1] = anti_windup(PID_output[1], -400, 400);
    PID_output[2] = anti_windup(PID_output[2], -400, 400);

  /* 3x PID - [roll, pitch, yaw] = [0,1,2] */
}
