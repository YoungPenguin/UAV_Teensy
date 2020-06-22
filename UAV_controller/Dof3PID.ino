void Dof3PID() {
  /* 3x PID - [roll, pitch, yaw] = [0,1,2] */
  P_term[0] = roll_pid_values[0] * error[0];
  P_term[1] = pitch_pid_values[0] * error[1];
  P_term[2] = yaw_pid_values[0] * error[2];

  I_term[0] = old_I_term[0] + roll_pid_values[1] * error[0];
  I_term[1] = old_I_term[1] + pitch_pid_values[1] * error[1];
  I_term[2] = old_I_term[2] + yaw_pid_values[1] * error[2];

  D_term[0] = roll_pid_values[2] * (error[0] - old_error[0]);
  D_term[1] = pitch_pid_values[2] * (error[1] - old_error[1]);
  D_term[2] = yaw_pid_values[2] * (error[2] - old_error[2]);

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
  /* 3x PID - [roll, pitch, yaw] = [0,1,2] */
}
