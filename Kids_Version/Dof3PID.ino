/*
   The PIDF control (PI discrete with tustin) for roll, pitch, yaw.
      
   The Lead term is in the feedback path!!!!!!!!!!!!!!!!!
*/

void Dof3PID() {
  /* 3x PID - [roll, pitch, yaw] = [0,1,2] */
  Iterm[0] = f1[0] * error[0] + old_error[0] * f1[0] + old_I[0];
  Iterm[1] = f1[1] * error[1] + old_error[1] * f1[1] + old_I[1];
  Iterm[2] = f1[2] * error[2] + old_error[2] * f1[2] + old_I[2];

  Iterm[0] = anti_windup(Iterm[0], -100, 100);
  Iterm[1] = anti_windup(Iterm[1], -100, 100);
  Iterm[2] = anti_windup(Iterm[2], -100, 100);

  PID_output[0] = kp[0] * Iterm[0];
  PID_output[1] = kp[1] * Iterm[1];
  PID_output[2] = kp[2] * Iterm[2];

  PID_output[0] = anti_windup(PID_output[0], -1000, 1000); // full range of output not more
  PID_output[1] = anti_windup(PID_output[1], -1000, 1000);
  PID_output[2] = anti_windup(PID_output[2], -1000, 1000);


  /*old values needed*/
  old_I[0] = Iterm[0];
  old_I[1] = Iterm[1];
  old_I[2] = Iterm[2];

  old_error[0] = error[0];
  old_error[1] = error[1];
  old_error[2] = error[2];


  old_roll = roll;
  old_pitch = pitch;
  old_yaw = yaw;

  old_Lead[0] = roll_val;
  old_Lead[1] = pitch_val;
  old_Lead[2] = yaw_val;

}
