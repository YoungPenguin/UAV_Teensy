/*
   The PIDF control (PI-Lead discrete with tustin) for roll, pitch, yaw. 
*/

void Dof3PID() {
  /* 3x PID - [roll, pitch, yaw] = [0,1,2] */

  Iterm[0] = f[0] * (error[0] + old_error[0]) + old_I[0];
  Iterm[1] = f[1] * (error[1] + old_error[1]) + old_I[1];
  Iterm[2] = f[2] * (error[2] + old_error[2]) + old_I[2];

  Dterm[0] = K2[0] * (error[0] - old_error[0]) - (K1[0] * old_D[0]);
  Dterm[1] = K2[1] * (error[1] - old_error[1]) - (K1[1] * old_D[1]);
  Dterm[2] = K2[2] * (error[2] - old_error[2]) - (K1[2] * old_D[2]);

  Iterm[0] = anti_windup(Iterm[0], -100, 100); 
  Iterm[1] = anti_windup(Iterm[1], -100, 100);
  Iterm[2] = anti_windup(Iterm[2], -100, 100);

  PID_output[0] = kp[0] * error[0] + Iterm[0] +  K2[0] * (error[0] - old_error[0]) - (K1[0] * old_D[0]);
  PID_output[1] = kp[1] * error[1] + Iterm[1] +  K2[1] * (error[1] - old_error[1]) - (K1[1] * old_D[1]);
  PID_output[2] = kp[2] * error[2] + Iterm[2] +  K2[2] * (error[2] - old_error[2]) - (K1[2] * old_D[2]);

  PID_output[0] = anti_windup(PID_output[0], -1000, 1000); // full range of output not more
  PID_output[1] = anti_windup(PID_output[1], -1000, 1000);
  PID_output[2] = anti_windup(PID_output[2], -1000, 1000);

  old_I[0] = Iterm[0];
  old_I[1] = Iterm[1];
  old_I[2] = Iterm[2];

  old_D[0] = Dterm[0];
  old_D[1] = Dterm[1];
  old_D[2] = Dterm[2];

  old_error[0] = error[0];
  old_error[1] = error[1];
  old_error[2] = error[2];



}
