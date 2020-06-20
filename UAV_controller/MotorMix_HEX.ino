void MotorMix_HEX(float input, float roll_PID, float pitch_PID, float yaw_PID) {
  pwm_[0] = input - roll_PID - 1.732 * pitch_PID - yaw_PID;
  pwm_[1] = input - 0.5 * roll_PID + yaw_PID;
  pwm_[2] = input - roll_PID + 1.732 * pitch_PID - yaw_PID;
  pwm_[3] = input + roll_PID + 1.732 * pitch_PID + yaw_PID;
  pwm_[4] = input + 0.5 * roll_PID - yaw_PID;
  pwm_[5] = input + roll_PID - 1.732 * pitch_PID + yaw_PID;

  for (int nm = 0; nm < 6; nm++) {
    pwm_[nm] = anti_windup(pwm_[nm], 1000, 2000);
  }

  for (int Prop_PWM = 0; Prop_PWM < 6; Prop_PWM++) {
    Propeller[Prop_PWM].writeMicroseconds(pwm_[Prop_PWM]);
  }
}
