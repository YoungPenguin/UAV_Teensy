/*
                          FRONT
                    CW (6)     CCW (1)

                      O         O
                       \       /
                        \     /
                         \   /
                          \ /
  LEFT  CCW (5) o--------  # --------o CW (2)    RIGHT
                          / \
                         /   \
                        /     \
                       /       \
                      O         O

                   CW (4)     CCW (3)
                          BACK
*/

#define MOTORS 6
uint16_t MotorOut[MOTORS] = {1000, 1000, 1000, 1000, 1000, 1000};

void updateMotorsMix() {
  // Limit YAW to +- 200 (20%)
  YawMotorSpeed = constrain(YawMotorSpeed, -200, 200);

  // All of the motor outputs are constrained to standard 1000 - 2000 us PWM
  // Minimum Armed Throttle variable was added to prevent motors stopping during flight (in extreme "stabilization" behaviours)
  MotorOut[0] = constrain(throttle - RollMotorSpeed - 1.732 * PitchMotorSpeed -  YawMotorSpeed, 1000, 2000);
  MotorOut[1] = constrain(throttle - 0.5 * RollMotorSpeed + YawMotorSpeed, 1000, 2000);
  MotorOut[2] = constrain(throttle - RollMotorSpeed + 1.732 * PitchMotorSpeed -  YawMotorSpeed, 1000, 2000);
  MotorOut[3] = constrain(throttle + RollMotorSpeed + 1.732 * PitchMotorSpeed +  YawMotorSpeed, 1000, 2000);
  MotorOut[4] = constrain(throttle + 0.5 * RollMotorSpeed -  YawMotorSpeed, 1000, 2000);
  MotorOut[5] = constrain(throttle + RollMotorSpeed - 1.732 * PitchMotorSpeed +  YawMotorSpeed, 1000, 2000);
}
