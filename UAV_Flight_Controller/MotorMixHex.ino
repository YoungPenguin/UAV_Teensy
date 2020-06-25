/*
 * Just a basic motor mix algo for a Hexacopter in X config
 * You can just replace this file with the UAV config you are using
*/

void MotorMixHex() {    
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
