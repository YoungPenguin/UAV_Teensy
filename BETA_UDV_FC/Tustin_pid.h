#ifndef Tustin_pid_h
#define Tustin_pid_h

class Tustin_PID {
    public:
        Tustin_PID(float* Input, float* Setpoint, float* terms);
        float Compute();
        void IntegralReset();

    private:
        float *PID_input;
        float *PID_setpoint;
        
        float *Kp, *f1, *f2, *K, *K1, *K2, *K3;
        float *AntiWindup;
        
        float previous_error, previous_D, previous_I;
        float integral;
};
#endif
