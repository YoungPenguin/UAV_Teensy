#ifndef Tustin_pid_h
#define Tustin_pid_h

class Tustin_PID {
    public:
        Tustin_PID(float* Input, float* Output, float* Setpoint, float* terms);
        void Compute();
        void IntegralReset();

    private:
        float *PID_input;
        float *PID_output;
        float *PID_setpoint;
        
        float *Kp, *Ki, *Kd;
        float *AntiWindup;
        
        float previous_error;
        float integral;
        unsigned long last_time;
};
#endif
