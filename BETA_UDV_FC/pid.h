class PID {
    public:
        PID(float* Input, float* Output, float* Setpoint, float* terms);
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
