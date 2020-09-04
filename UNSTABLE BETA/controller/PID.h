class PID {
    public:
        // Constructor
        PID() {
        };
        
        PID(float* Input, float* Output, float* Setpoint, float* terms) {
            previous_error = 0.0;
            integral = 0.0;
            
            PID_input = Input;
            PID_output = Output;
            PID_setpoint = Setpoint;
            
            Kp = &terms[P];
            Ki = &terms[I];
            Kd = &terms[D];
            
            windupGuard = &terms[WG];        
        };
        
        void Compute() {            
            unsigned long now = micros();
            float delta_time = (now - last_time) / 1000000.0;
            float error = *PID_setpoint - *PID_input;
            integral = constrain(integral + error * delta_time, -*windupGuard, *windupGuard);
            float derivative = (*PID_input - previous_error) / delta_time;
            
            *PID_output = *Kp * error + *Ki * integral + *Kd * derivative;
            
            previous_error = *PID_input;
            last_time = now;
        };
        
        void IntegralReset() {
            integral = 0.0;
        };
    
    private:
        float *PID_input;
        float *PID_output;
        float *PID_setpoint;
        
        float *Kp, *Ki, *Kd;
        float *windupGuard;
        
        float previous_error;
        float integral;
        unsigned long last_time;
}; 
