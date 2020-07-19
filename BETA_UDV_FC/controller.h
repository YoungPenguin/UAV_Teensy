// all PID controllers has the setup  = {Kp, Ki, Kd, Anti-windup};

// Attitude - Outer loop
float PID_YAW_c[4]   = {0.0, 0.0, 0.0, 0.0};
float PID_PITCH_c[4] = {0.0, 0.0, 0.0, 0.0};
float PID_ROLL_c[4]  = {0.0, 0.0, 0.0, 0.0};

// Rate - Inner loop
float PID_YAW_m[4]   = {0.0, 0.0, 0.0, 0.0};
float PID_PITCH_m[4] = {0.0, 0.0, 0.0, 0.0};
float PID_ROLL_m[4]  = {0.0, 0.0, 0.0, 0.0};

// Height
float PID_BARO[4]  = {0.0, 0.0, 0.0, 0.0};
float PID_SONAR[4] = {0.0, 0.0, 0.0, 0.0};

// GPS
float PID_GPS[4] = {0.0, 0.0, 0.0, 0.0};

#define loop_time 450000 // 2.5ms @ 180Mhz 

bool magOn             = true;
bool dataOn            = false;

bool altitudeHoldBaro  = false;
bool altitudeHoldSonar = false;
bool positionHoldGPS   = false;

// Flight modes
#define DISARMED      0
#define RATE_MODE     1
#define ACRO_MODE     2
#define ATTITUDE_MODE 3

// Primary channel definitions
#define ROLL        0
#define PITCH       1
#define THROTTLE    2
#define YAW         3

// PID pseudo definitions
#define P  0 // Proportional
#define I  1 // Integral
#define D  2 // Derivative
#define AW 3 // Anti-Windup

volatile int cycles;
unsigned short int flightflag = 0;
bool all_ready = false;

// i like time constants a bit more ....
float P_values[9]     = {PID_PITCH_c[P], PID_ROLL_c[P], PID_YAW_c[P], PID_PITCH_m[P], PID_ROLL_m[P], PID_YAW_m[P], PID_BARO[P], PID_SONAR[P], PID_GPS[P]};
float tau_I_values[9] = {P_values[0]/PID_PITCH_c[I], P_values[1]/PID_ROLL_c[I], P_values[2]/PID_YAW_c[I], P_values[3]/PID_PITCH_m[I], P_values[4]/PID_ROLL_m[I], P_values[5]/PID_YAW_m[I], P_values[6]/PID_BARO[I], P_values[7]/PID_SONAR[I], P_values[8]/PID_GPS[I]};
float tau_D_values[9] = {PID_PITCH_c[D]/P_values[0], PID_ROLL_c[D]/P_values[1], PID_YAW_c[D]/P_values[2], PID_PITCH_m[D]/P_values[3], PID_ROLL_m[D]/P_values[4], PID_YAW_m[D]/P_values[5], PID_BARO[D]/P_values[6], PID_SONAR[D]/P_values[7], PID_GPS[D]/P_values[8]};
float AW_values[9]    = {PID_PITCH_c[AW], PID_ROLL_c[AW], PID_YAW_c[AW], PID_PITCH_m[AW], PID_ROLL_m[AW], PID_YAW_m[AW], PID_BARO[AW], PID_SONAR[AW], PID_GPS[AW]};
