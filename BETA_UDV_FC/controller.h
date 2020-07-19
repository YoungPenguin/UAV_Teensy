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
