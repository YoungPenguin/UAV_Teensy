// Flight modes
#define DISARMED 0
#define RATE_MODE 1
#define ACRO_MODE 2
#define ATTITUDE_MODE 3

// Primary channel definitions
#define THROTTLE    0
#define ROLL        1
#define PITCH       2
#define YAW         3

// PID pseudo definitions
#define P  0 // Proportional
#define I  1 // Integral
#define D  2 // Derivative
#define WG 3 // WindupGuard


#define loop_time 450000 // 2.5ms @ 180Mhz 

volatile int cycles;
int8_t flightflag = 0;

bool all_ready = false;
bool altitudeHoldBaro = false;
bool altitudeHoldSonar = false;
bool positionHoldGPS = false;


// FlightController commands definitions
float commandYaw, commandYawAttitude, commandPitch, commandRoll, commandThrottle;

// Heading related variables
float headingError = 0.0;
float headingSetpoint = 0.0;

// PID variables
float YawCommandPIDSpeed, PitchCommandPIDSpeed, RollCommandPIDSpeed;
float YawMotorSpeed, PitchMotorSpeed, RollMotorSpeed, AltitudeHoldMotorSpeed;
int16_t throttle = 1000;
