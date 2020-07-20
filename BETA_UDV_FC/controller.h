#include "./Frames/FrameType_QuadX.h"

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
