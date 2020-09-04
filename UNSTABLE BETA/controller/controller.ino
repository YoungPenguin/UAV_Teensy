// Arduino standard library imports
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

// Custom imports
#include "controller.h"
#include "sensors.h"
#include "math.h"
#include "PID.h"
#include "dataStorage.h"

// == Hardware setup/s == 
#define PHOENIX_SHIELD_V_01
//#define AQ_SHIELD_V_20
//#define AQ_SHIELD_V_21

#ifdef PHOENIX_SHIELD_V_01
    // Led defines
    #define LED_WHITE 2
    #define LED_BLUE 4
    #define LED_ARDUINO 13
    
    // Features requested
    #define Magnetometer
    #define AltitudeHoldBaro
    #define BatteryMonitorCurrent
    #define GPS
    
    // Critical sensors on board (gyro/accel)
    #include <mpu6050_10DOF_stick_px01.h>
    
    // Magnetometer
    #include <Magnetometer_HMC5883L.h>
    
    // Barometer
    #include <Barometer_ms5611.h>
    
    // GPS (ublox neo 6m)
    #include <GPS_ublox.h>
    
    // Current sensor
    #include <BatteryMonitor_current.h>
    
    // Kinematics used
    #include <kinematics_CMP.h>
    
    // Receiver
    #include <Receiver_teensy3_HW_PPM.h>
    
    // Frame type definition
    #include <FrameType_QuadX.h> 

    // Motor / ESC setup
    #define ESC_400HZ
    #include <ESC_teensy3_HW.h>     
#endif

#ifdef AQ_SHIELD_V_20
    // Led defines
    #define LED_ARDUINO 13
    
    // Features requested
    #define Magnetometer
    
    // Critical sensors on board (gyro/accel)
    #include <ITG3200_AQ_v20.h>
    #include <BMA180_AQ_v20.h>
    
    // Magnetometer
    #include <Magnetometer_HMC5883L.h>
    
    // Kinematics used
    #include <kinematics_CMP.h>
    
    // Receiver
    #include <Receiver_328p_HW_PPM.h> // this is just temporary, this shield requires proper mega 1280 2560 support
    
    // Frame type definition
    #include <FrameType_QuadX.h> 

    // Motor / ESC setup
    #include <ESC_328p_HW.h> // this is just temporary, this shield requires proper mega 1280 2560 support      
#endif

#ifdef AQ_SHIELD_V_21
    // Led defines
    #define LED_ARDUINO 13
    
    // Features requested
    #define Magnetometer
    
    // Critical sensors on board (gyro/accel)
    #include <ITG3200_AQ_v21.h>
    #include <ADXL345_AQ_v21.h>
    
    // Magnetometer
    #include <Magnetometer_HMC5883L.h>
    
    // Kinematics used
    #include <kinematics_CMP.h>
    
    // Receiver
    #include <Receiver_328p_HW_PPM.h> // this is just temporary, this shield requires proper mega 1280 2560 support
    
    // Frame type definition
    #include <FrameType_QuadX.h> 

    // Motor / ESC setup
    #include <ESC_328p_HW.h> // this is just temporary, this shield requires proper mega 1280 2560 support     
#endif
// == END of Hardware setup ==

// Global PID object definitions
PID yaw_command_pid;
PID pitch_command_pid;
PID roll_command_pid;

PID yaw_motor_pid;
PID pitch_motor_pid;
PID roll_motor_pid;

#ifdef AltitudeHoldBaro
PID altitude_hold_baro_pid;
#endif

#ifdef AltitudeHoldSonar    
PID altitude_hold_sonar_pid;
#endif

#ifdef GPS
PID yaw_position_hold_pid;
PID pitch_position_hold_pid;
PID roll_position_hold_pid;
#endif  

// Function to reset I terms inside PID objects
void reset_PID_integrals() {
    yaw_command_pid.IntegralReset();
    pitch_command_pid.IntegralReset();
    roll_command_pid.IntegralReset();
    
    yaw_motor_pid.IntegralReset();
    pitch_motor_pid.IntegralReset();
    roll_motor_pid.IntegralReset();
    
    #ifdef AltitudeHoldBaro
    altitude_hold_baro_pid.IntegralReset();
    #endif
    
    #ifdef AltitudeHoldSonar
    altitude_hold_sonar_pid.IntegralReset();
    #endif      
}
  
// Include this last as it contains objects from previous declarations
#include "GPS.h"
#include "PilotCommandProcessor.h"
#include "SerialCommunication.h"  
  
void setup() {
    // PIN settings
    pinMode(LED_ARDUINO, OUTPUT);
    
#ifdef LED_WHITE
    pinMode(LED_WHITE, OUTPUT);
#endif

#ifdef LED_BLUE
    pinMode(LED_BLUE, OUTPUT);
#endif

    // Initialize serial communication
    Serial.begin(38400); // Virtual USB Serial on teensy 3.0 is always 12 Mbit/sec (can be initialized with baud rate 0)

#ifdef GPS
    Serial3.begin(38400);
#endif
 
    // Join I2C bus as master
    Wire.begin();

    // I2C bus hardware specific settings
#if defined(__MK20DX128__)
    I2C0_F = 0x00; // 2.4 MHz (prescaler 20)
    I2C0_FLT = 4;
#endif
    
#if defined(__AVR__)
    TWBR = 12; // 400 KHz (maximum supported frequency)
#endif
    
    // Read data from EEPROM to CONFIG union
    readEEPROM();
    
    // Initialize PID objects with data from EEPROM
    yaw_command_pid = PID(&headingError, &YawCommandPIDSpeed, &headingSetpoint, (float*) &CONFIG.data.PID_YAW_c);
    pitch_command_pid = PID(&kinematicsAngle[YAXIS], &PitchCommandPIDSpeed, &commandPitch, (float*) &CONFIG.data.PID_PITCH_c);
    roll_command_pid = PID(&kinematicsAngle[XAXIS], &RollCommandPIDSpeed, &commandRoll, (float*) &CONFIG.data.PID_ROLL_c);
    
    yaw_motor_pid = PID(&gyro[ZAXIS], &YawMotorSpeed, &YawCommandPIDSpeed, (float*) &CONFIG.data.PID_YAW_m);
    pitch_motor_pid = PID(&gyro[YAXIS], &PitchMotorSpeed, &PitchCommandPIDSpeed, (float*) &CONFIG.data.PID_PITCH_m);
    roll_motor_pid = PID(&gyro[XAXIS], &RollMotorSpeed, &RollCommandPIDSpeed, (float*) &CONFIG.data.PID_ROLL_m);  
    
#ifdef AltitudeHoldBaro
    altitude_hold_baro_pid = PID(&baroAltitudeToHoldTarget, &AltitudeHoldMotorSpeed, &baroAltitudeRunning, (float*) &CONFIG.data.PID_BARO);
#endif
    
#ifdef AltitudeHoldSonar
    altitude_hold_sonar_pid = PID(&sonarAltitudeToHoldTarget, &AltitudeHoldMotorSpeed, &sonarAltitude, (float*) &CONFIG.data.PID_SONAR);
#endif    
    
#ifdef GPS
    // yaw_position_hold_pid = PID();
    // pitch_position_hold = PID();
    // roll_position_hold = PID();
#endif
    
    // Initialize motors/receivers/sensors
    initializeESC();    
    initializeReceiver();
    
    sensors.initializeGyro();
    sensors.initializeAccel();
    
#ifdef Magnetometer
    sensors.initializeMag();
#endif
    
#ifdef AltitudeHoldBaro
    sensors.initializeBaro();    
#endif
    
#ifdef AltitudeHoldSonar
    initializeSonar();
#endif    
    
#ifdef GPS
    gps.initializeBaseStation();
#endif
    
    // All is ready, start the loop
    all_ready = true;
}

void loop() {   
    // Dont start the loop until everything is ready
    if (!all_ready) return; 
 
    // Used to measure loop performance
    itterations++;
    
    // Timer
    currentTime = micros();
    
    // Read data (not faster then every 1 ms)
    if (currentTime - sensorPreviousTime >= 1000) {
        sensors.readGyroSum();
        sensors.readAccelSum();        
        
#ifdef AltitudeHoldSonar
        // Bring sonar pin down (complete TLL trigger pulse)
        readSonarFinish();
#endif    
        
        sensorPreviousTime = currentTime;
    }    
    
    // 100 Hz task loop (10 ms)
    if (currentTime - previousTime > 10000) {
        frameCounter++;
        
        process100HzTask();
        
        // 50 Hz tak (20 ms)
        if (frameCounter % TASK_50HZ == 0) {
            process50HzTask();
        }
        
        // 10 Hz task (100 ms)
        if (frameCounter % TASK_10HZ == 0) {
            process10HzTask();
        }  
        
        // 1 Hz task (1000 ms)
        if (frameCounter % TASK_1HZ == 0) {
            process1HzTask();
        }
        
        // Reset frameCounter back to 0 after reaching 100 (1s)
        if (frameCounter >= 100) {
            frameCounter = 0;
        }
        
        previousTime = currentTime;
    }
}

void process100HzTask() {    
    sensors.evaluateGyro();
    sensors.evaluateAccel();
    
#ifdef AltitudeHoldBaro
    // Baro is being sampled every 10ms (because measuring pressure is slow) 
    sensors.readBaroSum();
#endif    
    
#ifdef GPS
    sensors.readGPS();
#endif

    // Listens/read Serial commands on Serial1 interface (used to pass data from configurator)
    readSerial();
    
    // Update kinematics with latest data
    kinematics_update(gyro[XAXIS], gyro[YAXIS], gyro[ZAXIS], accel[XAXIS], accel[YAXIS], accel[ZAXIS]);
    
    // Update heading
    headingError = kinematicsAngle[ZAXIS] - commandYawAttitude;
    NORMALIZE(headingError); // +- PI
    
    // Update PIDs according the selected mode
    if (flightMode == ATTITUDE_MODE) {
        // Compute command PIDs (with kinematics correction)
        yaw_command_pid.Compute();
        pitch_command_pid.Compute();
        roll_command_pid.Compute();
    } else if (flightMode == RATE_MODE) {
        // Stick input, * 4.0 is the rotation speed factor
        YawCommandPIDSpeed = commandYaw * 4.0;
        PitchCommandPIDSpeed = commandPitch * 4.0;
        RollCommandPIDSpeed = commandRoll * 4.0;        
    }   
    
    // Compute motor PIDs (rate-based)    
    yaw_motor_pid.Compute();
    pitch_motor_pid.Compute();
    roll_motor_pid.Compute();     
    
    // This is the place where the actual "force" gets applied
    if (armed) {
        if (TX_throttle >= CONFIG.data.minimumArmedThrottle) { // disable stabilization if throttle is very low
            updateMotorsMix(); // Frame specific motor mix
        } else {
            // Set all motors to minimum armed throttle
            for (uint8_t i = 0; i < MOTORS; i++) {
                MotorOut[i] = CONFIG.data.minimumArmedThrottle;
            }
        }
        updateMotors(); // Update ESCs
    } 
}

void process50HzTask() {
    processPilotCommands();
    
#ifdef AltitudeHoldBaro
    sensors.evaluateBaroAltitude();
#endif   

#ifdef LED_WHITE
    // Blink "aircraft beacon" LED
    if ((Beacon_LED_state == 51) || (Beacon_LED_state == 59) || (Beacon_LED_state == 67)) {
        digitalWrite(LED_WHITE, HIGH);
    } else {
        digitalWrite(LED_WHITE, LOW);
    }

    Beacon_LED_state++;
    
    if (Beacon_LED_state >= 100) {
        Beacon_LED_state = 0;
    }
#endif
}

void process10HzTask() {
    // Trigger RX failsafe function every 100ms
    RX_failSafe();
    
#ifdef AltitudeHoldSonar
    // Request sonar reading
    readSonar();
#endif    
    
#ifdef Magnetometer
    sensors.readMag();
    sensors.evaluateMag();
#endif
    
#ifdef BatteryMonitorCurrent
    readBatteryMonitorCurrent();
#endif   
    
    // Print itterations per 100ms
#ifdef DISPLAY_ITTERATIONS
    Serial.println(itterations);
#endif
    
    // Blink integrated arduino LED
    Arduino_LED_state = !Arduino_LED_state;
    digitalWrite(LED_ARDUINO, Arduino_LED_state);   
    
    // Reset Itterations
    itterations = 0;    
}

void process1HzTask() {   
#ifdef LED_BLUE
    // Armed/ Dis-armed indicator
    if (armed) {
        digitalWrite(LED_BLUE, HIGH);
    } else {
        digitalWrite(LED_BLUE, LOW);
    }
#endif
}
