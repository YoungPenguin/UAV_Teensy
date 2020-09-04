/*  10 DOF stick featuring MPU6050, HMC5883L, MS5611

    According to MPU6050 datasheet, this chip should support I2C speeds up to 400kHz (Fast-mode Fm)
    
    However on Teensy 3.0 i am able to reach 2.4MHz (High-speed mode) without any problems.
    (which cuts down the reading time of accel + gyro to about 180us)
    
    Please note that external pullup resistors are required on Teensy 3.0,
    while they are not "required" on other platforms, i highly recommend adding them.
    1000 ohm pullup seems to work best in my case.
*/

// MPU 6050 Registers
#define MPU6050_ADDRESS         0x68
#define MPUREG_WHOAMI           0x75
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
#define MPUREG_INT_PIN_CFG      0x37
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A
#define MPUREG_ACCEL_XOUT_H     0x3B
#define MPUREG_ACCEL_XOUT_L     0x3C
#define MPUREG_ACCEL_YOUT_H     0x3D
#define MPUREG_ACCEL_YOUT_L     0x3E
#define MPUREG_ACCEL_ZOUT_H     0x3F
#define MPUREG_ACCEL_ZOUT_L     0x40
#define MPUREG_TEMP_OUT_H       0x41
#define MPUREG_TEMP_OUT_L       0x42
#define MPUREG_GYRO_XOUT_H      0x43
#define MPUREG_GYRO_XOUT_L      0x44
#define MPUREG_GYRO_YOUT_H      0x45
#define MPUREG_GYRO_YOUT_L      0x46
#define MPUREG_GYRO_ZOUT_H      0x47
#define MPUREG_GYRO_ZOUT_L      0x48
#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1       0x6B
#define MPUREG_PWR_MGMT_2       0x6C
#define MPUREG_FIFO_COUNTH      0x72
#define MPUREG_FIFO_COUNTL      0x73
#define MPUREG_FIFO_R_W         0x74


// Configuration bits
#define BIT_SLEEP               0x40
#define BIT_H_RESET             0x80
#define BITS_CLKSEL             0x07
#define MPU_CLK_SEL_PLLGYROX    0x01
#define MPU_CLK_SEL_PLLGYROZ    0x03
#define MPU_EXT_SYNC_GYROX      0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN          0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA     0x01

float gyro[3];
float accel[3];
float gyro_temperature;

class MPU6050 {
    public: 
        // Constructor
        MPU6050() {
            // Range = +-1000 dps
            // Scale factor = positive range / positive sensitivity
            // or you can use full range / full sensitivity, which will result in the same output.
            gyroScaleFactor = radians(1000.0 / 32768.0); // 0.030517578125
            
            // Accel scale factor = 9.81 m/s^2 / scale
            accelScaleFactor = 9.81 / 8192.0; // 0.001197509765625
            
            gyroSamples = 0;
            accelSamples = 0;
        };

        void initialize() {
            // Chip reset
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_PWR_MGMT_1, BIT_H_RESET);
            
            // Startup delay 
            delay(100);  
            
            // Check if sensor is alive
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPUREG_WHOAMI);
            Wire.endTransmission();
            
            Wire.requestFrom(MPU6050_ADDRESS, 1);
            
            uint8_t register_value = Wire.read();
            
            if (register_value == 0x68) {
                sensors.sensors_detected |= GYROSCOPE_DETECTED;
                sensors.sensors_detected |= ACCELEROMETER_DETECTED;
            } else {
                return;
            }            
            
            // Enable auxiliary I2C bus bypass
            // *NOT* Necessary for all setups, but some boards have magnetometer attached to the auxiliary I2C bus
            // and without this settings magnetometer won't be accessible.
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_INT_PIN_CFG, 0x02); // I2C _BYPASS _EN 1
            
            // Wake Up device and select GyroZ clock (better performance)
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_PWR_MGMT_2, 0);    
            
            // Sample rate = 1kHz 
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_SMPLRT_DIV, 0x00);

            // FS & DLPF, FS = 1000 degrees/s (dps), DLPF = 42Hz (low pass filter)
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_CONFIG, BITS_DLPF_CFG_42HZ); 

            // Gyro scale 1000 degrees/s (dps)
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_GYRO_CONFIG, BITS_FS_1000DPS);
            
            // Accel scale +-4g (8192LSB/g)
            sensors.i2c_write8(MPU6050_ADDRESS, MPUREG_ACCEL_CONFIG, 0x08);    

            // Initial delay after proper configuration
            // let sensors heat up (especially gyro)
            delay(1500);
            
            // setup axis mapping
            if (CONFIG.data.GYRO_AXIS_MAP.initialized == 0 || CONFIG.data.ACCEL_AXIS_MAP.initialized == 0) { // check if map was defined before, if not "save default" order set  
                CONFIG.data.GYRO_AXIS_MAP.axis1 = 1; // y
                CONFIG.data.GYRO_AXIS_MAP.axis2 = 0; // x
                CONFIG.data.GYRO_AXIS_MAP.axis3 = 2; // z
                
                CONFIG.data.GYRO_AXIS_MAP.axis1_sign = 0;
                CONFIG.data.GYRO_AXIS_MAP.axis2_sign = 0;
                CONFIG.data.GYRO_AXIS_MAP.axis3_sign = 0;
                
                CONFIG.data.ACCEL_AXIS_MAP.axis1 = 1; // y
                CONFIG.data.ACCEL_AXIS_MAP.axis2 = 0; // x
                CONFIG.data.ACCEL_AXIS_MAP.axis3 = 2; // z
                
                CONFIG.data.ACCEL_AXIS_MAP.axis1_sign = 1; // reverse force (*= -1);
                CONFIG.data.ACCEL_AXIS_MAP.axis2_sign = 0;
                CONFIG.data.ACCEL_AXIS_MAP.axis3_sign = 0;
                
                CONFIG.data.GYRO_AXIS_MAP.initialized = 1;
                CONFIG.data.ACCEL_AXIS_MAP.initialized = 1;
                
                // save default in eeprom
                writeEEPROM();
            }
        };
        
        // ~1280ms (in case of error ~ms = calibration sanity passed)
        void calibrate_gyro() {
            static uint8_t retry = 0;
            uint8_t i, count = 128;
            int16_t xSum = 0, ySum = 0, zSum = 0;

            for (i = 0; i < count; i++) {
                readGyroRaw();
                xSum += gyroRaw[XAXIS];
                ySum += gyroRaw[YAXIS];
                zSum += gyroRaw[ZAXIS];
                delay(10);
            }
            
            gyro_offset[XAXIS] = -xSum / count;
            gyro_offset[YAXIS] = -ySum / count;
            gyro_offset[ZAXIS] = -zSum / count; 
            
            // Calibration sanity check
            // if suitable offset couldn't be established, break out of the loop after 10 retries
            if ((abs(gyro_offset[XAXIS]) > 300 || abs(gyro_offset[YAXIS]) > 300 || abs(gyro_offset[ZAXIS]) > 300) && retry < 10) {
                // gyro calibration failed, run again
                retry++;
                
                // small delay before next gyro calibration
                delay(500);
                
                calibrate_gyro();
            }
        };
        
        // ~1280ms (only runs when requested)
        void calibrate_accel() {
            uint8_t i, count = 128;
            int32_t xSum = 0, ySum = 0, zSum = 0;

            for (i = 0; i < count; i++) {
                readAccelRaw();
                xSum += accelRaw[XAXIS];
                ySum += accelRaw[YAXIS];
                zSum += accelRaw[ZAXIS];
                delay(10);
            }
            
            CONFIG.data.ACCEL_BIAS[XAXIS] = xSum / count;
            CONFIG.data.ACCEL_BIAS[YAXIS] = ySum / count;
            CONFIG.data.ACCEL_BIAS[ZAXIS] = (zSum / count) - 8192; // - 1G;

            // Reverse calibration forces
            CONFIG.data.ACCEL_BIAS[XAXIS] *= -1;
            CONFIG.data.ACCEL_BIAS[YAXIS] *= -1;
            CONFIG.data.ACCEL_BIAS[ZAXIS] *= -1;
        };

        // Order and +- signs of each axis depends on the chip orientation.
        // Default order: +X, +Y, +Z
        void readGyroRaw() {
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPUREG_GYRO_XOUT_H);
            Wire.endTransmission();
            
            Wire.requestFrom(MPU6050_ADDRESS, 6);
            
            gyroRaw[CONFIG.data.GYRO_AXIS_MAP.axis1] = ((Wire.read() << 8) | Wire.read()) * (CONFIG.data.GYRO_AXIS_MAP.axis1_sign?-1:1);
            gyroRaw[CONFIG.data.GYRO_AXIS_MAP.axis2] = ((Wire.read() << 8) | Wire.read()) * (CONFIG.data.GYRO_AXIS_MAP.axis2_sign?-1:1);
            gyroRaw[CONFIG.data.GYRO_AXIS_MAP.axis3] = ((Wire.read() << 8) | Wire.read()) * (CONFIG.data.GYRO_AXIS_MAP.axis3_sign?-1:1);
        };

        // Order and +- signs of each axis depends on the chip orientation.
        // Default order: -X, +Y, +Z        
        void readAccelRaw() {
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPUREG_ACCEL_XOUT_H);
            Wire.endTransmission();
            
            Wire.requestFrom(MPU6050_ADDRESS, 6);
            
            accelRaw[CONFIG.data.ACCEL_AXIS_MAP.axis1] = ((Wire.read() << 8) | Wire.read()) * (CONFIG.data.ACCEL_AXIS_MAP.axis1_sign?-1:1);
            accelRaw[CONFIG.data.ACCEL_AXIS_MAP.axis2] = ((Wire.read() << 8) | Wire.read()) * (CONFIG.data.ACCEL_AXIS_MAP.axis2_sign?-1:1);
            accelRaw[CONFIG.data.ACCEL_AXIS_MAP.axis3] = ((Wire.read() << 8) | Wire.read()) * (CONFIG.data.ACCEL_AXIS_MAP.axis3_sign?-1:1);
        };        
        
        void readGyroSum() {
            readGyroRaw();
            
            gyroSum[XAXIS] += gyroRaw[XAXIS];
            gyroSum[YAXIS] += gyroRaw[YAXIS];
            gyroSum[ZAXIS] += gyroRaw[ZAXIS];
            
            gyroSamples++;
        };        
        
        void readAccelSum() {
            readAccelRaw();
            
            accelSum[XAXIS] += accelRaw[XAXIS];
            accelSum[YAXIS] += accelRaw[YAXIS];
            accelSum[ZAXIS] += accelRaw[ZAXIS];  

            accelSamples++;
        };
        
        void evaluateGyro() {
            // Calculate average
            gyro[XAXIS] = gyroSum[XAXIS] / gyroSamples;
            gyro[YAXIS] = gyroSum[YAXIS] / gyroSamples;
            gyro[ZAXIS] = gyroSum[ZAXIS] / gyroSamples;    
            
            // Apply offsets
            gyro[XAXIS] += gyro_offset[XAXIS];
            gyro[YAXIS] += gyro_offset[YAXIS];
            gyro[ZAXIS] += gyro_offset[ZAXIS];         
            
            // Apply correct scaling (at this point gyro is in radians)
            gyro[XAXIS] *= gyroScaleFactor;
            gyro[YAXIS] *= gyroScaleFactor;
            gyro[ZAXIS] *= gyroScaleFactor;
            
            // Reset SUM variables
            gyroSum[XAXIS] = 0;
            gyroSum[YAXIS] = 0;
            gyroSum[ZAXIS] = 0;
            gyroSamples = 0;            
        };
        
        void evaluateAccel() {
            // Calculate average
            accel[XAXIS] = accelSum[XAXIS] / accelSamples;
            accel[YAXIS] = accelSum[YAXIS] / accelSamples;
            accel[ZAXIS] = accelSum[ZAXIS] / accelSamples;  
            
            // Apply offsets
            accel[XAXIS] += CONFIG.data.ACCEL_BIAS[XAXIS];
            accel[YAXIS] += CONFIG.data.ACCEL_BIAS[YAXIS];
            accel[ZAXIS] += CONFIG.data.ACCEL_BIAS[ZAXIS];
            
            // Apply correct scaling (at this point accel reprensents +- 1g = 9.81 m/s^2)
            accel[XAXIS] *= accelScaleFactor;
            accel[YAXIS] *= accelScaleFactor;
            accel[ZAXIS] *= accelScaleFactor;
            
            // Reset SUM variables
            accelSum[XAXIS] = 0;
            accelSum[YAXIS] = 0;
            accelSum[ZAXIS] = 0;
            accelSamples = 0;
        };
        
        // The temperature sensor is -40 to +85 degrees Celsius.
        // It is a signed integer.
        // 340 per degrees Celsius, -512 at 35 degrees.
        // At 0 degrees: -512 - (340 * 35) = -12412
        void readGyroTemperature() {
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPUREG_TEMP_OUT_H);
            Wire.endTransmission();
            
            Wire.requestFrom(MPU6050_ADDRESS, 2);
            
            int16_t raw_temperature = (Wire.read() << 8) | Wire.read();

            gyro_temperature = ((float) raw_temperature + 12412.0) / 340.0;
        };
    private:
        int16_t gyro_offset[3];
        
        float gyroScaleFactor;
        float accelScaleFactor; 

        int16_t gyroRaw[3];
        float gyroSum[3];

        int16_t accelRaw[3];
        float accelSum[3];
        
        uint8_t gyroSamples;
        uint8_t accelSamples;        
};

MPU6050 mpu;

void SensorArray::initializeGyro() {
    mpu.initialize();
    mpu.calibrate_gyro();
}

void SensorArray::initializeAccel() {
}

void SensorArray::calibrateAccel() {
    mpu.calibrate_accel();
}

void SensorArray::readGyroSum() {
    mpu.readGyroSum();
}

void SensorArray::readAccelSum() {
    mpu.readAccelSum();
}

void SensorArray::evaluateGyro() {
    mpu.evaluateGyro();
}

void SensorArray::evaluateAccel() {
    mpu.evaluateAccel();
}
