#include <Arduino.h>
#include <Wire.h>

#define GYRO_SDA 4
#define GYRO_SCL 5

#define GYRO_ADDRESS 0x68


struct Gyro{
    
    double pitch_calib;
    double yaw_calib;
    
    void gyroSetup();
    void gyroCalibration();
    
};
