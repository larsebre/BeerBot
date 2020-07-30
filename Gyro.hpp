#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define GYRO_SDA 2
#define GYRO_SCL 3
#define LED_PIN_R 12
#define LED_PIN_G 11

#define SIGNAL_PATH_RESET 0x68
#define PWR_MGMT_1 0x6B
#define ANGLE_RANGE 0x00
#define RESET_VALUE 0x00
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define CONFIG 0x1A
#define ACCEL_RANGE 0x08
#define FILTER_BDW 0x04           //For bigger bandwidth 0x03
#define GYRO_XOUT_H 0x43
#define ACCEL_XOUT_H 0x3B
#define ACCEL_ZOUT_H 0x3F

#define TIME_STEP 0.004;
#define pi  3.141
#define OFFSET_ACCEL 12


struct GYRO{
    
    double accel_pitch;
    
    double pitch;
    double yaw;
    
    double pitch_vel;
    double pitch_vel_prev;
    double yaw_vel;
    double yaw_vel_prev;

    double pitch_vel_offset;
    double yaw_vel_offset;
    double accelX;
    double accelZ;
    
    void gyroSetup();
    void gyroCalibration();
    void accelAngleCalc();
    void updateAngularMotion(); 
};
