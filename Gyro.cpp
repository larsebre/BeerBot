#include "Gyro.hpp"


void GYRO::gyroSetup(){
    Serial.begin(9600);
    Wire.begin();
    
    //Wake up gyro
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    
    //Gyro
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();
    
    //Accelerometer
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();
    
    //Filter
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();
    
    pinMode(GYRO_SDA, OUTPUT);
    pinMode(GYRO_SCL, OUTPUT);
    
    pitch_calib = 0.0;
    yaw_calib = 0.0;
}

void GYRO::gyroCalibration(){
    for (unsigned int i = 0; i < 500; i++){
        Wire.beginTransmission(GYRO_ADDRESS);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(GYRO_ADDRESS, 4);
        
        yaw_calib += Wire.read()<<8|Wire.read();
        pitch_calib += Wire.read()<<8|Wire.read();
        
        delayMicroseconds(3700);
    }
    
    yaw_calib /= 500;
    pitch_calib /= 500;
    
}
