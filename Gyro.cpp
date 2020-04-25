#include "Gyro.hpp"


void GYRO::gyroSetup(){

    //Wake up gyro
    Wire.beginTransmission(SIGNAL_PATH_RESET);
    Wire.write(PWR_MGMT_1);
    Wire.write(RESET_VALUE);
    Wire.endTransmission();
    
    //Gyro config and +/- 250 deg/s
    Wire.beginTransmission(SIGNAL_PATH_RESET);
    Wire.write(GYRO_CONFIG);
    Wire.write(ANGLE_RANGE);
    Wire.endTransmission();
    
    //Accelerometer config and +/- 4g
    Wire.beginTransmission(SIGNAL_PATH_RESET);
    Wire.write(ACCEL_CONFIG);
    Wire.write(ACCEL_RANGE);
    Wire.endTransmission();
    
    //Filter config and 44Hz bandwidth
    Wire.beginTransmission(SIGNAL_PATH_RESET);
    Wire.write(CONFIG);
    Wire.write(FILTER_BDW);
    Wire.endTransmission();
    
    pinMode(GYRO_SDA, OUTPUT);
    pinMode(GYRO_SCL, OUTPUT);

    accel_pitch = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    pitch_vel = 0.0;
    pitch_vel_prev = 0.0;
    yaw_vel = 0.0;
    yaw_vel_prev = 0.0;
    pitch_vel_offset = 0.0;
    yaw_vel_offset = 0.0;
    accelX = 0.0;
    accelZ = 0.0;
    
}

void GYRO::gyroCalibration(){

      pinMode(LED_PIN_R, OUTPUT);
      pinMode(LED_PIN_G, OUTPUT);
      digitalWrite(LED_PIN_R, LOW);
      digitalWrite(LED_PIN_G, LOW);

    Serial.print("Calibrating...");
    Serial.print('\n');

    bool light = false;
    
    for (unsigned int i = 0; i < 500; i++){

        if (i%15 == 0){
          if (light == false){
            digitalWrite(LED_PIN_G, HIGH);
            light = true;
          }else{
            digitalWrite(LED_PIN_G, LOW);
            light = false;
          }
        }
        
        Wire.beginTransmission(SIGNAL_PATH_RESET);
        Wire.write(GYRO_XOUT_H);
        Wire.endTransmission();
        Wire.requestFrom(SIGNAL_PATH_RESET, 4);
        
        yaw_vel_offset += Wire.read()<<8|Wire.read();
        pitch_vel_offset += Wire.read()<<8|Wire.read();
        
        delayMicroseconds(3700);
    }
    
    yaw_vel_offset /= 500;
    pitch_vel_offset /= 500;
    digitalWrite(LED_PIN_G, LOW);
}

void GYRO::accelAngleCalc(){
    
   for (unsigned int i = 0; i < 500; i++){
        Wire.beginTransmission(SIGNAL_PATH_RESET);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission();
        Wire.requestFrom(SIGNAL_PATH_RESET, 2);
        accelX += Wire.read()<<8|Wire.read();
        
        Wire.beginTransmission(SIGNAL_PATH_RESET);
        Wire.write(ACCEL_ZOUT_H);
        Wire.endTransmission();
        Wire.requestFrom(SIGNAL_PATH_RESET, 2);
        accelZ += Wire.read()<<8|Wire.read();
        
        delayMicroseconds(3700);
    }
    
    accelX /= 500;
    accelZ /= 500;
    accelX /= 8192;                                                                 //Converts to number of g's
    accelZ /= 8192;
    
    if (abs(accelX) <= 0.01) accelZ = 0.01;                                         //Makes sure not deviding by zero
    
    pitch = atan(accelZ / accelX);
    pitch = pitch * (180 / pi);                                                            //Convert pitch-angle to degrees
}

void GYRO::updateAngularMotion(){
    Wire.beginTransmission(SIGNAL_PATH_RESET);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(SIGNAL_PATH_RESET, 4);
    
    yaw_vel = ((Wire.read()<<8|Wire.read()) - yaw_vel_offset)/131.0;                //Converting to deg/s
    pitch_vel = ((Wire.read()<<8|Wire.read()) - pitch_vel_offset)/131.0;
    
    yaw += (yaw_vel + yaw_vel_prev) * 0.5 * 0.004;
    pitch += (pitch_vel + pitch_vel_prev) * 0.5 * 0.004;
    
    Wire.beginTransmission(SIGNAL_PATH_RESET);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(SIGNAL_PATH_RESET, 2);
    accelX = Wire.read()<<8|Wire.read();
    
    Wire.beginTransmission(SIGNAL_PATH_RESET);
    Wire.write(ACCEL_ZOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(SIGNAL_PATH_RESET, 2);
    accelZ = Wire.read()<<8|Wire.read();
    
    accel_pitch = atan(accelZ / accelX);
    accel_pitch = accel_pitch * (180 / pi);
    
    pitch = pitch * 0.995 + accel_pitch * 0.005;                                       //Countering the drift
    
    pitch_vel_prev = pitch_vel;
    yaw_vel_prev = yaw_vel;
}
