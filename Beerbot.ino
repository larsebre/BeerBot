#include "Motor.hpp"
#include "Gyro.hpp"

//Creating objects
volatile uint8_t counter = 0;
volatile uint8_t timerAlert = LOW;

MOTOR motL;
GYRO gyro;
long loop_timer;


void setup() {
  
  motL.motorSetup(STEP_PIN_L, DIR_PIN_L);
  gyro.gyroSetup();
  gyro.gyroCalibration();
  gyro.accelAngleCalc();

  loop_timer = 0;
}

int printer = 0;

void loop() {

  gyro.updateAngularMotion();
  
  if ((printer%30) == 0){
    Serial.print(gyro.pitch);
    Serial.print('\n');
    printer = 0;
  }
  printer++;
  
  while(micros() - loop_timer < 4000){
    
  }
  loop_timer = micros();
}
