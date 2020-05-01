#include "Motor.hpp"
#include "Gyro.hpp"
#include "PID.hpp"
#include <Adafruit_INA219.h>

#define LED_PIN_R 12
#define LED_PIN_G 11

//Creating objects
MOTOR motL;
MOTOR motR;
GYRO gyro;
PID pid;
Adafruit_INA219 ina219;

int printer = 0;
int PC_val = 10;
double thrust = 0;
unsigned long int loop_timer = micros();
double busvoltage = 0;
double batteryLimit = 10.3;

void setup(void) {
  Serial.begin(9600);
  Wire.begin();
  uint32_t currentFrequency;
  ina219.begin();
  TWBR = 12;

  motL.motorSetup(STEP_PIN_L, DIR_PIN_L);
  motR.motorSetup(STEP_PIN_H, DIR_PIN_H);
  gyro.gyroSetup();
  gyro.gyroCalibration();
  gyro.accelAngleCalc();
  pid.pidSetup(0.33, 0.0001, 0.01, 20.0, 0.0, 20.0);              //Without beer: P = 0.285, I = 0.0002, D = 0.0075
                                                                //With full beer: P = 0.33, I = 0.0001, D = 0.01
  //Interrupt setup every 20us
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21);
  OCR2A = 39;
  TCCR2A |= (1 << WGM21);
}

void loop(void) {
  
  busvoltage = ina219.getBusVoltage_V();
  if (busvoltage < batteryLimit){
    digitalWrite(LED_PIN_R, HIGH);
    digitalWrite(LED_PIN_G, LOW);
  }else{
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, HIGH);
  }
  
  
  if ((printer%20) == 0){
    Serial.print(pid.pitch_ref);
    Serial.print('\n');
    printer = 0;
  }
  printer++;

  gyro.updateAngularMotion();
  pid.calcPositionThrust(motR.pulse_total);
  pid.calcThrust(gyro.pitch, gyro.pitch_vel);
  
  //Driving the motor
  motL.dirControl(pid.sum_thrust);
  motR.dirControl(pid.sum_thrust);
  digitalWrite(motL.dirPIN, motL.Direction);
  digitalWrite(motR.dirPIN, motR.Direction);
  pid.sum_thrust = abs(pid.sum_thrust);
  PC_val = 31.25/pid.sum_thrust;
  
  
  
  while (loop_timer > micros());
  loop_timer += 4000;
}

ISR(TIMER2_COMPA_vect){
  motR.motorDrive(PC_val);
  motL.motorDrive(PC_val);
}
