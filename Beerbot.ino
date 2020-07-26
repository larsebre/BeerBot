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
PID angle;
PID angle_vel;
PID vel;
Adafruit_INA219 ina219;

int printer = 0;
int PC_val = 10;
int antiwindup = 1;
double thrust = 0;
double tot_thrust = 0;
unsigned long int loop_timer = micros();
double busvoltage = 0;
double batteryLimit = 10.3;

double angle_ref = 20.0;
double angle_vel_ref = 0.0;
double velocity_ref = 0.0;

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
  
  angle.pidSetup(0.16, 0.07, 0.0);                     //0.2, 0.0, 0.0
  angle_vel.pidSetup(0.005, 0.0, 0.0003);              //0.008, 0.0, 0.00025
  vel.pidSetup(9.5, 0.4, 0.07);                          //9.0, 0.3, 0.06
                                                                
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

  gyro.updateAngularMotion();
  
  if (abs(tot_thrust) > 5){
    antiwindup = 0;
  }else{
    antiwindup = 1;
  }

  /*if ((printer%20) == 0){
    Serial.print(motR.velocity, 4);
    Serial.print(",");
    Serial.println(motR.test, 4);
    printer = 0;
  }
  printer++;*/
  
  angle.calcThrust(angle_ref, gyro.pitch, antiwindup);
  angle_vel.calcThrust(angle_vel_ref, gyro.pitch_vel, antiwindup);
  vel.calcThrust(velocity_ref, motR.velocity, antiwindup);
  tot_thrust = angle.sum_thrust + (1 * angle_vel.sum_thrust) +  (1 * vel.sum_thrust);
  
  
  //Driving the motor
  motL.dirControl(tot_thrust);
  motR.dirControl(tot_thrust);
  digitalWrite(motL.dirPIN, motL.Direction);
  digitalWrite(motR.dirPIN, motR.Direction);
  tot_thrust = abs(tot_thrust);
  PC_val = 31.25/tot_thrust;
  
  
  
  while (loop_timer > micros());
  loop_timer += 4000;
}

ISR(TIMER2_COMPA_vect){
  motR.motorDrive(PC_val);
  motL.motorDrive(PC_val);
}
