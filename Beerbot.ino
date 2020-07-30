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
PID yaw_vel;
Adafruit_INA219 ina219;

int printer = 0;
int PC_val_L = 10;
int PC_val_R = 10;
int antiwindup = 1;
double thrust = 0;
double tot_thrustR = 0;
double tot_thrustL = 0;
unsigned long int loop_timer = micros();
int incomingByte;
double busvoltage = 0;
double batteryLimit = 10.3;

double angle_ref = 5.0;                //25
double angle_vel_ref = 0.0;
double yaw_vel_ref = 0.0;
double velocity_ref = 0.0;

uint8_t velo;
uint8_t yaw_velo;

char serialRead[2];

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
  
  angle.pidSetup(0.31, 0.0, 0.0);                     //0.20, 0.0, 0.0
  angle_vel.pidSetup(0.011, 0.0, 0.00015);             //0.008, 0.0, 0.0001
  vel.pidSetup(4.0, 0.3, 0.025);                       //5.0, 0.4, 0.03
  yaw_vel.pidSetup(0.011, 0.06, 0.00015);              //0.009, 0.04, 0.0001
                                                                
  //Interrupt setup every 20us
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21);
  OCR2A = 39;
  TCCR2A |= (1 << WGM21);
}

void loop(void) {

if (Serial.available() > 2) {

    serialRead[0] = Serial.read();
    serialRead[1] = Serial.read();
    if (serialRead[0] == 'A'){
      velo = serialRead[1];
      velocity_ref = ((velo - 50) / 500.0) * 4.5;
    }
    if (serialRead[0] == 'B'){
      yaw_velo = serialRead[1];
      yaw_vel_ref = -((yaw_velo - 50) / 5.0) * 14;
    }

    /* say what you got:
    Serial.print("Velocity: ");
    Serial.println(velocity_ref);
    Serial.print("Angle: ");
    Serial.println(yaw_vel_ref);*/
  }
  
  busvoltage = ina219.getBusVoltage_V();
  if (busvoltage < batteryLimit){
    digitalWrite(LED_PIN_R, HIGH);
    digitalWrite(LED_PIN_G, LOW);
  }else{
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, HIGH);
  }

  gyro.updateAngularMotion();
  
  if (abs(tot_thrustR) > 5){
    antiwindup = 0;
  }else{
    antiwindup = 1;
  }

  /*if ((printer%20) == 0){
    Serial.println(gyro.yaw_vel, 4);
    printer = 0;
  }
  printer++;*/
  
  angle.calcThrust(angle_ref, gyro.pitch, antiwindup);
  angle_vel.calcThrust(angle_vel_ref, gyro.pitch_vel, antiwindup);
  vel.calcThrust(velocity_ref, motR.velocity, antiwindup);
  yaw_vel.calcThrust(yaw_vel_ref, gyro.yaw_vel, antiwindup);
  tot_thrustL = angle.sum_thrust +  angle_vel.sum_thrust + vel.sum_thrust + yaw_vel.sum_thrust;
  tot_thrustR = angle.sum_thrust +  angle_vel.sum_thrust + vel.sum_thrust - yaw_vel.sum_thrust;
  
  
  //Driving the motor
  motL.dirControl(tot_thrustL);
  motR.dirControl(tot_thrustR);
  digitalWrite(motL.dirPIN, motL.Direction);
  digitalWrite(motR.dirPIN, motR.Direction);
  tot_thrustL = abs(tot_thrustL);
  tot_thrustR = abs(tot_thrustR);
  PC_val_L = 31.25/tot_thrustL;
  PC_val_R = 31.25/tot_thrustR;
  
  
  
  while (loop_timer > micros());
  loop_timer += 4000;
}

ISR(TIMER2_COMPA_vect){
  motL.motorDrive(PC_val_L);
  motR.motorDrive(PC_val_R);
}
