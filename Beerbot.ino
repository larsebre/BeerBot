#include "Motor.hpp"
#include "Gyro.hpp"
#include "PID.hpp"

//Creating objects
MOTOR motL;
GYRO gyro;
PID pid;

int printer = 0;
int PC_val = 10;
double thrust = 0;
unsigned long int loop_timer = micros();


void setup() {
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;

  motL.motorSetup(STEP_PIN_L, DIR_PIN_L);
  gyro.gyroSetup();
  gyro.gyroCalibration();
  gyro.accelAngleCalc();
  pid.pidSetup(0.1, 0.003, 0.025);

  //Interrupt setup every 20us
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21);
  OCR2A = 39;
  TCCR2A |= (1 << WGM21);
}

void loop() {
  
  if ((printer%20) == 0){
    Serial.print(pid.sum_thrust);
    Serial.print('\n');
    printer = 0;
  }
  printer++;

  gyro.updateAngularMotion();
  pid.calcThrust(gyro.pitch, gyro.pitch_vel);
  
  //Driving the motor
  motL.dirControl(pid.sum_thrust);
  digitalWrite(motL.dirPIN, motL.Direction);
  pid.sum_thrust = abs(pid.sum_thrust);
  PC_val = 31.25/pid.sum_thrust;
  
  while (loop_timer > micros());
  loop_timer += 4000;
  motL.pulse_counter = 0;
}

ISR(TIMER2_COMPA_vect){
  motL.motorDrive(PC_val);
}
