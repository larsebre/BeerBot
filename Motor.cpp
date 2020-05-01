#include "Motor.hpp"


void MOTOR::motorSetup(int step, int dir) {
    stepPIN = step;
    dirPIN = dir;
    Direction = DIRECTION::FORWARDS;
    pulse_counter, pulse_total = 0;
    step_pin_on = true;
    pinMode(stepPIN, OUTPUT);
    pinMode(dirPIN, OUTPUT);
    pinMode(QUARTER_STEP, OUTPUT);
    digitalWrite(QUARTER_STEP, HIGH);
}

void MOTOR::dirControl(double u){
    
    if (dirPIN == DIR_PIN_L){
        if (u > 0){
            Direction = DIRECTION::FORWARDS;
        }else if (u < 0){
            Direction = DIRECTION::BACKWARDS;
        }
    }else{
        if (u < 0){
            Direction = DIRECTION::FORWARDS;
        }else if (u > 0){
            Direction = DIRECTION::BACKWARDS;
        }
    }
}

void MOTOR::motorDrive(int PC_val){
    

    if ((PC_val <= 2000) && (PC_val >= 6)){
      if (step_pin_on){
        if (pulse_counter >= PC_val){
          digitalWrite(stepPIN,LOW);
          step_pin_on = false;
          pulse_counter = 0;
        }
      }else{
        if (pulse_counter >= PC_val){
          digitalWrite(stepPIN,HIGH);
          step_pin_on = true;
          pulse_counter = 0;
            if (Direction == DIRECTION::FORWARDS){
                pulse_total++;
            }else{
                pulse_total--;
            }
        }
      }
    
      pulse_counter++; 
    }else{
      pulse_counter = 0;
    }
}
