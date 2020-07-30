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


void MOTOR::pushData(double value, int numbElements){

  for (int i = (numbElements - 1); i >= 0; i--){
    if (i == 0){
      vel_array[i] = value;
    }else{
      vel_array[i] = vel_array[i - 1];
    }
  }
}

double MOTOR::getAverage(){
  double sum = 0.0;

  for (int i = 0; i < elements; i++){
    sum += vel_array[i];
  }

  return (sum / elements);
}


void MOTOR::dirControl(double u){

    pulse = pulse_total;
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

    pushData((((pulse - pulse_prev) * ((0.065 * 3.14) / 800)) / 0.004) , 20);
    velocity =  getAverage();
    
    pulse_prev = pulse;   
}

void MOTOR::motorDrive(int PC_val){

    if ((PC_val <= 350) && (PC_val >= 6)){       //2000
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
