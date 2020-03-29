#include "Motor.hpp"


void MOTOR::motorSetup(int step, int dir) {
    stepPIN = step;
    dirPIN = dir;
    direction = DIRECTION::FORWARDS;
    step_mic = 0;
    pinMode(stepPIN, OUTPUT);
    pinMode(dirPIN, OUTPUT);
}

void MOTOR::dirControl(double thrust){
    
    if (dirPIN == DIR_PIN_L){
        if (thrust > 0){
            direction = DIRECTION::FORWARDS;
        }else if (thrust < 0){
            direction = DIRECTION::BACKWARDS;
        }
    }else{
        if (thrust < 0){
            direction = DIRECTION::FORWARDS;
        }else if (thrust > 0){
            direction = DIRECTION::BACKWARDS;
        }
    }
}

void MOTOR::motorDrive(double thrust){
    
    dirControl(thrust);
    digitalWrite(dirPIN,direction);
    
    step_mic = (int)((MIN_STEP_MIC - MAX_STEP_MIC) * (thrust/MAX_THRUST) + MAX_STEP_MIC);
    if (step_mic > MAX_STEP_MIC) step_mic = MAX_STEP_MIC;
    
    digitalWrite(stepPIN,HIGH);
    delayMicroseconds(step_mic);
    digitalWrite(stepPIN,LOW);
    delayMicroseconds(step_mic);
    
}
