#include "Motor.hpp"


void MOTOR::motorSETUP(int step, int dir) {
    stepPIN = step;
    dirPIN = dir;
    direction = DIRECTION::FORWARDS;
    ms_delay = 0;
    pinMode(stepPIN, OUTPUT);
    pinMode(dirPIN, OUTPUT);
}

void MOTOR::motorDrive(){
    digitalWrite(dirPIN,direction);
    for(int i = 0; i < 1600; i++) {
        digitalWrite(stepPIN,HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPIN,LOW);
        delayMicroseconds(500);
    }
}
