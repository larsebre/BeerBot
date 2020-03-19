#include <Arduino.h>

#define STEP_PIN_L 3
#define DIR_PIN_L 4
#define STEP_PIN_H 5
#define DIR_PIN_H 6

enum DIRECTION { BACKWARDS, FORWARDS};

struct MOTOR{

    int stepPIN;
    int dirPIN;
    int direction;
    int ms_delay;
    
    void motorSETUP(int step, int dir);
    void motorDrive();
};
