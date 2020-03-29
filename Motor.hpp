#include <Arduino.h>

#define STEP_PIN_L 3
#define DIR_PIN_L 4
#define STEP_PIN_H 5
#define DIR_PIN_H 6

#define MIN_STEP_MIC 450
#define MAX_STEP_MIC 4000
#define MAX_THRUST 100

enum DIRECTION { BACKWARDS, FORWARDS};

struct MOTOR{

    int stepPIN;
    int dirPIN;
    int direction;
    int step_mic;
    
    void motorSetup(int step, int dir);
    void dirControl(double thrust);
    void motorDrive(double thrust);
};


