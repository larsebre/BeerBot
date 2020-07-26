#include <Arduino.h>

#define STEP_PIN_L 3
#define DIR_PIN_L 4
#define STEP_PIN_H 6
#define DIR_PIN_H 7
#define QUARTER_STEP 2

#define MIN_STEP_MIC 450
#define MAX_STEP_MIC 4000
#define MAX_THRUST 100

enum DIRECTION { BACKWARDS, FORWARDS};

struct MOTOR{

    int stepPIN;
    int dirPIN;
    int Direction;

    int pulse_counter;
    long pulse_total;
    bool step_pin_on;

    int pulse;
    int pulse_prev;
    int elements = 20;

    double vel_array[20];
    double velocity;
    double test;
    
    void motorSetup(int step, int dir);
    void dirControl(double u);
    void motorDrive(int PC_val);
    void pushData(double value, int numbElements);
    double getAverage();
};
