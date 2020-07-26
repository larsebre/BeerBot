#include <Arduino.h>
#include <math.h>

#define pi  3.141


struct PID{
    double Kp;
    double Kd;
    double Ki;
    
    double P_thrust;
    double I_thrust;
    double D_thrust;
    
    double error;
    double error_prev;
    
    double state_prev;
    
    double sum_thrust;
    
    void pidSetup(double P, double I, double D);
    void calcThrust(double state_ref, double state_current, int antiwindup);
};
