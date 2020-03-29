#include <Arduino.h>
#include <math.h>
#include "Gyro.hpp"


struct PID{
    double Kp;
    double Kd;
    double Ki;
    
    double P_thrust;
    double I_thrust;
    double D_thrust;
    
    double error;
    double error_prev;
    
    double sum_thrust;
    
    int antiwindup;
    
    GYRO gyro;
    double pitch_ref;
    
    void pidSetup(double P, double I, double D, GYRO g);
    double calcThrust();
}
