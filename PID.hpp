#include <Arduino.h>
#include <math.h>


struct PID{
    double Kp;
    double Kd;
    double Ki;
    
    double pitch_ref;
    
    double P_thrust;
    double I_thrust;
    double D_thrust;
    
    double error;
    double error_prev;
    
    double sum_thrust;
    
    int antiwindup;
    
    void pidSetup(double P, double I, double D);
    void calcThrust(double gyro_pitch, double gyro_pitch_vel);
};
