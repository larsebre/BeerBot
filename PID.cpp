#include "PID.hpp"


void PID::pidSetup(double P, double I, double D, GYRO g){
    Kp = P;
    Ki = I;
    Kd = D;
    gyro = g;
    
    pitch_ref = 0.0;
    error, error_prev = 0.0;
    sum_thrust = 0.0;
    
    antiwindup = 1;
}

double PID::calcThrust(){
    
    if (abs(sum_thrust) > 5){
        antiwindup = 0;
    }else{
        antiwindup = 1;
    }
    
    error = pitch_ref - gyro.pitch;
    
    P_thrust = Kp * error;
    
    I_thrust += Ki * (error + error_prev) * 0.5 * 0.004 * antiwindup;
    
    D_thrust = Kd * gyro.pitch_vel;
    
    sum_thrust = P_thrust + I_thrust + D_thrust;
    
    error_prev = error;
}
