#include "PID.hpp"


void PID::pidSetup(double P, double I, double D){
    Kp = P;
    Ki = I;
    Kd = D;
    
    pitch_ref = 0.0;
    error, error_prev = 0.0;
    sum_thrust = 0.0;
    
    antiwindup = 1;
}

void PID::calcThrust(double gyro_pitch, double gyro_pitch_vel){
    
    if (abs(sum_thrust) > 5){
        antiwindup = 0;
    }else{
        antiwindup = 1;
    }
    
    error = pitch_ref - gyro_pitch;
    
    P_thrust = Kp * error;
    
    I_thrust += Ki * (error + error_prev) * 0.5 * 0.004 * antiwindup;
    
    D_thrust = Kd * gyro_pitch_vel;
    
    sum_thrust = P_thrust + I_thrust + D_thrust;
    if (sum_thrust > 5.0) sum_thrust = 5.0;
    if (sum_thrust < -5.0) sum_thrust = -5.0;
    
    error_prev = error;
}
