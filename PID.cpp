#include "PID.hpp"


void PID::pidSetup(double P, double I, double D){
    Kp = P;
    Ki = I;
    Kd = D;
   
    error, error_prev = 0.0;
    sum_thrust = 0.0;
}

void PID::calcThrust(double state_ref, double state_current, int antiwindup){

    error = state_ref - state_current;
    
    P_thrust = Kp * error;
    I_thrust += Ki * (error + error_prev) * 0.5 * 0.004 * antiwindup;
    D_thrust = Kd * (error - error_prev)/0.004;
    
    error_prev = error;

    sum_thrust = 0;
    
    sum_thrust = (P_thrust + I_thrust + D_thrust);

    
    
    if (sum_thrust > 5.0) sum_thrust = 5.0;
    if (sum_thrust < -5.0) sum_thrust = -5.0;
}
