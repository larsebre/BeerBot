#include "PID.hpp"


void PID::pidSetup(double P_pitch, double I_pitch, double D_pitch, double P_position, double I_position, double D_position){
    Kp_pitch = P_pitch;
    Ki_pitch = I_pitch;
    Kd_pitch = D_pitch;
    
    Kp_position = P_position;
    Kd_position = I_position;
    Ki_position = D_position;
    
    pitch_ref_const = 14.0;
    pitch_ref = pitch_ref_const;                               //Without beer: pitch_ref = 26.0
   
    error_pitch, error_prev_pitch = 0.0;
    error_position, error_prev_position = 0.0;
    sum_thrust = 0.0;
    
    antiwindup_pitch = 1;
    antiwindup_position = 1;
}

void PID::calcThrust(double gyro_pitch, double gyro_pitch_vel){
    
    if (abs(sum_thrust) >= 5){
        antiwindup_pitch = 0;
    }else{
        antiwindup_pitch = 1;
    }
    
    error_pitch = pitch_ref - gyro_pitch;
    
    P_thrust = Kp_pitch * error_pitch;
    I_thrust += Ki_pitch * (error_pitch + error_prev_pitch) * 0.5 * 0.004 * antiwindup_pitch;
    D_thrust = Kd_pitch * gyro_pitch_vel;
    
    sum_thrust = P_thrust + I_thrust - D_thrust;
    if (sum_thrust > 5.0) sum_thrust = 5.0;
    if (sum_thrust < -5.0) sum_thrust = -5.0;
    
    error_prev_pitch = error_pitch;
}

void PID::calcPositionThrust(double pulse_total){

    double Position = pulse_total * ((pi * 0.067)/800.0);
    
    if ((P_position + I_position + D_position) > 10){
           pitch_ref = pitch_ref_const - 10;
           antiwindup_position = 0;
       }if ((P_position + I_position + D_position) < -10) {
           pitch_ref = pitch_ref_const + 10;
           antiwindup_position = 0;
       }else{
           antiwindup_position = 1;
       }
    
    error_position = position_ref - Position;
    
    P_position = Kp_position * error_position;
    I_position += Ki_position * (error_position + error_prev_position) * 0.5 * 0.004 * antiwindup_position;
    D_position = Kd_position * (error_position - error_prev_position) / 0.004;
    
    pitch_ref = pitch_ref_const + (P_position + I_position + D_position);
    
    error_prev_position = error_position;
}
