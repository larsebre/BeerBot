#include "PID.hpp"


void PID::pidSetup(double P_pitch, double I_pitch, double D_pitch, double K_2, double P_vel, double I_vel, double D_vel){
    Kp_pitch = P_pitch;
    Ki_pitch = I_pitch;
    Kd_pitch = D_pitch;
    K_internal = K_2;

    Kp_vel = P_vel;
    Ki_vel = I_vel;
    Kd_vel = D_vel;
    
    
    pitch_ref_const = 7.0;                               //Without beer: pitch_ref = 26.0
    pitch_ref = pitch_ref_const;

    velocity_ref = 0;
    velocity_pulses = 0;
   
    error_pitch, error_pitch_prev = 0.0;
    error_vel, error_vel_prev = 0.0;
  
    sum_thrust = 0.0;
    
    antiwindup_pitch = 1;
}

void PID::calcThrust(double gyro_pitch, double gyro_pitch_vel, long pulse_total){     //K_1 for internal feedback

    pulses = pulse_total;
    velocity_pulses = (pulses - pulses_prev);

    error_vel = velocity_ref - velocity_pulses;
 
    if (abs(sum_thrust) >= 5){
        antiwindup_pitch = 0;
    }else{
        antiwindup_pitch = 1;
    }

    P_pitch = Kp_vel * error_vel;
    I_pitch = 0;
    D_pitch = 0;

    pitch_ref = pitch_ref_const - P_pitch;

    sum_thrust = 0;
    
    error_pitch = pitch_ref - gyro_pitch;
    
    P_thrust = Kp_pitch * error_pitch;
    I_thrust += Ki_pitch * (error_pitch + error_pitch_prev) * 0.5 * 0.004 * antiwindup_pitch;
    D_thrust = Kd_pitch * (error_pitch - error_pitch_prev)/0.004;
    
    sum_thrust = (P_thrust + I_thrust + D_thrust - K_internal*gyro_pitch_vel);

    
    
    if (sum_thrust > 5.0) sum_thrust = 5.0;
    if (sum_thrust < -5.0) sum_thrust = -5.0;

    
    error_pitch_prev = error_pitch;
    pulses_prev = pulses;
}
