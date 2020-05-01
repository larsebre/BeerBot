#include <Arduino.h>
#include <math.h>

#define pi  3.141


struct PID{
    double Kp_pitch;
    double Kd_pitch;
    double Ki_pitch;
    
    double Kp_position;
    double Kd_position;
    double Ki_position;
    
    double pitch_ref;
    double pitch_ref_const;
    double position_ref;            //Meters
    
    double P_thrust;
    double I_thrust;
    double D_thrust;
    
    double P_position;
    double I_position;
    double D_position;
    
    double error_pitch;
    double error_prev_pitch;
    
    double error_position;
    double error_prev_position;
    
    double sum_thrust;
    
    int antiwindup_pitch;
    int antiwindup_position;
    
    void pidSetup(double P_pitch, double I_pitch, double D_pitch, double P_position, double I_position, double D_position);
    void calcThrust(double gyro_pitch, double gyro_pitch_vel);
    void calcPositionThrust(double pulse_total);
};
