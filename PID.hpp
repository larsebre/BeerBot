#include <Arduino.h>
#include <math.h>

#define pi  3.141


struct PID{
    double Kp_pitch;
    double Kd_pitch;
    double Ki_pitch;
    double K_internal;

    double Kp_vel;
    double Ki_vel;
    double Kd_vel;
    
    double pitch_ref_const;
    double pitch_ref;

    double velocity_ref;
    double velocity_pulses;
    
    double P_thrust;
    double I_thrust;
    double D_thrust;

    double P_pitch;
    double I_pitch;
    double D_pitch;
    
    double error_pitch;
    double error_pitch_prev;

    double error_vel;
    double error_vel_prev;

    long pulses;
    long pulses_prev;
    
    double sum_thrust;
    
    int antiwindup_pitch;
    
    void pidSetup(double P_pitch, double I_pitch, double D_pitch, double K_2, double P_vel, double I_vel, double D_vel);
    void calcThrust(double gyro_pitch, double gyro_pitch_vel, long pulse_total);
};
