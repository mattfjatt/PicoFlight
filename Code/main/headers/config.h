#ifndef CONFIG_H
#define CONFIG_H
#include "pico/stdlib.h"

//This file contains variable declarations for variable used in various source files 

#define LOGGING

/*---Controller---*/

//Controller gains:
extern const double kp_x, kp_y, kp_z;
extern const double kd_x, kd_y, kd_z;
extern const double ki_x, ki_y, ki_z;
//Throttle limits:
extern const double throttle_max; // This is the maximum pulse width the PD controller is allowed to send to the motors
extern const double throttle_min; // This is the minimum pulse width the PD controller is allowed to send to the motors 
extern const double throttle_cutoff; // If T is below this, the motors will be switched off
extern const double throttle_poweroff; //This is a safe value the motors will be initialized with
//Integrator wind-up limits:
extern const double windup_x, windup_y, windup_z;

//Arm threshold for the four radio channels
extern const uint32_t arm_ch0, arm_ch1, arm_ch2, arm_ch3;

//Controller behavior selection: Choose between *angle* control and *rate* control 
//#define RATE_ROLL_CONTROL //Not implemented
//#define RATE_PITCH_CONTROL //Not implemented
#define RATE_YAW_CONTROL //Comment this out to control yaw angle instead
extern const double roll_rate, pitch_rate, yaw_rate;

/*---Estimator---*/

//Estimator gains:
extern const double kp_xe, kp_ye, kp_ze; //For the complementary filter
extern const double ki_xe, ki_ye, ki_ze; //For gyro-bias estimation

//Gyroscope LP-filter time constant. Note that this is done on the RP2350, not the IMU
extern const double lambda;

#endif
