#ifndef CONFIG_H
#define CONFIG_H
#include "pico/stdlib.h"

//This file contains variable declarations for variable used in various source files 

#define LOGGING

/*---Controller---*/

//Controller gains:
extern const double KpX, KpY, KpZ;
extern const double KdX, KdY, KdZ;
extern const double KiX, KiY, KiZ;
//Throttle limits:
extern const double TMax; // This is the maximum pulse width the PD controller is allowed to send to the motors
extern const double TMin; // This is the minimum pulse width the PD controller is allowed to send to the motors 
extern const double TCutoff; // If T is below this, the motors will be switched off
extern const double TPowerOff; //This is a safe value the motors will be initialized with
//Integrator wind-up limits:
extern const double WindUpX, WindupY, WindUpZ;

//Arm threshold for the four radio channels
extern const uint32_t ArmCh0, ArmCh1, ArmCh2, ArmCh3;

//Controller behavior selection: Choose between *angle* control and *rate* control 
//#define RATE_ROLL_CONTROL //Not implemented
//#define RATE_PITCH_CONTROL //Not implemented
#define RATE_YAW_CONTROL //Comment this out to control yaw angle instead
extern const double RollRate, PitchRate, YawRate;

/*---Estimator---*/

//Estimator gains:
extern const double KpXe, KpYe, KpZe; //For the complementary filter
extern const double KiXe, KiYe, KiZe; //For gyro-bias estimation

//Gyroscope LP-filter time constant. Note that this is done on the RP2350, not the IMU
extern const double Lambda;

#endif
