#include "headers/config.h"

/*---Controller---*/

//Controller gains:
const double kp_x = 1500.0;
const double kp_y = 1500.0;
const double kp_z = 1500.0;

const double kd_x = 150.0;
const double kd_y = 150.0;
const double kd_z = 0.0;

const double ki_x = 0.0; //450.0;
const double ki_y = 0.0; //450.0;
const double ki_z = 0.0;

//Integrator wind-up limits:
const double windup_x = 100.0;
const double windup_y = 100.0;
const double windup_z = 100.0;

//Throttle limits:
const double throttle_max = 1920.0;
const double throttle_min = 1050.0;
const double throttle_cutoff = 980.0;
const double throttle_poweroff = 0.0;

//Arm threshold for the four radio channels
const uint32_t arm_ch0 = 1200; //Yaw
const uint32_t arm_ch1 = 1000; //Throttle
const uint32_t arm_ch2 = 1200; //Pitch
const uint32_t arm_ch3 = 1200; //Roll

//Controller behavior selection
const double roll_rate = 1.0; //Not implemented
const double pitch_rate = 1.0; //Not implemented
const double yaw_rate = 1.0;



/*---Estimator---*/

//For the complementary filter
const double kp_xe = 0.5;
const double kp_ye = 0.5; 
const double kp_ze = 0.5; //Set to 0 until magnetometer is introduced

//For gyro-bias estimation
const double ki_xe = 0.15;
const double ki_ye = 0.15; 
const double ki_ze = 0.15; //Set to 0 until magnetometer is introduced

//Gyroscope LP-filter time constant
const double lambda = 0.0107;
