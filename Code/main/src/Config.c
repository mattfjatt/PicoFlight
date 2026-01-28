#include "headers/Config.h"

/*---Controller---*/

//Controller gains:
const double KpX = 1500.0;
const double KpY = 1500.0;
const double KpZ = 1500.0;

const double KdX = 150.0;
const double KdY = 150.0;
const double KdZ = 0.0;

const double KiX = 0.0; //450.0;
const double KiY = 0.0; //450.0;
const double KiZ = 0.0;

//Integrator wind-up limits:
const double WindUpX = 100.0;
const double WindupY = 100.0;
const double WindUpZ = 100.0;

//Throttle limits:
const double TMax = 1920.0;
const double TMin = 1050.0;
const double TCutoff = 980.0;
const double TPowerOff = 0.0;

//Arm threshold for the four radio channels
const uint32_t ArmCh0 = 1200; //Yaw
const uint32_t ArmCh1 = 1000; //Throttle
const uint32_t ArmCh2 = 1200; //Pitch
const uint32_t ArmCh3 = 1200; //Roll

//Controller behavior selection
const double RollRate = 1.0; //Not implemented
const double PitchRate = 1.0; //Not implemented
const double YawRate = 1.0;



/*---Estimator---*/

//For the complementary filter
const double KpXe = 0.5;
const double KpYe = 0.5; 
const double KpZe = 0.0; //Set to 0 until magnetometer is introduced

//For gyro-bias estimation
const double KiXe = 0.15;
const double KiYe = 0.15; 
const double KiZe = 0.0f; //Set to 0 until magnetometer is introduced

//Gyroscope LP-filter time constant
const double Lambda = 0.0107;
