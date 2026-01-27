#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "headers/Estimator.h"
#include "headers/Receiver.h"
#include "headers/LinAlg.h"
#include "headers/Servo.h"
#include "headers/Config.h"
#include <stdlib.h>

#define PI 3.14159265F

typedef struct{
    //Matrices:
    float Rd[3][3]; //Desired matrix
    float Rd_T[3][3]; //Desired matrix transposed
    float E_hat[3][3]; //Estimate of error matrix
    float I_hat[3][3]; //Estimate of inertia matrix
    float Kp[3][3]; //P-gain
    float Kd[3][3]; //D-gain
    float Ki[3][3]; //I-gain

    //Vectors:
    float e_hat[3]; //Estimate of error
    float e_w_hat[3]; //Estimate of rate error. Yaw will be rate controlled
    float e_i_hat[3]; //Integral of e_hat
    float e_i_wind_up[3]; //Anti wind-up limits
    float tau[3]; //Control input
    float Kp_e_hat[3];  //Kp*e_hat
    float Kd_w_hat[3];  //Kd*w_hat
    float euler[3];

    //Scalars:
    float T;  //Throttle
    float ArmTimer; //Counting how long receiver inputs have been in startup position
    float ArmThreshold;//When this is reached, arm the controller

    //Motor forces tricopter:
    float FL; //Force left
    float FR; //Force right
    float FB; //Force back
    float FT; //Force "turn"
    float FA; //MAgnitude of FB and FT

    //Controller inputs for tricopter:
    float dFL; //Delta force left
    float dFR; //Delta force right
    float dFB; //Delta force back
    float dFT; //Delta force "turn", is equal to FT
    float a;  //Angle alpha

    //Motor forces quadcopter:
    float F0; //Front right
    float F1; //Front left
    float F2; //Back left
    float F3; //Back right

    //Controller inputs for quadcopter:
    float dF0; //Delta front right
    float dF1; //Delta front left
    float dF2; //Delta back left
    float dF3; //Delta back right

    //Flags & arming
    bool RunIntegrators;
    bool ArmController;
    uint32_t ArmLimits[4];
    uint32_t LedBuiltin;

    //Throttle thresholds used for truncating duty cycle and safe power off when throttle at lowest position
    float TMax;
    float TMin;
    float TCutoff;

    //Controller behavior
    float RollRate;
    float PitchRate;
    float YawRate;
}contStruct;

extern contStruct controllerData;

void Controller_reset_integrated_setpoints(contStruct* contData);

void Controller_init(contStruct* contData);

void Controller_get_e_hat(contStruct* contData, estStruct* estData);

void Controller_get_e_hat_integral(contStruct* contData, estStruct* estData, float h);

void Controller_get_tau(contStruct* contData, estStruct* estData);

void Controller_control_alloc_tricopter(contStruct* contData);

void Controller_control_alloc_quadcopter(contStruct* contData);

void Controller_run_tricopter(contStruct* contData, recStruct* recData, estStruct* estData);

void Controller_run_quadcopter(contStruct* contData, recStruct* recData, estStruct* estData, float h);

void Controller_check_for_arming(contStruct* contData, recStruct* recData, float h);

float Controller_force_to_duty(float F); //Converts force to PWM value

float Controller_angle_deg_to_duty(float a); //Converts angle to PWM value

void Controller_print_forces_PWM_tricopter(const contStruct* contData, const char* type);

void Controller_print_forces_PWM_quadcopter(const contStruct* contData);

#endif
