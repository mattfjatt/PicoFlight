#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "headers/Logging.h"
#include "headers/Estimator.h"
#include "headers/Receiver.h"
#include "headers/LinAlg.h"
#include "headers/Servo.h"
#include "headers/Config.h"
#include <stdlib.h>

#define PI 3.14159265F

typedef struct{
    int N;
    //Matrices:
    double Rd[3][3]; //Desired matrix
    double Rd_T[3][3]; //Desired matrix transposed
    double E_hat[3][3]; //Estimate of error matrix
    double I_hat[3][3]; //Estimate of inertia matrix
    double Kp[3][3]; //P-gain
    double Kd[3][3]; //D-gain
    double Ki[3][3]; //I-gain

    //Vectors:
    double e_hat[3]; //Estimate of error
    double e_w_hat[3]; //Estimate of rate error. Yaw will be rate controlled
    double e_i_hat[3]; //Integral of e_hat
    double e_i_wind_up[3]; //Anti wind-up limits
    double tau[3]; //Control input
    double Kp_e_hat[3];  //Kp*e_hat
    double Kd_w_hat[3];  //Kd*w_hat
    double euler[3];

    //Scalars:
    double T;  //Throttle
    double ArmTimer; //Counting how long receiver inputs have been in startup position
    double ArmThreshold;//When this is reached, arm the controller

    //Motor forces tricopter:
    double FL; //Force left
    double FR; //Force right
    double FB; //Force back
    double FT; //Force "turn"
    double FA; //MAgnitude of FB and FT

    //Controller inputs for tricopter:
    double dFL; //Delta force left
    double dFR; //Delta force right
    double dFB; //Delta force back
    double dFT; //Delta force "turn", is equal to FT
    double a;  //Angle alpha

    //Motor forces quadcopter:
    double F0; //Front right
    double F1; //Front left
    double F2; //Back left
    double F3; //Back right

    //Controller inputs for quadcopter:
    double dF0; //Delta front right
    double dF1; //Delta front left
    double dF2; //Delta back left
    double dF3; //Delta back right

    //Flags & arming
    bool RunIntegrators;
    bool ArmController;
    uint32_t ArmLimits[4];
    uint32_t LedBuiltin;

    //Throttle thresholds used for truncating duty cycle and safe power off when throttle at lowest position
    double TMax;
    double TMin;
    double TCutoff;

    //Controller behavior
    double RollRate;
    double PitchRate;
    double YawRate;
}contStruct;

extern contStruct controllerData;

void Controller_reset_integrated_setpoints(contStruct* contData);

void Controller_init(contStruct* contData);

void Controller_get_e_hat(contStruct* contData, estStruct* estData);

void Controller_get_e_hat_integral(contStruct* contData, estStruct* estData, double h);

void Controller_get_tau(contStruct* contData, estStruct* estData);

void Controller_control_alloc_tricopter(contStruct* contData);

void Controller_control_alloc_quadcopter(contStruct* contData);

void Controller_run_tricopter(contStruct* contData, recStruct* recData, estStruct* estData);

void Controller_run_quadcopter(contStruct* contData, recStruct* recData, estStruct* estData, double h);

void Controller_check_for_arming(contStruct* contData, recStruct* recData, double h);

double Controller_force_to_duty(double F); //Converts force to PWM value

double Controller_angle_deg_to_duty(double a); //Converts angle to PWM value

void Controller_print_forces_PWM_tricopter(const contStruct* contData, const char* type);

void Controller_print_forces_PWM_quadcopter(const contStruct* contData);

#endif
