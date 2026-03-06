#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "headers/logging.h"
#include "headers/estimator.h"
#include "headers/receiver.h"
#include "headers/linalg.h"
#include "headers/servo.h"
#include "headers/config.h"
#include <stdlib.h>

#define PI 3.14159265

typedef struct{
    int N;
    //Matrices:
    double rot_mat_d[3][3]; //Desired matrix
    double rot_mat_d_transposed[3][3]; //Desired matrix transposed
    double e_mat_hat[3][3]; //Estimate of error matrix
    double inertia_mat_hat[3][3]; //Estimate of inertia matrix
    double kp_mat[3][3]; //P-gain
    double kd_mat[3][3]; //D-gain
    double ki_mat[3][3]; //I-gain

    //Vectors:
    double e_hat[3]; //Estimate of error
    double e_w_hat[3]; //Estimate of rate error. Yaw will be rate controlled
    double e_i_hat[3]; //Integral of e_hat
    double e_i_windup[3]; //Anti wind-up limits
    double tau[3]; //Control input
    double kp_e_hat[3];  //Kp*e_hat
    double kd_w_hat[3];  //Kd*w_hat
    double euler[3];

    //Scalars:
    double throttle;  //Throttle
    double arm_timer; //Counting how long receiver inputs have been in startup position
    double arm_threshold; //When this is reached, arm the controller

    //Motor forces tricopter:
    double force_left; //Force left
    double force_right; //Force right
    double force_back; //Force back
    double force_turn; //Force "turn"
    double force_aft; //MAgnitude of FB and FT

    //Controller inputs for tricopter:
    double d_force_left; //Delta force left
    double d_force_right; //Delta force right
    double d_force_back; //Delta force back
    double d_force_turn; //Delta force "turn", is equal to FT
    double alpha;  //Angle alpha

    //Motor forces quadcopter:
    double force_0; //Front right
    double force_1; //Front left
    double force_2; //Back left
    double force_3; //Back right

    //Controller inputs for quadcopter:
    double d_force_0; //Delta front right
    double d_force_1; //Delta front left
    double d_force_2; //Delta back left
    double d_force_3; //Delta back right

    //Flags & arming
    bool run_integrators;
    bool arm_controller;
    uint32_t arm_limits[4];
    uint32_t led_builtin;

    //Throttle thresholds used for truncating duty cycle and safe power off when throttle at lowest position
    double throttle_max;
    double throttle_min;
    double throttle_cutoff;

    //Controller behavior
    double roll_rate;
    double pitch_rate;
    double yaw_rate;
}contStruct;

extern contStruct controller_data;

void controller_reset_integrated_setpoints(contStruct* cont_data);

void controller_init(contStruct* cont_data);

void controller_get_e_hat(contStruct* cont_data, estStruct* est_data);

void controller_get_e_hat_integral(contStruct* cont_data, estStruct* est_data, double h);

void controller_get_tau(contStruct* cont_data, estStruct* est_data);

void controller_control_alloc_tricopter(contStruct* cont_data);

void controller_control_alloc_quadcopter(contStruct* cont_data);

void controller_run_tricopter(contStruct* cont_data, recStruct* rec_data, estStruct* est_data);

void controller_run_quadcopter(contStruct* cont_data, recStruct* rec_data, estStruct* est_data, double h);

void controller_check_for_arming(contStruct* cont_data, recStruct* rec_data, double h);

double controller_force_to_duty(double F); //Converts force to PWM value

double controller_angle_deg_to_duty(double a); //Converts angle to PWM value

void controller_print_forces_PWM_tricopter(const contStruct* cont_data, const char* type);

void controller_print_forces_PWM_quadcopter(const contStruct* cont_data);

#endif
