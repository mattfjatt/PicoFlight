#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "headers/logging.h"
#include "headers/linalg.h"
#include "headers/mpu6050.h"
#include "headers/mmc5603.h"
#include "headers/icm20948.h"
#include "headers/icm45686.h"
#include "headers/config.h"
#include "pico/stdlib.h"


typedef struct{
    int n;
    //Estimates of orientation and bias and w:
    double rot_mat_hat[3][3];
    double rot_mat_hat_transposed[3][3];
    double b_hat[3];
    double w_hat[3];
    double w_hat_f[3]; //Filtered w_hat
    double b_hat_dot[3];
    double db_hat[3];

    //Reference vectors and skew symmetric forms:
    double v1_hat[3], v2_hat[3], v1_x_v1_hat[3], v2_x_v2_hat[3];
    double v1[3], v2[3];
    double v1_x[3][3], v2_x[3][3];

    //Raw readings:
    double w[3];
    double a[3];
    double m[3];

    //Unit vectors and their negated versions, may be useful:
    double i1[3], i2[3], i3[3], ni1[3], ni2[3], ni3[3];

    //Unit vector for v2_hat:
    double u1[3];

    //Matrices to store skew-symmetric outputs:
    double w_hat_x[3][3];
    double kpe_c[3];
    double kpe_c_x[3][3];

    //Correction term from reference vectors:
    double c[3];

    //Terms used to find dR:
    double skew_mat[3][3];
    double skew_mat_h[3][3]; 
    double d_rot_mat_hat[3][3];

    //Estimator gains:
    double kp_e[3][3];
    double ki_e[3][3];
    double k1, k2;

    //IMU gyro LP filter time constant
    double lambda;
}estStruct;

extern estStruct estimator_data;

void estimator_rot_mat_to_euler(double rot_mat[][3], double euler[3]); //R: input matrix, T: vector of euler angles: T =[phi, theta, psi]

void estimator_euler_to_rot_mat(double euler[3], double rot_mat[][3]); //T input euler angles, R: Output matrix

void estimator_rot_mat_next(double rot_mat[][3],double w[3], double h); //Update R

void estimator_vec_low_pass(int n, double y[n], double x[n], double k); //AR1-low pass filter for an Nx1 vector

void estimator_scal_low_pass(double* y, double* x, double k); //Low pass filter for a scalar

void estimator_init(estStruct* est_data);

void estimator_estimate_attitude(estStruct* est_data, double h);

void estimator_find_current_mag_direction(estStruct* est_data); //Find the local direction of the magnetic field

void estimator_set_initial_gyro_bias(estStruct* est_data);

void estimator_get_imu_data(estStruct* est_data);

#endif
