#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "headers/LinAlg.h"
#include "headers/MPU6050.h"
#include "headers/Config.h"
#include "pico/stdlib.h"


typedef struct{
    int N;
    //Estimates of orientation and bias and w:
    double R_hat[3][3];
    double R_hat_T[3][3];
    double b_hat[3];
    double w_hat[3];
    double w_hat_f[3]; //Filtered w_hat
    double b_hat_dot[3];
    double db_hat[3];

    //Reference vectors and skew symmetric forms:
    double v1_hat[3], v2_hat[3];
    double v1[3], v2[3];
    double v1_x[3][3], v2_x[3][3];

    //Correction matrix and bias for the magnetometer:
    double M[3][3]; 
    double B[3];
    double m_corr[3];

    //Raw IMU readings:
    double w[3];
    double a[3];
    double m[3];

    //Unit vectors and their negated versions, may be useful:
    double i1[3], i2[3], i3[3], ni1[3], ni2[3], ni3[3];

    //Unit vector for v2_hat:
    double u1[3];

    //Matrices to store skew-symmetric outputs:
    double w_hat_X[3][3];
    double Kpe_c[3];
    double Kpe_c_X[3][3];

    //Correction term from reference vectors:
    double c[3];

    //Terms used to find dR:
    double S[3][3];
    double Sh[3][3]; 
    double dR[3][3];

    //Estimator gains:
    double Kpe[3][3];
    double Kie[3][3];
    double k1, k2;

    //IMU gyro LP filter time constant
    double Lambda;
}estStruct;

extern estStruct estimatorData;

void Estimator_R_to_euler(double R[][3], double T[3]); //R: input matrix, T: vector of euler angles: T =[phi, theta, psi]

void Estimator_euler_to_R(double T[3], double R[][3]); //T input euler angles, R: Output matrix

void Estimator_R_next(double R[][3],double w[3], double h); //Update R

void Estimator_vecLP(int N, double y[N], double x[N], double k); //AR1-low pass filter for an Nx1 vector

void Estimator_scalLP(double* y, double* x, double k); //Low pass filter for a scalar

void Estimator_init(estStruct* estData);

void Estimator_estimate_R(estStruct* estData, double h);
// void Estimator_find_current_mag_direction(estStruct* estData); //Find the local direction of the magnetic field
// void Estimator_mag_correction(estStruct* estData);

#endif
