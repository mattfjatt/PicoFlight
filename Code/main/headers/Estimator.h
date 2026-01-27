#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "headers/LinAlg.h"
#include "headers/MPU6050.h"
#include "headers/Config.h"
#include "pico/stdlib.h"


typedef struct{
    //Estimates of orientation and bias and w:
    float R_hat[3][3];
    float R_hat_T[3][3];
    float b_hat[3];
    float w_hat[3];
    float w_hat_f[3]; //Filtered w_hat
    float b_hat_dot[3];
    float db_hat[3];

    //Reference vectors and skew symmetric forms:
    float v1_hat[3], v2_hat[3];
    float v1[3], v2[3];
    float v1_x[3][3], v2_x[3][3];

    //Correction matrix and bias for the magnetometer:
    float M[3][3]; 
    float B[3];
    float m_corr[3];

    //Raw IMU readings:
    float w[3];
    float a[3];
    float m[3];

    //Unit vectors and their negated versions, may be useful:
    float i1[3], i2[3], i3[3], ni1[3], ni2[3], ni3[3];

    //Unit vector for v2_hat:
    float u1[3];

    //Matrices to store skew-symmetric outputs:
    float w_hat_X[3][3];
    float Kpe_c[3];
    float Kpe_c_X[3][3];

    //Correction term from reference vectors:
    float c[3];

    //Terms used to find dR:
    float S[3][3];
    float Sh[3][3]; 
    float dR[3][3];

    //Estimator gains:
    float Kpe[3][3];
    float Kie[3][3];
    float k1, k2;

    //IMU gyro LP filter time constant
    float Lambda;
}estStruct;

extern estStruct estimatorData;

void Estimator_R_to_euler(float R[][3], float T[3]); //R: input matrix, T: vector of euler angles: T =[phi, theta, psi]
void Estimator_euler_to_R(float T[3], float R[][3]); //T input euler angles, R: Output matrix
void Estimator_R_next(float R[][3],float w[3], float h); //Update R
void Estimator_vecLP(float y[3], float x[3], float k); //AR1-low pass filter for a 3x1 vector
void Estimator_scalLP(float* y, float* x, float k); //Low pass filter for a scalar
void Estimator_init(estStruct* estData);
void Estimator_estimate_R(estStruct* estData, float h);
// void Estimator_find_current_mag_direction(estStruct* estData); //Find the local direction of the magnetic field
// void Estimator_mag_correction(estStruct* estData);

#endif
