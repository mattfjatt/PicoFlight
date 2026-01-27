#include "headers/Estimator.h"

estStruct estimatorData;

void Estimator_R_to_euler(float R[][3], float T[3]){
    LinAlg_zerovec(T);
    T[0] = -atan2(R[1][2],R[2][2]);
    T[1] =  asin(R[0][2]);
    T[2] = -atan2(R[0][1],R[0][0]);
}

void Estimator_euler_to_R(float T[3], float R[][3]){
    float Rx[3][3];
    float Ry[3][3];
    float Rz[3][3];
    LinAlg_rotX(T[0],Rx);
    LinAlg_rotY(T[1],Ry);
    LinAlg_rotZ(T[2],Rz);
    LinAlg_matmatmul(Rx,Ry,R);
    LinAlg_matmatmul(R,Rz,R);
}

void Estimator_R_next(float R[][3],float w[3],float h){
    float hwx[3][3];
    float dR[3][3];
    LinAlg_zeromat(dR);
    LinAlg_vec2skew(w,hwx);
    LinAlg_matscalmult(hwx,h,hwx);
    LinAlg_expm(hwx,dR,9);
    LinAlg_matmatmul(R,dR,R);
}

void Estimator_vecLP(float y[3], float x[3], float k){
    for(int i = 0; i < 3; i++){
        y[i] = (1-k)*y[i] + k*x[i]; 
    }
}

void Estimator_scalLP(float* y, float* x, float k){
    *y = (1-k)*(*y) + k*(*x); 
}

void Estimator_init(estStruct* estData){
    LinAlg_eye(estData->R_hat);
    LinAlg_eye(estData->R_hat_T);
    LinAlg_eye(estData->Kie);
    LinAlg_zeromat(estData->v1_x);
    LinAlg_zeromat(estData->v2_x);
    LinAlg_zeromat(estData->dR);
    LinAlg_zeromat(estData->Kpe);
    LinAlg_zeromat(estData->w_hat_X);
    LinAlg_zeromat(estData->Kpe_c_X);
    LinAlg_zerovec(estData->b_hat);
    LinAlg_zerovec(estData->w_hat);
    LinAlg_zerovec(estData->c);
    LinAlg_zerovec(estData->b_hat_dot);
    LinAlg_zerovec(estData->db_hat);
    LinAlg_zeromat(estData->S);
    LinAlg_zeromat(estData->Sh);
    estData->k1 = 0.0;
    estData->k2 = 0.0;
    LinAlg_zerovec(estData->v1);
    LinAlg_zerovec(estData->v2);
    LinAlg_zerovec(estData->v1_hat);
    LinAlg_zerovec(estData->v2_hat);
    LinAlg_zerovec(estData->Kpe_c);
    LinAlg_zerovec(estData->B);
    LinAlg_zeromat(estData->M);
    LinAlg_zerovec(estData->w);
    LinAlg_zerovec(estData->a);
    LinAlg_zerovec(estData->m);
    LinAlg_zerovec(estData->m_corr);
    LinAlg_zerovec(estData->w_hat_f);

    float ident[3][3];
    LinAlg_eye(ident);
    LinAlg_mat2colvecs(ident, estData->i1, estData->i2, estData->i3);
    LinAlg_matscalmult(ident, -1.0, ident);
    LinAlg_mat2colvecs(ident, estData->ni1, estData->ni2, estData->ni3);
    LinAlg_zerovec(estData->u1);
    //Can warm-start the b_hat vector here, but the numbers should be checked regularly and updated if needed
    estData->b_hat[2] = 0.25f*3.14159f/180.f;

    //IMU gyro LP filter time constant
    estData->Lambda = Lambda;
};

void Estimator_estimate_R(estStruct* estData,float h){
    MPU6050_get_imu_data(estData->a, estData->w);
    //mag_correction(estData); //Correct the magnetic reading

    //Define the reference vectors:
    LinAlg_normalize(estData->a,estData->v1);
    //Quite a large error on the y-axis of the accel, doing a quick and dirty subtraction:
    estData->v1[0] = estData->v1[0] + 0.01115f;
    estData->v1[1] = estData->v1[1] - 0.06315f;
    //normalize(estData->m_corr,estData->v2); //This gives NaN if norm(m_corr) = 0;
    LinAlg_transpose(estData->R_hat, estData->R_hat_T);
    LinAlg_matvecmul(estData->R_hat_T, estData->ni3, estData->v1_hat);
    //matvecmul(estData->R_hat_T, estData->u1, estData->v2_hat);
    LinAlg_vec2skew(estData->v1, estData->v1_x);
    //vec2skew(estData->v2, estData->v2_x);

    //float err[3];
    //vecvecsub(estData->v2_hat,estData->v2,err); //Err should be close to 0 for properly calibrated magnetometer
    //printvec(err);

    //c = k1*S(v1)*v1_hat + k2*S(v2)*v2_hat
    estData->k1 = 1.0000;
    estData->k2 = 0.0000; //Don't use if magnetic correction parameters are outdated
    LinAlg_matscalmult(estData->v1_x, estData->k1, estData->v1_x);
    //matscalmult(estData->v2_x, estData->k2, estData->v2_x);
    LinAlg_matvecmul(estData->v1_x, estData->v1_hat, estData->v1_hat);
    //matvecmul(estData->v2_x, estData->v2_hat, estData->v2_hat);
    LinAlg_vecvecadd(estData->v1_hat, estData->v2_hat, estData->c);
    

    //Correction term for bias estimation/integral term(they are supposed to be negative, check the paper):
    estData->Kie[0][0] = - KiXe;
    estData->Kie[1][1] = - KiYe;
    estData->Kie[2][2] = - KiZe;
    //Complementary part/proportional term:
    estData->Kpe[0][0] = KpXe;
    estData->Kpe[1][1] = KpYe;
    estData->Kpe[2][2] = KpZe;

    LinAlg_matvecmul(estData->Kie, estData->c, estData->b_hat_dot); //b_hat_dot = Kie*c
    LinAlg_vecscalmult(estData->b_hat_dot, estData->db_hat, h); //db_hat <- b_hat_dot*h
    LinAlg_vecvecadd(estData->b_hat, estData->db_hat, estData->b_hat);
    //printvec(estData->w_hat);

    //Correction term in R_hat_dot:
    LinAlg_matvecmul(estData->Kpe, estData->c, estData->Kpe_c);
    LinAlg_vec2skew(estData->Kpe_c, estData->Kpe_c_X); //Kpe_c_X = S(Kpe*c)
    LinAlg_vecvecsub(estData->w, estData->b_hat, estData->w_hat); //w_hat = w - b_hat
    LinAlg_vec2skew(estData->w_hat, estData->w_hat_X); //w_hat_X = S(w_hat)
    LinAlg_matmatadd(estData->w_hat_X, estData->Kpe_c_X, estData->S); //S = S(w_hat) + S(Kpe*c)
    LinAlg_matscalmult(estData->S,h,estData->Sh); // Sh = S*h
    LinAlg_expm(estData->Sh, estData->dR,9); //expm(Sh) = dR
    LinAlg_matmatmul(estData->R_hat, estData->dR, estData->R_hat); //R_hat <- R_hat*dR
    LinAlg_matnormalizerotation(estData->R_hat); //Ensure R_hat remains in SO(3)

    //Also do a a lowpass of w_hat
    Estimator_vecLP(estData->w_hat_f, estData->w_hat, h / estData->Lambda);
}


//Mag-sensor not available on MPU 6050, hence these functions are ignored

//void find_current_mag_direction(estStruct* estData){
//     float fmagread[3];
//     zerovec(fmagread);
//     for(int i = 0; i < 200; i++){
//         get_raw_mag_data(estData->m);
//         mag_correction(estData); //Correct for mag biases
//         normalize(estData->m_corr,estData->m_corr);
//         vecLP(fmagread,estData->m_corr,0.05);
//         sleep_ms(20); //Delay of 2ms may be too short
//         //printvec(fmagread);
//     }
//     //Setting the filtered, normalized, and corrected magnetometer reading equal to the unit u1 vector used to find v2_hat:
//     veccopy(fmagread, estData->u1);
// }

// void mag_correction(estStruct* estData){
//     /*
//     0.9887    0.0148    0.0115
//     0.0148    1.0122    0.0390
//     0.0115    0.0390    1.0011
//     */
//     estData->M[0][0] = 0.9887;
//     estData->M[0][1] = 0.0148;
//     estData->M[0][2] = 0.0115;

//     estData->M[1][0] = 0.0148;
//     estData->M[1][1] = 1.0122;
//     estData->M[1][2] = 0.0390;

//     estData->M[2][0] = 0.0115;
//     estData->M[2][1] = 0.0390;
//     estData->M[2][2] = 1.0011;
//     /*
//     35.0389   74.2616  -25.0450
//     */
//     estData->B[0] = 35.0389;
//     estData->B[1] = 74.2616;
//     estData->B[2] = -25.0450;

//     vecvecsub(estData->m, estData->B, estData->m_corr);
//     matvecmul(estData->M, estData->m_corr, estData->m_corr);
// }
