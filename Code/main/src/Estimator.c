#include "headers/Estimator.h"

estStruct estimatorData;

void Estimator_R_to_euler(double R[][3], double T[3]){
    int N = 3;
    LinAlg_zerovec(N,T);
    T[0] = -atan2(R[1][2],R[2][2]);
    T[1] =  asin(R[0][2]);
    T[2] = -atan2(R[0][1],R[0][0]);
}

void Estimator_euler_to_R(double T[3], double R[][3]){
    int N = 3;
    double Rx[3][3];
    double Ry[3][3];
    double Rz[3][3];
    LinAlg_rotX(T[0],Rx);
    LinAlg_rotY(T[1],Ry);
    LinAlg_rotZ(T[2],Rz);
    LinAlg_matmatmul_small(N,N,Rx,Ry,R);
    LinAlg_matmatmul_small(N,N,R,Rz,R);
}

void Estimator_R_next(double R[][3],double w[3],double h){
    int N = 3;
    double hwx[3][3];
    double dR[3][3];
    LinAlg_zeromat(N,N,dR);
    LinAlg_vec2skew3x3(w,hwx);
    LinAlg_matscalmult(N,N,hwx,h,hwx);
    LinAlg_expm3x3(hwx,dR,9);
    LinAlg_matmatmul_small(N,N,R,dR,R);
}

void Estimator_vecLP(int N, double y[N], double x[N], double k){
    for(int i = 0; i < N; i++){
        y[i] = (1-k)*y[i] + k*x[i]; 
    }
}

void Estimator_scalLP(double* y, double* x, double k){
    *y = (1-k)*(*y) + k*(*x); 
}

void Estimator_init(estStruct* estData){
    estData->N = 3;
    LinAlg_eye(estData->N,estData->R_hat);
    LinAlg_eye(estData->N,estData->R_hat_T);
    LinAlg_eye(estData->N,estData->Kie);
    LinAlg_zeromat(estData->N,estData->N,estData->v1_x);
    LinAlg_zeromat(estData->N,estData->N,estData->v2_x);
    LinAlg_zeromat(estData->N,estData->N,estData->dR);
    LinAlg_zeromat(estData->N,estData->N,estData->Kpe);
    LinAlg_zeromat(estData->N,estData->N,estData->w_hat_X);
    LinAlg_zeromat(estData->N,estData->N,estData->Kpe_c_X);
    LinAlg_zerovec(estData->N,estData->b_hat);
    LinAlg_zerovec(estData->N,estData->w_hat);
    LinAlg_zerovec(estData->N,estData->c);
    LinAlg_zerovec(estData->N,estData->b_hat_dot);
    LinAlg_zerovec(estData->N,estData->db_hat);
    LinAlg_zeromat(estData->N,estData->N,estData->S);
    LinAlg_zeromat(estData->N,estData->N,estData->Sh);
    LinAlg_zerovec(estData->N,estData->v1);
    LinAlg_zerovec(estData->N,estData->v2);
    LinAlg_zerovec(estData->N,estData->v1_hat);
    LinAlg_zerovec(estData->N,estData->v2_hat);
    LinAlg_zerovec(estData->N,estData->Kpe_c);
    LinAlg_zerovec(estData->N,estData->w);
    LinAlg_zerovec(estData->N,estData->a);
    LinAlg_zerovec(estData->N,estData->m);
    LinAlg_zerovec(estData->N,estData->w_hat_f);

    estData->k1 = 1.0; //For IMU
    estData->k2 = 1.0; //For magnetometer

    //Correction term for bias estimation/integral term(they are supposed to be negative, check the paper):
    estData->Kie[0][0] = - KiXe;
    estData->Kie[1][1] = - KiYe;
    estData->Kie[2][2] = - KiZe;
    //Complementary part/proportional term:
    estData->Kpe[0][0] = KpXe;
    estData->Kpe[1][1] = KpYe;
    estData->Kpe[2][2] = KpZe;

    double ident[3][3];
    LinAlg_eye(estData->N,ident);
    LinAlg_mat2colvecs3x3(ident, estData->i1, estData->i2, estData->i3);
    LinAlg_matscalmult(estData->N,estData->N,ident, -1.0, ident);
    LinAlg_mat2colvecs3x3(ident, estData->ni1, estData->ni2, estData->ni3);
    LinAlg_zerovec(estData->N,estData->u1);

    //IMU gyro LP filter time constant
    estData->Lambda = Lambda;

    //Find in which direction the magnetic fields point at startup, this will serve as reference heading.
    //To get actual north, a look up table based on location is needed
    Estimator_find_current_mag_direction(estData);
};

void Estimator_estimate_R(estStruct* estData,double h){
    MPU6050_get_imu_data(estData->a, estData->w);
    MMC5603_get_corrected_mag_reading(estData->m);

    //Define the reference vectors v1 and v2, and ensure no division by 0

    //v1:
    if(fabs(LinAlg_vecnorm(estData->N, estData->a)) < 0.01){
        LinAlg_zerovec(estData->N, estData->v1);
    }else{
        LinAlg_normalize(estData->N,estData->a,estData->v1);
    }

    //v2
    if(fabs(LinAlg_vecnorm(estData->N, estData->m)) < 0.01){
        LinAlg_zerovec(estData->N, estData->v2);
    }else{
        LinAlg_normalize(estData->N,estData->m,estData->v2);
    }
    
    
    LinAlg_mattranspose(estData->N,estData->N,estData->R_hat, estData->R_hat_T);
    LinAlg_matvecmul(estData->N,estData->N,estData->R_hat_T, estData->ni3, estData->v1_hat);
    LinAlg_matvecmul(estData->N,estData->N,estData->R_hat_T, estData->u1, estData->v2_hat);
    LinAlg_vec2skew3x3(estData->v1, estData->v1_x);
    LinAlg_vec2skew3x3(estData->v2, estData->v2_x);


    //c = k1*S(v1)*v1_hat + k2*S(v2)*v2_hat
    LinAlg_matscalmult(estData->N,estData->N,estData->v1_x, estData->k1, estData->v1_x);
    LinAlg_matscalmult(estData->N,estData->N,estData->v2_x, estData->k2, estData->v2_x);
    LinAlg_matvecmul(estData->N,estData->N,estData->v1_x, estData->v1_hat, estData->v1_hat);
    LinAlg_matvecmul(estData->N,estData->N,estData->v2_x, estData->v2_hat, estData->v2_hat);
    LinAlg_vecvecadd(estData->N,estData->v1_hat, estData->v2_hat, estData->c);
    

    LinAlg_matvecmul(estData->N,estData->N,estData->Kie, estData->c, estData->b_hat_dot); //b_hat_dot = Kie*c
    LinAlg_vecscalmult(estData->N,estData->b_hat_dot, estData->db_hat, h); //db_hat <- b_hat_dot*h
    LinAlg_vecvecadd(estData->N,estData->b_hat, estData->db_hat, estData->b_hat);
    //LinAlg_printvec(estData->w_hat);

    //Correction term in R_hat_dot:
    LinAlg_matvecmul(estData->N,estData->N,estData->Kpe, estData->c, estData->Kpe_c);
    LinAlg_vec2skew3x3(estData->Kpe_c, estData->Kpe_c_X); //Kpe_c_X = S(Kpe*c)
    LinAlg_vecvecsub(estData->N,estData->w, estData->b_hat, estData->w_hat); //w_hat = w - b_hat
    //LinAlg_zerovec(estData->N, estData->w_hat); //THIS IS SET TO ZERO TO GAUGE EFFECT OF REFERENCE VECTORS IN ISOLATION
    LinAlg_vec2skew3x3(estData->w_hat, estData->w_hat_X); //w_hat_X = S(w_hat)
    LinAlg_matmatadd(estData->N,estData->N,estData->w_hat_X, estData->Kpe_c_X, estData->S); //S = S(w_hat) + S(Kpe*c)
    LinAlg_matscalmult(estData->N,estData->N,estData->S,h,estData->Sh); // Sh = S*h
    LinAlg_expm3x3(estData->Sh, estData->dR,9); //expm(Sh) = dR
    LinAlg_matmatmul_small(estData->N,estData->N,estData->R_hat, estData->dR, estData->R_hat); //R_hat <- R_hat*dR
    LinAlg_matnormalizerotation(estData->R_hat); //Ensure R_hat remains in SO(3)

    //Also do a lowpass of w_hat
    Estimator_vecLP(estData->N,estData->w_hat_f, estData->w_hat, h / estData->Lambda);
}


void Estimator_find_current_mag_direction(estStruct* estData){
    //This function should only run when the magnetometer is stationary at roll = pitch = 0
    //How to stop it from running before this? Look at size of w?
    int is_stationary = 0;
    double w_threshold = 0.15;
    uint32_t start = 0;
    uint32_t duration = 0;
    uint32_t threshold = 3000000; //3 seconds
    while(!is_stationary){
        start = time_us_32();
        sleep_ms(20);
        MPU6050_get_imu_data(estData->a, estData->w);
        if(fabs(LinAlg_vecnorm(estData->N, estData->w)) > w_threshold){
            duration = 0;
            PRINT("MOVING\n");
        }else{
            duration += time_us_32() - start;
            PRINT("STATIONARY\n");
        }
        if(duration >= threshold){
            is_stationary = 1;
        }
    }
    PRINT("Starting tuning...\n");

    int N = 3;
    double fmagread[3];
    LinAlg_zerovec(N, fmagread);
    for(int i = 0; i < 200; i++){
        MMC5603_get_corrected_mag_reading(estData->m);
        LinAlg_normalize(N,estData->m,estData->m);
        Estimator_vecLP(N,fmagread,estData->m,0.05);
        sleep_ms(20); //Delay of 2ms may be too short
        //printvec(fmagread);
    }
    //Setting the filtered, normalized, and corrected magnetometer reading equal to the unit u1 vector used to find v2_hat:
    LinAlg_veccopy(N,fmagread, estData->u1);
}
