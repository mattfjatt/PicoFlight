#include "headers/estimator.h"

estStruct estimator_data;

void estimator_rot_mat_to_euler(double rot_mat[][3], double euler[3]){
    int n = 3;
    linalg_zerovec(n,euler);
    euler[0] = -atan2(rot_mat[1][2],rot_mat[2][2]);
    euler[1] =  asin(rot_mat[0][2]);
    euler[2] = -atan2(rot_mat[0][1],rot_mat[0][0]);
}

void estimator_euler_to_rot_mat(double euler[3], double rot_mat[][3]){
    int n = 3;
    double rot_x[3][3];
    double rot_y[3][3];
    double rot_z[3][3];
    linalg_zeromat(n,n,rot_mat);
    linalg_rotX(euler[0],rot_x);
    linalg_rotY(euler[1],rot_y);
    linalg_rotZ(euler[2],rot_z);
    linalg_matmatmul_small(n,n,rot_x,rot_y,rot_mat);
    linalg_matmatmul_small(n,n,rot_mat,rot_z,rot_mat);
}

void estimator_rot_mat_next(double rot_mat[][3],double w[3],double h){
    int n = 3;
    double hwx[3][3];
    double d_rot_mat[3][3];
    linalg_zeromat(n,n,d_rot_mat);
    linalg_vec2skew3x3(w,hwx);
    linalg_matscalmult(n,n,hwx,h,hwx);
    linalg_expm3x3(hwx,d_rot_mat,9);
    linalg_matmatmul_small(n,n,rot_mat,d_rot_mat,rot_mat);
}

void estimator_vec_low_pass(int n, double y[n], double x[n], double k){
    for(int i = 0; i < n; i++){
        y[i] = (1-k)*y[i] + k*x[i]; 
    }
}

void estimator_scal_low_pass(double* y, double* x, double k){
    *y = (1-k)*(*y) + k*(*x); 
}

void estimator_init(estStruct* est_data){
    est_data->n = 3;
    linalg_eye(est_data->n,est_data->rot_mat_hat);
    linalg_eye(est_data->n,est_data->rot_mat_hat_transposed);
    linalg_eye(est_data->n,est_data->ki_e);
    linalg_zeromat(est_data->n,est_data->n,est_data->v1_x);
    linalg_zeromat(est_data->n,est_data->n,est_data->v2_x);
    linalg_zeromat(est_data->n,est_data->n,est_data->d_rot_mat_hat);
    linalg_zeromat(est_data->n,est_data->n,est_data->kp_e);
    linalg_zeromat(est_data->n,est_data->n,est_data->w_hat_x);
    linalg_zeromat(est_data->n,est_data->n,est_data->kpe_c_x);
    linalg_zerovec(est_data->n,est_data->b_hat);
    linalg_zerovec(est_data->n,est_data->w_hat);
    linalg_zerovec(est_data->n,est_data->c);
    linalg_zerovec(est_data->n,est_data->b_hat_dot);
    linalg_zerovec(est_data->n,est_data->db_hat);
    linalg_zeromat(est_data->n,est_data->n,est_data->skew_mat);
    linalg_zeromat(est_data->n,est_data->n,est_data->skew_mat_h);
    linalg_zerovec(est_data->n,est_data->v1);
    linalg_zerovec(est_data->n,est_data->v2);
    linalg_zerovec(est_data->n,est_data->v1_hat);
    linalg_zerovec(est_data->n,est_data->v2_hat);
    linalg_zerovec(est_data->n,est_data->v1_x_v1_hat);
    linalg_zerovec(est_data->n,est_data->v2_x_v2_hat);
    linalg_zerovec(est_data->n,est_data->kpe_c);
    linalg_zerovec(est_data->n,est_data->w);
    linalg_zerovec(est_data->n,est_data->a);
    linalg_zerovec(est_data->n,est_data->m);
    linalg_zerovec(est_data->n,est_data->w_hat_f);

    est_data->k1 = 1.0; //For accelerometer
    est_data->k2 = 1.0; //For magnetometer

    //Correction term for bias estimation/integral term(they are supposed to be negative, check the paper):
    est_data->ki_e[0][0] = - ki_xe;
    est_data->ki_e[1][1] = - ki_ye;
    est_data->ki_e[2][2] = - ki_ze;
    //Complementary part/proportional term:
    est_data->kp_e[0][0] = kp_xe;
    est_data->kp_e[1][1] = kp_ye;
    est_data->kp_e[2][2] = kp_ze;

    double ident[3][3];
    linalg_eye(est_data->n,ident);
    linalg_mat2colvecs3x3(ident, est_data->i1, est_data->i2, est_data->i3);
    linalg_matscalmult(est_data->n,est_data->n,ident, -1.0, ident);
    linalg_mat2colvecs3x3(ident, est_data->ni1, est_data->ni2, est_data->ni3);
    linalg_zerovec(est_data->n,est_data->u1);

    //IMU gyro LP filter time constant
    est_data->lambda = lambda;

    //Find in which direction the magnetic fields point at startup, this will serve as reference heading.
    //To get actual north, a look up table based on location is needed
    //estimator_find_current_mag_direction(est_data);

    //Warm-starting gyro bias
    estimator_set_initial_gyro_bias(est_data);
};

void estimator_estimate_attitude(estStruct* est_data,double h){
    estimator_get_imu_data(est_data);
    //mmc5603_get_corrected_mag_reading(est_data->m);

    //Define the reference vectors v1 and v2, and ensure no division by 0
    int check_magnitude = 1;
    //v1:
    if((linalg_vecnorm(est_data->n, est_data->a) > 1.15 || linalg_vecnorm(est_data->n, est_data->a) < 0.85) && check_magnitude){ //1.15 and 0.85 can be moved closer to 1.0 when accel is calibrated
        linalg_zerovec(est_data->n, est_data->v1);
        LOG("Magnitude of accel outside limits\n");
    }else{
        linalg_normalize(est_data->n,est_data->a,est_data->v1);
    }

    //v2
    if((linalg_vecnorm(est_data->n, est_data->m) > 1.20 || linalg_vecnorm(est_data->n, est_data->m) < 0.80) && check_magnitude){
        linalg_zerovec(est_data->n, est_data->v2);
        //LOG("Magnitude of mag outside limits\n");
    }else{
        linalg_normalize(est_data->n,est_data->m,est_data->v2);
    }
    
    
    linalg_mattranspose(est_data->n,est_data->n,est_data->rot_mat_hat, est_data->rot_mat_hat_transposed);
    //Create v1_hat
    linalg_matvecmul(est_data->n,est_data->n,est_data->rot_mat_hat_transposed, est_data->ni3, est_data->v1_hat);
    //Create v2_hat
    linalg_matvecmul(est_data->n,est_data->n,est_data->rot_mat_hat_transposed, est_data->u1, est_data->v2_hat);

    //Make the skew-symmetric matrices
    linalg_vec2skew3x3(est_data->v1, est_data->v1_x);
    linalg_vec2skew3x3(est_data->v2, est_data->v2_x);


    //c = k1*S(v1)*v1_hat + k2*S(v2)*v2_hat
    linalg_matscalmult(est_data->n,est_data->n,est_data->v1_x, est_data->k1, est_data->v1_x);
    linalg_matscalmult(est_data->n,est_data->n,est_data->v2_x, est_data->k2, est_data->v2_x);
    linalg_matvecmul(est_data->n,est_data->n,est_data->v1_x, est_data->v1_hat, est_data->v1_x_v1_hat);
    linalg_matvecmul(est_data->n,est_data->n,est_data->v2_x, est_data->v2_hat, est_data->v2_x_v2_hat);
    linalg_vecvecadd(est_data->n,est_data->v1_x_v1_hat, est_data->v2_x_v2_hat, est_data->c);
    

    linalg_matvecmul(est_data->n,est_data->n,est_data->ki_e, est_data->c, est_data->b_hat_dot); //b_hat_dot = Kie*c
    linalg_vecscalmult(est_data->n,est_data->b_hat_dot, est_data->db_hat, h); //db_hat <- b_hat_dot*h
    linalg_vecvecadd(est_data->n,est_data->b_hat, est_data->db_hat, est_data->b_hat);
    //linalg_printvec(est_data->w_hat);

    //Correction term in R_hat_dot:
    linalg_matvecmul(est_data->n,est_data->n,est_data->kp_e, est_data->c, est_data->kpe_c);
    linalg_vec2skew3x3(est_data->kpe_c, est_data->kpe_c_x); //Kpe_c_X = S(Kpe*c)
    linalg_vecvecsub(est_data->n,est_data->w, est_data->b_hat, est_data->w_hat); //w_hat = w - b_hat
    //linalg_zerovec(est_data->n, est_data->w_hat); //<------------------------------- THIS IS SET TO ZERO TO GAUGE EFFECT OF REFERENCE VECTORS IN ISOLATION
    linalg_vec2skew3x3(est_data->w_hat, est_data->w_hat_x); //w_hat_X = S(w_hat)
    linalg_matmatadd(est_data->n,est_data->n,est_data->w_hat_x, est_data->kpe_c_x, est_data->skew_mat); //S = S(w_hat) + S(Kpe*c)
    linalg_matscalmult(est_data->n,est_data->n,est_data->skew_mat,h,est_data->skew_mat_h); // Sh = S*h
    linalg_expm3x3(est_data->skew_mat_h, est_data->d_rot_mat_hat,9); //expm(Sh) = dR
    linalg_matmatmul_small(est_data->n,est_data->n,est_data->rot_mat_hat, est_data->d_rot_mat_hat, est_data->rot_mat_hat); //R_hat <- R_hat*dR
    linalg_matnormalizerotation(est_data->rot_mat_hat); //Ensure R_hat remains in SO(3)

    //Also do a lowpass of w_hat
    estimator_vec_low_pass(est_data->n,est_data->w_hat_f, est_data->w_hat, h / est_data->lambda);
}


void estimator_find_current_mag_direction(estStruct* est_data){
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
        estimator_get_imu_data(est_data);
        if(fabs(linalg_vecnorm(est_data->n, est_data->w)) > w_threshold){
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

    PRINT("Gathering magnetometer samples...\n");

    int N = 3;
    double fmagread[3];
    linalg_zerovec(N, fmagread);
    for(int i = 0; i < 200; i++){
        mmc5603_get_corrected_mag_reading(est_data->m);
        linalg_normalize(N,est_data->m,est_data->m);
        estimator_vec_low_pass(N,fmagread,est_data->m,0.05);
        sleep_ms(20); //Delay of 2ms may be too short
        //printvec(fmagread);
    }
    //Setting the filtered, normalized, and corrected magnetometer reading equal to the unit u1 vector used to calculate v2_hat:
    linalg_veccopy(N,fmagread, est_data->u1);
    PRINT("Done\n");
}

void estimator_set_initial_gyro_bias(estStruct* est_data)
{
    PRINT("Gathering IMU gyro samples...\n");
    double sum[3];
    int samples = 1500;
    linalg_zerovec(3, sum);
    for(int i = 0; i < samples; i++){
        estimator_get_imu_data(est_data);
        for(int j = 0; j < 3; j++){
            sum[j] += est_data->w[j];
        }
        sleep_ms(3);
    }
    //Set the initial gyro bias
    for(int j = 0; j < 3; j++){
        est_data->b_hat[j] = sum[j]/samples;
    }
    PRINT("Done\n");
}

void estimator_get_imu_data(estStruct* est_data)
{
    // MPU6050_get_imu_data(est_data->a, est_data->w);
    // MPU6050_six_point_accel_correction(est_data->a);
    // icm20948_get_imu_data(est_data->a, est_data->w);
    icm45686_get_imu_data(est_data->a, est_data->w);
}
