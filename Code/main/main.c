#include "headers/config.h"
#include "headers/logging.h"
#include <string.h>
#include "pico/stdlib.h"

//Custom includes
#include "headers/bus.h"
#include "headers/mpu6050.h"
#include "headers/mmc5603.h"
#include "headers/icm20948.h"
#include "headers/icm45686.h"
#include "headers/linalg.h"
#include "headers/estimator.h"
#include "headers/receiver.h"
#include "headers/servo.h"
#include "headers/controller.h"
#include "headers/optimizer.h"

void Main_init(contStruct* cont_data, recStruct* rec_data, estStruct* est_data);

void Main_run(contStruct* cont_data, recStruct* rec_data, estStruct* est_data, double h);


int main()
{
    stdio_init_all();
    sleep_ms(7000);
    Main_init(&controller_data, &receiver_data, &estimator_data);
    
    int N = 3;
    double eul[3];
    double bias[3];
    double wRaw[3];
    double empty[3];
    double various_data[3];
    double a_f[3];
    linalg_zerovec(N,empty);
    linalg_zerovec(N,empty);
    linalg_zerovec(N,various_data);
    double h = 0.0005;
    double mat[3][3];
    sleep_ms(1000);
    
    while (true)
    {
        uint64_t start = time_us_64();
        Main_run(&controller_data, &receiver_data, &estimator_data, h);
        
        estimator_rot_mat_to_euler(estimator_data.rot_mat_hat, eul);
        linalg_vecscalmult(N,eul, eul, 180.0/PI);
        linalg_vecscalmult(N,estimator_data.b_hat, bias, 180.0/PI);
        linalg_vecscalmult(N,estimator_data.w, wRaw, 180.0/PI);
        linalg_vecscalmult(N,estimator_data.b_hat, bias, 180.0/3.141590);
        linalg_colvecs2mat3x3(mat,eul,bias,wRaw);
        linalg_printmat(N,N,mat);

        
        sleep_ms(10);
        h = (time_us_64() - start)/1e6;
    }
}

void Main_init(contStruct* cont_data, recStruct* rec_data, estStruct* est_data)
{
    bus_spi_init();
    bus_i2c_init();
    sleep_ms(500);
    icm45686_init();
    //icm20948_init();

    //mmc5603_init();
    //MPU6050_init();
    servo_init();
    receiver_init(rec_data);
    controller_init(cont_data);
    estimator_init(est_data);
    //sleep_ms(500);
}

void Main_run(contStruct* cont_data, recStruct* rec_data, estStruct* est_data, double h)
{
    estimator_estimate_attitude(est_data, h);
    controller_run_quadcopter(cont_data, rec_data, est_data, h);
}


