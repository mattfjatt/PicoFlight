#include "headers/config.h"
#include "headers/logging.h"
#include <string.h>
#include "pico/stdlib.h"

//Custom includes
#include "headers/bus.h"
#include "headers/mpu6050.h"
#include "headers/mmc5603.h"
#include "headers/mmc5983.h"
#include "headers/bmp388.h"
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
    double eul[3] = {0};
    double gyr[3] = {0};
    double acc[3] = {0};
    double bias[3] = {0};
    double wRaw[3] = {0};
    double empty[3] = {0};
    double various_data[3] = {0};
    double a_f[3] = {0};
    double h = 0.05;
    double mat[3][3] = {0};
    sleep_ms(1000);
    int counter = 0;

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
        linalg_printmat(3,3,mat);
        
        sleep_ms(5);
        h = (time_us_64() - start)/1e6;
    }
}

void Main_init(contStruct* cont_data, recStruct* rec_data, estStruct* est_data)
{
    bus_spi_init();
    sleep_ms(500);

    //bmp388_init();
    //mmc5983_init(); //May have a blocking while-loop!!
    // icm45686_TEST_FIFO();
    icm45686_init();

    //servo_init();
    //receiver_init(rec_data);
    //controller_init(cont_data);
    estimator_init(est_data);
    //sleep_ms(500);
}

void Main_run(contStruct* cont_data, recStruct* rec_data, estStruct* est_data, double h)
{
    estimator_estimate_attitude(est_data, h);
    //controller_run_quadcopter(cont_data, rec_data, est_data, h);
}


