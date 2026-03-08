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

    double h = 0.005;
    
    while (true)
    {
        uint64_t start = time_us_64();
        Main_run(&controller_data, &receiver_data, &estimator_data, h);
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
    //mpu6050_init();
    //servo_init();
    //receiver_init(rec_data);
    controller_init(cont_data);
    estimator_init(est_data);
    //sleep_ms(500);
}

void Main_run(contStruct* cont_data, recStruct* rec_data, estStruct* est_data, double h)
{
    estimator_estimate_attitude(est_data, h);
    controller_run_quadcopter(cont_data, rec_data, est_data, h);
}


