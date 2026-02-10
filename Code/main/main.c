#include "headers/Config.h"
#include "headers/Logging.h"
#include <string.h> //Not sure what this does, but it was included in the mpu6050 example. Maybe the %s formatter
#include "pico/stdlib.h"

//Custom includes
#include "headers/MPU6050.h"
#include "headers/MMC5603.h"
#include "headers/LinAlg.h"
#include "headers/Estimator.h"
#include "headers/Receiver.h"
#include "headers/Servo.h"
#include "headers/Controller.h"
#include "headers/Optimizer.h"

void Main_init(contStruct* contData, recStruct* recData, estStruct* estData);

void Main_run(contStruct* contData, recStruct* recData, estStruct* estData, double h);


int main()
{
    stdio_init_all();
    sleep_ms(7000);


    MMC5603_init();
    Main_init(&controllerData, &receiverData, &estimatorData);
    
    int N = 3;
    double eul[3];
    double bias[3];
    double wRaw[3];
    double empty[3];
    double various_data[3];
    double a_f[3];
    LinAlg_zerovec(N,empty);
    LinAlg_zerovec(N,empty);
    LinAlg_zerovec(N,various_data);
    double h = 0.0005;
    double mat[3][3];
    sleep_ms(1000);
    
    while (true)
    {
        uint64_t start = time_us_64();
        Main_run(&controllerData, &receiverData, &estimatorData, h);
        
        Estimator_R_to_euler(estimatorData.R_hat, eul);
        LinAlg_vecscalmult(N,eul, eul, 180.f/PI);
        LinAlg_vecscalmult(N,estimatorData.b_hat, bias, 180.f/PI);
        LinAlg_vecscalmult(N,estimatorData.w, wRaw, 180.f/PI);
        LinAlg_vecscalmult(N,estimatorData.b_hat, bias, 180.f/3.14159f);
        // various_data[0] = LinAlg_vecnorm(N, estimatorData.m);
        // various_data[1] = LinAlg_vecnorm(N, a_f);
        // Estimator_vecLP(N,a_f,estimatorData.a,0.005);
        LinAlg_colvecs2mat3x3(mat,eul,bias,wRaw);
        //LinAlg_colvecs2mat3x3(mat,eul,estimatorData.v1,estimatorData.v1_hat);
        //LinAlg_printmat(N,N,mat);
        // LinAlg_matvecmul(N,N,estimatorData.R_hat_T, estimatorData.ni3, various_data);
        // LinAlg_printvec(N,various_data);
        
        sleep_ms(10);
        h = (time_us_64() - start)/1e6;
    }
}

void Main_init(contStruct* contData, recStruct* recData, estStruct* estData)
{
    MPU6050_init();
    Servo_init();
    Receiver_init(recData);
    Controller_init(contData);
    Estimator_init(estData);
}

void Main_run(contStruct* contData, recStruct* recData, estStruct* estData, double h)
{
    Estimator_estimate_R(estData, h);
    Controller_run_quadcopter(contData, recData, estData, h);
}
