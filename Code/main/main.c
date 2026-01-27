#include <stdio.h>
#include <string.h> //Not sure what this does, but it was included in the mpu6050 example. Maybe the %s formatter
#include "pico/stdlib.h"

//Custom includes
#include "headers/MPU6050.h"
#include "headers/LinAlg.h"
#include "headers/Estimator.h"
#include "headers/Receiver.h"
#include "headers/Servo.h"
#include "headers/Controller.h"

void Main_init(contStruct* contData, recStruct* recData, estStruct* estData);

void Main_run(contStruct* contData, recStruct* recData, estStruct* estData, float h);


int main()
{
    stdio_init_all();
    Main_init(&controllerData, &receiverData, &estimatorData);
    
    float eul[3];
    float bias[3];
    float wRaw[3];
    float empty[3];
    LinAlg_zerovec(empty);
    float h = 0.0005f;
    float mat[3][3];
    sleep_ms(1000);
    
    while (true)
    {
        uint64_t start = time_us_64();
        Main_run(&controllerData, &receiverData, &estimatorData, h);
        
        Estimator_R_to_euler(estimatorData.R_hat, eul);
        LinAlg_vecscalmult(eul, eul, 180.f/PI);
        LinAlg_vecscalmult(estimatorData.b_hat, bias, 180.f/PI);
        LinAlg_vecscalmult(estimatorData.w, wRaw, 180.f/PI);
        LinAlg_vecscalmult(estimatorData.b_hat, bias, 180.f/3.14159f);
        LinAlg_colvecs2mat(mat,eul,bias,empty);
        LinAlg_printmat(mat);
        
        // sleep_ms(10);
        h = (time_us_64() - start)/1e6;
    }
}

void Main_init(contStruct* contData, recStruct* recData, estStruct* estData)
{
    Receiver_init(recData);
    Controller_init(contData);
    Estimator_init(estData);
    MPU6050_init();
    Servo_init();
}

void Main_run(contStruct* contData, recStruct* recData, estStruct* estData, float h)
{
    Estimator_estimate_R(estData, h);
    Controller_run_quadcopter(contData, recData, estData, h);
}
