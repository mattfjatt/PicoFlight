#include "headers/Controller.h"

contStruct controllerData;
double root3 = 1.732050f;
uint16_t ServoArray[4] = {SERVO_0, SERVO_1, SERVO_2, SERVO_3};


void Controller_init(contStruct* contData){
    contData->N = 3;
    LinAlg_eye(contData->N,contData->Rd);
    LinAlg_eye(contData->N,contData->Rd_T);
    LinAlg_eye(contData->N,contData->I_hat);
    LinAlg_zeromat(contData->N,contData->N,contData->E_hat);
    LinAlg_zeromat(contData->N,contData->N,contData->Kp);
    LinAlg_zeromat(contData->N,contData->N,contData->Kd);
    LinAlg_zeromat(contData->N,contData->N,contData->Ki);

    LinAlg_zerovec(contData->N,contData->e_hat);
    LinAlg_zerovec(contData->N,contData->tau);
    LinAlg_zerovec(contData->N,contData->Kp_e_hat);
    LinAlg_zerovec(contData->N,contData->Kd_w_hat);
    LinAlg_zerovec(contData->N,contData->e_w_hat);
    LinAlg_zerovec(contData->N,contData->euler);
    LinAlg_zerovec(contData->N,contData->e_i_hat);

    //Set gains:
    contData->Kp[0][0] = - KpX;
    contData->Kp[1][1] = - KpY;
    contData->Kp[2][2] = - KpZ;

    contData->Kd[0][0] = - KdX;
    contData->Kd[1][1] = - KdY;
    contData->Kd[2][2] = - KdZ;

    contData->Ki[0][0] = - KiX;
    contData->Ki[1][1] = - KiY;
    contData->Ki[2][2] = - KiZ;

    //Set integrator wind-up limits
    contData->e_i_wind_up[0] = WindUpX;
    contData->e_i_wind_up[1] = WindupY;
    contData->e_i_wind_up[2] = WindUpZ;

    //Flags & arming
    contData->RunIntegrators = false; //Integrators not allowed to run immediately
    contData->ArmController = false; //Controller starts as disarmed

    contData->ArmTimer = 0.0;
    contData->ArmThreshold = 2.0; //Seconds
    contData->ArmLimits[0] = ArmCh0;
    contData->ArmLimits[1] = ArmCh1;
    contData->ArmLimits[2] = ArmCh2;
    contData->ArmLimits[3] = ArmCh3;

    //Builtin LED setup
    contData->LedBuiltin = 25;
    gpio_init(contData->LedBuiltin);
    gpio_set_dir(contData->LedBuiltin, GPIO_OUT);

    //Set all motors to off
    Servo_set_us(SERVO_0, TPowerOff);
    Servo_set_us(SERVO_1, TPowerOff);
    Servo_set_us(SERVO_2, TPowerOff);
    Servo_set_us(SERVO_3, TPowerOff);

    //Throttle thresholds used for truncating duty cycle and safe power off when throttle at lowest position
    contData->TCutoff = TCutoff;
    contData->TMax = TMax;
    contData->TMin = TMin;

    //Controller behavior
    contData->RollRate = RollRate;
    contData->PitchRate = PitchRate;
    contData->YawRate = YawRate;
}

void Controller_get_e_hat(contStruct* contData, estStruct* estData){
    LinAlg_mattranspose(contData->N,contData->N,contData->Rd, contData->Rd_T);
    LinAlg_matmatmul_small(contData->N,contData->N,contData->Rd_T, estData->R_hat, contData->E_hat);
    //Extract the error vector e_hat from E_hat:
    contData->e_hat[0] = 0.5*(contData->E_hat[2][1] - contData->E_hat[1][2]);
    contData->e_hat[1] = 0.5*(contData->E_hat[0][2] - contData->E_hat[2][0]);
    contData->e_hat[2] = 0.5*(contData->E_hat[1][0] - contData->E_hat[0][1]);
}

void Controller_get_e_hat_integral(contStruct* contData, estStruct* estData, double h)
{
    if(!contData->RunIntegrators){
        return;
    }
    //e_i_hat = e_i_hat + h*Ki*e_hat
    double temp[3];
    LinAlg_matvecmul(contData->N,contData->N,contData->Ki, contData->e_hat, temp); //temp = Ki*e_hat
    LinAlg_vecscalmult(contData->N,temp, temp, h); //temp = h*Ki*e_hat
    LinAlg_vecvecadd(contData->N,contData->e_i_hat, temp, contData->e_i_hat); //e_i_hat = e_i_hat + temp 

    //anti wind-up
    for(int i = 0; i < 3; i++){
        if(contData->e_i_hat[i] > contData->e_i_wind_up[i]){
            contData->e_i_hat[i] = contData->e_i_wind_up[i];
        }else if(contData->e_i_hat[i] < - contData->e_i_wind_up[i]){
            contData->e_i_hat[i] = - contData->e_i_wind_up[i];
        }
    }
}

void Controller_get_tau(contStruct* contData, estStruct* estData){
    //tau = I_hat*(- Kp*e_hat - Kd*w_hat - Ki*e_i_hat) + S(I_hat*w_hat)*w_hat <- ignoring the skew part:)
    double temp[3];
    Controller_get_e_hat(contData, estData);
    LinAlg_matvecmul(contData->N,contData->N,contData->Kp, contData->e_hat, contData->Kp_e_hat);
    LinAlg_matvecmul(contData->N,contData->N,contData->Kd, estData->w_hat_f, contData->Kd_w_hat); //Using the filtered w_hat for the derivative part
    LinAlg_vecvecadd(contData->N,contData->Kp_e_hat, contData->Kd_w_hat, temp);
    LinAlg_vecvecadd(contData->N,contData->e_i_hat, temp, contData->tau); //Ki is already multiplied elsewhere
    LinAlg_matvecmul(contData->N,contData->N,contData->I_hat, contData->tau, contData->tau);
}

void Controller_control_alloc_tricopter(contStruct* contData){
    //Implement control allocation for tricopter:

    //Controller inputs:
    contData->dFL = 4.0f/root3*contData->tau[0] + 4.0f/3.0f*contData->tau[1];
    contData->dFR = - 4.0f/root3*contData->tau[0] + 4.0f/3.0f*contData->tau[1];
    contData->dFB = -8.0f/3.0f*contData->tau[1];
    contData->dFT = -4.0f*contData->tau[2];

    //Adding controller inputs to total force required from each motor:
    contData->T = 3.0; //Placeholder
    contData->FL = contData->T + contData->dFL;
    contData->FR = contData->T + contData->dFR;
    contData->FB = contData->T + contData->dFB;
    contData->FT = contData->dFT;

    contData->FA = sqrt(contData->FB*contData->FB + contData->FT*contData->FT);

    if(contData->FB > 0.4f){ //Confirm that 0.4[N] is a reasonable value
        contData->a = atan(contData->FT/contData->FB);
    }else{
        contData->a = 0.0f;
    }
}

void Controller_control_alloc_quadcopter(contStruct* contData)
{
    double K = 1.f; //This is some constant given by the geometry of the quadcopter-frame
    contData->dF0 = ( - contData->tau[0] + contData->tau[1] - K*contData->tau[2])/(4.f*K); //Flipped the plus/minus signs of the tau[2] terms as the motors spin the opposite direction
    contData->dF1 = (   contData->tau[0] + contData->tau[1] + K*contData->tau[2])/(4.f*K);
    contData->dF2 = (   contData->tau[0] - contData->tau[1] - K*contData->tau[2])/(4.f*K);
    contData->dF3 = ( - contData->tau[0] - contData->tau[1] + K*contData->tau[2])/(4.f*K);

    contData->F0 = contData->T + contData->dF0;
    contData->F1 = contData->T + contData->dF1;
    contData->F2 = contData->T + contData->dF2;
    contData->F3 = contData->T + contData->dF3;
}

void Controller_run_quadcopter(contStruct* contData, recStruct* recData, estStruct* estData, double h)
{
    if(!contData->ArmController){
        Controller_check_for_arming(contData, recData, h);
        return;
    }

    //Get desired input from R/C receiver and convert to desired rotation LinAlg Rd
    contData->euler[0] =   ((int)recData->pulse_width[3] - 1501)/10.0f/180.0f*PI; //Roll
    contData->euler[1] = - ((int)recData->pulse_width[2] - 1513)/10.0f/180.0f*PI; //Pitch

    #ifdef RATE_YAW_CONTROL
    int deadband = 5;
    int raw_yaw_input = abs((int)recData->pulse_width[0] - 1512) > deadband ? (int)recData->pulse_width[0] - 1512 : 0;
    double d_yaw = raw_yaw_input/10.0f/180.0f*PI*h*contData->YawRate;
    contData->euler[2] -= d_yaw;
    #else
    contData->euler[2] = - ((int)recData->pulse_width[0] - 1512)/10.0f/180.0f*PI; //Yaw
    #endif

    if(contData->T < contData->TCutoff){
        Controller_reset_integrated_setpoints(contData);
    }

    //Convert the euler angles to a desired rotation Rd
    Estimator_euler_to_R(contData->euler, contData->Rd); 

    //Get throtte command
    contData->T = (double)(recData->pulse_width[1]);

    //Compute the integral of the error vector
    Controller_get_e_hat_integral(contData, estData, h);

    //Get the desired torque tau
    Controller_get_tau(contData, estData);

    //Convert the torque to motor commands
    Controller_control_alloc_quadcopter(contData);

    //Send the commands to the motors after mapping force to microseconds
    //T \in [960, 1920]
    if(contData->T < contData->TCutoff){
        //This enables us to turn the motors off via the transmitter even if the controller contributions dFi are batshit crazy
        Servo_set_us(SERVO_0, contData->T);
        Servo_set_us(SERVO_1, contData->T);
        Servo_set_us(SERVO_2, contData->T);
        Servo_set_us(SERVO_3, contData->T);
        //Disable integrators
        contData->RunIntegrators = false;
    }else{
        //Yes, this should be loop-ified

        //Motor 0
        if(contData->F0 > contData->TMax){
            contData->F0 = contData->TMax;
        }else if(contData->F0 < contData->TMin){
            contData->F0 = contData->TMin;
        }

        //Motor 1
        if(contData->F1 > contData->TMax){
            contData->F1 = contData->TMax;
        }else if(contData->F1 < contData->TMin){
            contData->F1 = contData->TMin;
        }

        //Motor 2
        if(contData->F2 > contData->TMax){
            contData->F2 = contData->TMax;
        }else if(contData->F2 < contData->TMin){
            contData->F2 = contData->TMin;
        }

        //Motor 3
        if(contData->F3 > contData->TMax){
            contData->F3 = contData->TMax;
        }else if(contData->F3 < contData->TMin){
            contData->F3 = contData->TMin;
        }

        //Set the values
        Servo_set_us(SERVO_0, contData->F0);
        Servo_set_us(SERVO_1, contData->F1);
        Servo_set_us(SERVO_2, contData->F2);
        Servo_set_us(SERVO_3, contData->F3);

        //Enable integrators
        contData->RunIntegrators = true;
    }
}

void Controller_reset_integrated_setpoints(contStruct* contData)
{
    //When the arming sequence is running, all channels are at values close to one of their extremes, this isn't a problem
    //unless a given channel is used for rate control. The current implementation of rate control is "angle" control under the hood,
    //the angle is simply integrated and sent to the controller, this means that when arming, the yaw angle is integrated for at least 
    //two seconds, leading to a very large heading error internal to the controller. To mitigate this, we reset all input angles to 0
    //when the throttle is below TCutoff:
    LinAlg_zerovec(contData->N,contData->euler);
}

void Controller_check_for_arming(contStruct* contData, recStruct* recData, double h)
{
    if( recData->pulse_width[0] < contData->ArmLimits[0] && recData->pulse_width[0] > 0 &&
        recData->pulse_width[1] < contData->ArmLimits[1] && recData->pulse_width[1] > 0 &&
        recData->pulse_width[2] < contData->ArmLimits[2] && recData->pulse_width[2] > 0 &&
        recData->pulse_width[3] < contData->ArmLimits[3] && recData->pulse_width[3] > 0 &&
        contData->ArmTimer < contData->ArmThreshold && !contData->ArmController){
        contData->ArmTimer += h;
    }else if(contData->ArmTimer >= contData->ArmThreshold && !contData->ArmController){
        contData->ArmController = true;
        gpio_put(contData->LedBuiltin, 1); // turn builtin LED on to signal that the controller is armed
    }
}

void Controller_run_tricopter(contStruct* contData, recStruct* recData, estStruct* estData)
{
    //Get desired input from R/C receiver and convert to desired rotation LinAlg Rd
    double eul[3] = {0.0f, 0.0f, 0.0f};
    eul[0] =   ((int)recData->pulse_width[1] - 1500)/10.0f/180.0f*PI; //Roll
    eul[1] = - ((int)recData->pulse_width[3] - 1500)/10.0f/180.0f*PI; //Pitch
    eul[2] = - ((int)recData->pulse_width[2] - 1500)/10.0f/180.0f*PI; //Yaw
    LinAlg_eye(contData->N,contData->Rd);
    Estimator_euler_to_R(eul, contData->Rd); 
    
    //Get the desired torque tau
    Controller_get_tau(contData, estData);

    //Use the torque (not force) to obtain forces each motor is to produce
    Controller_control_alloc_tricopter(contData); 

    //Now convert the forces and angle to PWM values.
    //Forces are: FA, FL, FA
    //Angle: a
    double dutyA, dutyL, dutyR, dutya;
    dutyA = Controller_force_to_duty(contData->FA);
    dutyL = Controller_force_to_duty(contData->FL);
    dutyR = Controller_force_to_duty(contData->FR);
    dutya = Controller_angle_deg_to_duty(contData->a*180.0f/3.14159f);


    /*
    double dutyA_pwm = dutyA/100.f*800.f + 1500.f;
    printf("pwm A: %.2f\n", dutyA_pwm);
    set_pwm(dutyA_pwm, 0);
    //Use the duties above to control ESCs and servo. The duties are in [0, 100], need to be converted to microseconds
    //printf("dutyA: %.2f. dutyL: %.2f. dutyR: %.2f. dutya: %.2f\n", dutyA, dutyL, dutyR, dutya);
    set_pwm(dutya, 3);
    //printf("angle: %.2f\n", contData->a*180.0f/3.14159f);
    */
}

double Controller_force_to_duty(double F){
    //a and b are parameters for a linear model, obtained in Matlab.
    //The return value is the duty cycle in percent corresponding to
    //a desired force when 4 cell LiPo battery is used
    double a = 10.2684f;
    double b = 3.1f;
    return F*a + b;
}

double Controller_angle_deg_to_duty(double a){
    //This value is empirically determined
    // 30 deg: 1934
    // 0  deg: 1475
    //-30 deg: 1080
    double angle_pwm = 1475.0f + 800.0f*(a/30.0f);
    return angle_pwm;
}

void Controller_print_forces_PWM_tricopter(const contStruct* contData, const char* type){
    if(type == "Forces"){
        //printf("dFL: %.2f. dFR: %.2f, dFB: %.2f, dFT: %.2f\n", contData->dFL, contData->dFR, contData->dFB, contData->dFT);
        printf("FL: %.2f. FR: %.2f, FA: %.2f, a: %.2f\n", contData->FL, contData->FR, contData->FA, contData->a*180/3.1415926);
    }else if(type == "PWM"){
        printf("PWM L: %d. PWM R: %d, PWM A: %d\n", Controller_force_to_duty(contData->FL), Controller_force_to_duty(contData->FR), Controller_force_to_duty(contData->FA));
    }else{
        printf("No valid char* sent to print_forces_PWM\n");
    }
    
}

void Controller_print_forces_PWM_quadcopter(const contStruct* contData){
    //printf("dF0: %.2f. dF1: %.2f, dF2: %.2f, dF3: %.2f\n", contData->dF0, contData->dF1, contData->dF2, contData->dF3);
    printf("F0: %.2f. F1: %.2f, F2: %.2f, F3: %.2f\n", contData->F0, contData->F1, contData->F2, contData->F3);
}
