#include "headers/controller.h"

contStruct controller_data;
double root3 = 1.732050f;
uint16_t ServoArray[4] = {SERVO_0, SERVO_1, SERVO_2, SERVO_3};


void controller_init(contStruct* cont_data){
    cont_data->N = 3;
    linalg_eye(cont_data->N,cont_data->rot_mat_d);
    linalg_eye(cont_data->N,cont_data->rot_mat_d_transposed);
    linalg_eye(cont_data->N,cont_data->inertia_mat_hat);

    //Set gains:
    cont_data->kp_mat[0][0] = - kp_x;
    cont_data->kp_mat[1][1] = - kp_y;
    cont_data->kp_mat[2][2] = - kp_z;

    cont_data->kd_mat[0][0] = - kd_x;
    cont_data->kd_mat[1][1] = - kd_y;
    cont_data->kd_mat[2][2] = - kd_z;

    cont_data->ki_mat[0][0] = - ki_x;
    cont_data->ki_mat[1][1] = - ki_y;
    cont_data->ki_mat[2][2] = - ki_z;

    //Set integrator wind-up limits
    cont_data->e_i_windup[0] = windup_x;
    cont_data->e_i_windup[1] = windup_y;
    cont_data->e_i_windup[2] = windup_z;

    //Flags & arming
    cont_data->run_integrators = false; //Integrators not allowed to run immediately
    cont_data->arm_controller = false; //Controller starts as disarmed

    cont_data->arm_timer = 0.0;
    cont_data->arm_threshold = 2.0; //Seconds
    cont_data->arm_limits[0] = arm_ch0;
    cont_data->arm_limits[1] = arm_ch1;
    cont_data->arm_limits[2] = arm_ch2;
    cont_data->arm_limits[3] = arm_ch3;

    //Builtin LED setup
    cont_data->led_builtin = 25;
    gpio_init(cont_data->led_builtin);
    gpio_set_dir(cont_data->led_builtin, GPIO_OUT);

    //Set all motors to off
    servo_set_us(SERVO_0, throttle_poweroff);
    servo_set_us(SERVO_1, throttle_poweroff);
    servo_set_us(SERVO_2, throttle_poweroff);
    servo_set_us(SERVO_3, throttle_poweroff);

    //Throttle thresholds used for truncating duty cycle and safe power off when throttle at lowest position
    cont_data->throttle_cutoff = throttle_cutoff;
    cont_data->throttle_max = throttle_max;
    cont_data->throttle_min = throttle_min;

    //Controller behavior
    cont_data->roll_rate = roll_rate;
    cont_data->pitch_rate = pitch_rate;
    cont_data->yaw_rate = yaw_rate;
}

void controller_get_e_hat(contStruct* cont_data, estStruct* est_data){
    linalg_mattranspose(cont_data->N,cont_data->N,cont_data->rot_mat_d, cont_data->rot_mat_d_transposed);
    linalg_matmatmul_small(cont_data->N,cont_data->N,cont_data->rot_mat_d_transposed, est_data->rot_mat_hat, cont_data->e_mat_hat);
    //Extract the error vector e_hat from e_mat_hat:
    cont_data->e_hat[0] = 0.5*(cont_data->e_mat_hat[2][1] - cont_data->e_mat_hat[1][2]);
    cont_data->e_hat[1] = 0.5*(cont_data->e_mat_hat[0][2] - cont_data->e_mat_hat[2][0]);
    cont_data->e_hat[2] = 0.5*(cont_data->e_mat_hat[1][0] - cont_data->e_mat_hat[0][1]);
}

void controller_get_e_hat_integral(contStruct* cont_data, estStruct* est_data, double h)
{
    if(!cont_data->run_integrators){
        return;
    }
    //e_i_hat = e_i_hat + h*ki_mat*e_hat
    double temp[3];
    linalg_matvecmul(cont_data->N,cont_data->N,cont_data->ki_mat, cont_data->e_hat, temp); //temp = ki_mat*e_hat
    linalg_vecscalmult(cont_data->N,temp, temp, h); //temp = h*ki_mat*e_hat
    linalg_vecvecadd(cont_data->N,cont_data->e_i_hat, temp, cont_data->e_i_hat); //e_i_hat = e_i_hat + temp 

    //anti wind-up
    for(int i = 0; i < 3; i++){
        if(cont_data->e_i_hat[i] > cont_data->e_i_windup[i]){
            cont_data->e_i_hat[i] = cont_data->e_i_windup[i];
        }else if(cont_data->e_i_hat[i] < - cont_data->e_i_windup[i]){
            cont_data->e_i_hat[i] = - cont_data->e_i_windup[i];
        }
    }
}

void controller_get_tau(contStruct* cont_data, estStruct* est_data){
    //tau = I_hat*(- kp_mat*e_hat - Kd*w_hat - ki_mat*e_i_hat) + S(I_hat*w_hat)*w_hat <- ignoring the skew part:)
    double temp[3];
    controller_get_e_hat(cont_data, est_data);
    linalg_matvecmul(cont_data->N,cont_data->N,cont_data->kp_mat, cont_data->e_hat, cont_data->kp_e_hat);
    linalg_matvecmul(cont_data->N,cont_data->N,cont_data->kd_mat, est_data->w_hat_f, cont_data->kd_w_hat); //Using the filtered w_hat for the derivative part
    linalg_vecvecadd(cont_data->N,cont_data->kp_e_hat, cont_data->kd_w_hat, temp);
    linalg_vecvecadd(cont_data->N,cont_data->e_i_hat, temp, cont_data->tau); //ki_mat is already multiplied elsewhere
    linalg_matvecmul(cont_data->N,cont_data->N,cont_data->inertia_mat_hat, cont_data->tau, cont_data->tau);
}

void controller_control_alloc_tricopter(contStruct* cont_data){
    //Implement control allocation for tricopter:

    //Controller inputs:
    cont_data->d_force_left = 4.0f/root3*cont_data->tau[0] + 4.0f/3.0f*cont_data->tau[1];
    cont_data->d_force_right = - 4.0f/root3*cont_data->tau[0] + 4.0f/3.0f*cont_data->tau[1];
    cont_data->d_force_back = -8.0f/3.0f*cont_data->tau[1];
    cont_data->d_force_turn = -4.0f*cont_data->tau[2];

    //Adding controller inputs to total force required from each motor:
    cont_data->throttle = 3.0; //Placeholder
    cont_data->force_left = cont_data->throttle + cont_data->d_force_left;
    cont_data->force_right = cont_data->throttle + cont_data->d_force_right;
    cont_data->force_back = cont_data->throttle + cont_data->d_force_back;
    cont_data->force_turn = cont_data->d_force_turn;

    cont_data->force_aft = sqrt(cont_data->force_back*cont_data->force_back + cont_data->force_turn*cont_data->force_turn);

    if(cont_data->force_back > 0.4f){ //Confirm that 0.4[N] is a reasonable value
        cont_data->alpha = atan(cont_data->force_turn/cont_data->force_back);
    }else{
        cont_data->alpha = 0.0f;
    }
}

void controller_control_alloc_quadcopter(contStruct* cont_data)
{
    double K = 1.f; //This is some constant given by the geometry of the quadcopter-frame
    cont_data->d_force_0 = ( - cont_data->tau[0] + cont_data->tau[1] - K*cont_data->tau[2])/(4.f*K); //Flipped the plus/minus signs of the tau[2] terms as the motors spin the opposite direction
    cont_data->d_force_1 = (   cont_data->tau[0] + cont_data->tau[1] + K*cont_data->tau[2])/(4.f*K);
    cont_data->d_force_2 = (   cont_data->tau[0] - cont_data->tau[1] - K*cont_data->tau[2])/(4.f*K);
    cont_data->d_force_3 = ( - cont_data->tau[0] - cont_data->tau[1] + K*cont_data->tau[2])/(4.f*K);

    cont_data->force_0 = cont_data->throttle + cont_data->d_force_0;
    cont_data->force_1 = cont_data->throttle + cont_data->d_force_1;
    cont_data->force_2 = cont_data->throttle + cont_data->d_force_2;
    cont_data->force_3 = cont_data->throttle + cont_data->d_force_3;
}

void controller_run_quadcopter(contStruct* cont_data, recStruct* rec_data, estStruct* est_data, double h)
{
    if(!cont_data->arm_controller){
        controller_check_for_arming(cont_data, rec_data, h); //Move to init?
        return;
    }

    //Get desired input from R/C receiver and convert to desired rotation LinAlg rot_mat_d
    cont_data->euler[0] =   ((int)rec_data->pulse_width[3] - 1501)/10.0/180.0*PI; //Roll
    cont_data->euler[1] = - ((int)rec_data->pulse_width[2] - 1513)/10.0/180.0*PI; //Pitch

    #ifdef RATE_YAW_CONTROL
    int deadband = 5;
    int raw_yaw_input = abs((int)rec_data->pulse_width[0] - 1512) > deadband ? (int)rec_data->pulse_width[0] - 1512 : 0;
    double d_yaw = raw_yaw_input/10.0f/180.0f*PI*h*cont_data->yaw_rate;
    cont_data->euler[2] -= d_yaw;
    #else
    cont_data->euler[2] = - ((int)rec_data->pulse_width[0] - 1512)/10.0/180.0*PI; //Yaw
    #endif

    if(cont_data->throttle < cont_data->throttle_cutoff){
        controller_reset_integrated_setpoints(cont_data);
    }

    //Convert the euler angles to a desired rotation rot_mat_d
    estimator_euler_to_rot_mat(cont_data->euler, cont_data->rot_mat_d);

    //Get throtte command
    cont_data->throttle = (double)(rec_data->pulse_width[1]);

    //Compute the integral of the error vector
    controller_get_e_hat_integral(cont_data, est_data, h);

    //Get the desired torque tau
    controller_get_tau(cont_data, est_data);

    //Convert the torque to motor commands
    controller_control_alloc_quadcopter(cont_data);

    //Send the commands to the motors after mapping force to microseconds
    //T \in [960, 1920]
    if(cont_data->throttle < cont_data->throttle_cutoff){
        //This enables us to turn the motors off via the transmitter even if the controller contributions dFi are batshit crazy
        servo_set_us(SERVO_0, cont_data->throttle);
        servo_set_us(SERVO_1, cont_data->throttle);
        servo_set_us(SERVO_2, cont_data->throttle);
        servo_set_us(SERVO_3, cont_data->throttle);
        //Disable integrators
        cont_data->run_integrators = false;
    }else{
        //Yes, this should be loop-ified

        //Motor 0
        if(cont_data->force_0 > cont_data->throttle_max){
            cont_data->force_0 = cont_data->throttle_max;
        }else if(cont_data->force_0 < cont_data->throttle_min){
            cont_data->force_0 = cont_data->throttle_min;
        }

        //Motor 1
        if(cont_data->force_1 > cont_data->throttle_max){
            cont_data->force_1 = cont_data->throttle_max;
        }else if(cont_data->force_1 < cont_data->throttle_min){
            cont_data->force_1 = cont_data->throttle_min;
        }

        //Motor 2
        if(cont_data->force_2 > cont_data->throttle_max){
            cont_data->force_2 = cont_data->throttle_max;
        }else if(cont_data->force_2 < cont_data->throttle_min){
            cont_data->force_2 = cont_data->throttle_min;
        }

        //Motor 3
        if(cont_data->force_3 > cont_data->throttle_max){
            cont_data->force_3 = cont_data->throttle_max;
        }else if(cont_data->force_3 < cont_data->throttle_min){
            cont_data->force_3 = cont_data->throttle_min;
        }

        //Set the values
        servo_set_us(SERVO_0, cont_data->force_0);
        servo_set_us(SERVO_1, cont_data->force_1);
        servo_set_us(SERVO_2, cont_data->force_2);
        servo_set_us(SERVO_3, cont_data->force_3);

        //Enable integrators
        cont_data->run_integrators = true;
    }
}

void controller_reset_integrated_setpoints(contStruct* cont_data)
{
    //When the arming sequence is running, all channels are at values close to one of their extremes, this isn't a problem
    //unless a given channel is used for rate control. The current implementation of rate control is "angle" control under the hood,
    //the angle is simply integrated and sent to the controller, this means that when arming, the yaw angle is integrated for at least 
    //two seconds, leading to a very large heading error internal to the controller. To mitigate this, we reset all input angles to 0
    //when the throttle is below TCutoff:
    linalg_zerovec(cont_data->N,cont_data->euler);
}

void controller_check_for_arming(contStruct* cont_data, recStruct* rec_data, double h)
{
    if( rec_data->pulse_width[0] < cont_data->arm_limits[0] && rec_data->pulse_width[0] > 0 &&
        rec_data->pulse_width[1] < cont_data->arm_limits[1] && rec_data->pulse_width[1] > 0 &&
        rec_data->pulse_width[2] < cont_data->arm_limits[2] && rec_data->pulse_width[2] > 0 &&
        rec_data->pulse_width[3] < cont_data->arm_limits[3] && rec_data->pulse_width[3] > 0 &&
        cont_data->arm_timer < cont_data->arm_threshold && !cont_data->arm_controller){
        cont_data->arm_timer += h;
    }else if(cont_data->arm_timer >= cont_data->arm_threshold && !cont_data->arm_controller){
        cont_data->arm_controller = true;
        gpio_put(cont_data->led_builtin, 1); // turn builtin LED on to signal that the controller is armed
    }
}

void controller_run_tricopter(contStruct* cont_data, recStruct* rec_data, estStruct* est_data)
{
    //Get desired input from R/C receiver and convert to desired rotation LinAlg rot_mat_d
    double eul[3] = {0.0f, 0.0f, 0.0f};
    eul[0] =   ((int)rec_data->pulse_width[1] - 1500)/10.0f/180.0f*PI; //Roll
    eul[1] = - ((int)rec_data->pulse_width[3] - 1500)/10.0f/180.0f*PI; //Pitch
    eul[2] = - ((int)rec_data->pulse_width[2] - 1500)/10.0f/180.0f*PI; //Yaw
    linalg_eye(cont_data->N,cont_data->rot_mat_d);
    estimator_euler_to_rot_mat(eul, cont_data->rot_mat_d); 
    
    //Get the desired torque tau
    controller_get_tau(cont_data, est_data);

    //Use the torque (not force) to obtain forces each motor is to produce
    controller_control_alloc_tricopter(cont_data); 

    //Now convert the forces and angle to PWM values.
    //Forces are: FA, FL, FA
    //Angle: a
    double dutyA, dutyL, dutyR, dutya;
    dutyA = controller_force_to_duty(cont_data->force_aft);
    dutyL = controller_force_to_duty(cont_data->force_left);
    dutyR = controller_force_to_duty(cont_data->force_right);
    dutya = controller_angle_deg_to_duty(cont_data->alpha*180.0f/3.14159f);
}

double controller_force_to_duty(double F){
    //a and b are parameters for a linear model, obtained in Matlab.
    //The return value is the duty cycle in percent corresponding to
    //a desired force when 4 cell LiPo battery is used
    double a = 10.2684f;
    double b = 3.1f;
    return F*a + b;
}

double controller_angle_deg_to_duty(double a){
    //This value is empirically determined
    // 30 deg: 1934
    // 0  deg: 1475
    //-30 deg: 1080
    double angle_pwm = 1475.0f + 800.0f*(a/30.0f);
    return angle_pwm;
}

void controller_print_forces_PWM_tricopter(const contStruct* cont_data, const char* type){
    if(type == "Forces"){
        PRINTNUM("FL: %.2f.", cont_data->force_left);
        PRINTNUM("FR: %.2f.", cont_data->force_right);
        PRINTNUM("FA: %.2f.", cont_data->force_aft);
        PRINTNUM("a: %.2f\n.", cont_data->alpha*180/3.1415926);
    }else if(type == "PWM"){
        PRINTNUM("PWM L: %d.", controller_force_to_duty(cont_data->force_left));
        PRINTNUM("PWM R: %d.", controller_force_to_duty(cont_data->force_right));
        PRINTNUM("PWM A: %d.", controller_force_to_duty(cont_data->force_aft));
    }else{
        PRINT("No valid char* sent to print_forces_PWM\n");
    }
    
}

void controller_print_forces_PWM_quadcopter(const contStruct* cont_data){
    PRINTNUM("force_0: %.2f.", cont_data->force_0);
    PRINTNUM("force_1: %.2f.", cont_data->force_1);
    PRINTNUM("force_2: %.2f.", cont_data->force_2);
    PRINTNUM("force_3: %.2f\n.", cont_data->force_3);
}
