#include "headers/servo.h"

void servo_setup_pwm_pins()
{
    gpio_set_function(SERVO_0, GPIO_FUNC_PWM);
    gpio_set_function(SERVO_1, GPIO_FUNC_PWM);
    gpio_set_function(SERVO_2, GPIO_FUNC_PWM);
    gpio_set_function(SERVO_3, GPIO_FUNC_PWM);

    uint slice_1 = pwm_gpio_to_slice_num(SERVO_0);
    uint slice_2 = pwm_gpio_to_slice_num(SERVO_2);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, CLOCK_DIVIDER);
    pwm_config_set_wrap(&config, PWM_WRAP_VALUE);
    //Load config into the slice and start it
    pwm_init(slice_1, &config, true);
    pwm_init(slice_2, &config, true);

    //Start the PWM signal
    pwm_set_enabled(slice_1, true);
    pwm_set_enabled(slice_2, true);
}

void servo_init()
{
    servo_setup_pwm_pins();
}

void servo_set_us(uint8_t pin, uint16_t period_us)
{
    if(pin != SERVO_0 && pin != SERVO_1 && pin != SERVO_2 && pin != SERVO_3){
        PRINT("Servo: Invalid servo/ESC pin provided to Servo_set_us()!\n");
    }else{
        if(period_us > MAX_SERVO_DUTY || period_us < MIN_SERVO_DUTY){
            // printf("Servo: period out of accepted range!\n");
            // while(1); //Halt forever. This makes it impossible to start the thing, must find something better here, maybe add a delay to this test or not test before a valid pwm has been set first
        }
        uint slice = pwm_gpio_to_slice_num(pin);
        uint16_t on_time = (uint32_t)(period_us*US_TO_CYCLES);
        uint PWM_CHANNEL = pin % 2; //PWM_CHAN_A = 0, PWM_CHAN_B = 1
        pwm_set_chan_level(slice, PWM_CHANNEL, on_time);
    }
}


