#include "headers/Servo.h"

//PWM slice setup constants 
//Formula: PWM_WRAP_VALUE = 150*10^6/(Desired_PWM_freq*CLOCK_DIVIDER) - 1
//46874 at prescaler of 46874 gives a refresh rate of 50Hz(very slow)
//Setting it to 7037 gives a refresh rate of approximately 333Hz, which is what the DS452MG servos are designed for
//9375 - 1 -> 250Hz seems more stable
static const float CLOCK_DIVIDER = 64.f;
static const uint16_t PWM_WRAP_VALUE = 9374;
static const float F_CPU = 150*1e6; 
static const float US_TO_CYCLES = F_CPU/CLOCK_DIVIDER/1e6; //How many cycles there is in 1 micro second

static const int MAX_SERVO_DUTY = 2500;
static const int MIN_SERVO_DUTY = 500;

void Servo_setup_pwm_pins()
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

void Servo_init()
{
    Servo_setup_pwm_pins();
}

void Servo_set_us(ServoPin_t pin, uint16_t period_us)
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


