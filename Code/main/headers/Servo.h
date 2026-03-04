#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"
#include "headers/logging.h"
#include "headers/pins.h"
#include "hardware/pwm.h"

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

#define SERVO_0 18 //Slice 1, motor0
#define SERVO_1 19 //Slice 1, motor1
#define SERVO_2 20 //Slice 2, motor2
#define SERVO_3 21 //Slice 2, motor3


void servo_setup_pwm_pins();

void servo_init();

void servo_set_us(uint8_t pin, uint16_t period_us);

#endif
