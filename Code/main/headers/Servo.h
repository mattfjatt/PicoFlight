#ifndef SERVO_H
#define SERVO_H
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/pwm.h"

typedef enum {
    SERVO_0 = 18, //Slice 1, motor0
    SERVO_1 = 19, //Slice 1, motor1
    SERVO_2 = 20, //Slice 2, motor2
    SERVO_3 = 21, //Slice 2, motor3
} ServoPin_t;

void Servo_setup_pwm_pins();

void Servo_init();

void Servo_set_us(ServoPin_t pin, uint16_t period_us);

#endif
