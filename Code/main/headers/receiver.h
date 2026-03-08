#ifndef RECEIVER_H
#define RECEIVER_H

#include "headers/logging.h"
#include "headers/pins.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"

typedef struct{
    volatile uint32_t pulse_width[4];
    volatile uint32_t pulse_start[4];
    volatile bool previous_pin_state[4];
}recStruct;

extern recStruct receiver_data;

void receiver_gpio_callback(uint gpio, uint32_t events);

void receiver_setup_gpio_interrupts_pins();

void receiver_init_struct(recStruct* rec_data);

void receiver_print_data();

void receiver_init(recStruct* rec_data);

#endif
