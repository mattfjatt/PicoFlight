#ifndef RECEIVER_H
#define RECEIVER_H

#include "headers/Logging.h"
#include "headers/Pins.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"

typedef struct{
    volatile uint32_t pulse_width[4];
    volatile uint32_t pulse_start[4];
    volatile bool previous_pin_state[4];
}recStruct;

extern recStruct receiverData;

void Receiver_gpio_callback(uint gpio, uint32_t events);

void Receiver_setup_gpio_interrupts_pins();

void Receiver_init_struct(recStruct* recData);

void Receiver_print_data();

void Receiver_init(recStruct* recData);

#endif
