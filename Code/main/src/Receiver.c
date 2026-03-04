#include "headers/receiver.h"

recStruct receiver_data;

void receiver_gpio_callback(uint gpio, uint32_t events){
    
    switch (gpio)
    {
    case GPIO_CH_0:
        if((GPIO_IRQ_EDGE_RISE & events) && !receiver_data.previous_pin_state[0]){
            receiver_data.pulse_start[0] = time_us_32();
            receiver_data.previous_pin_state[0] = true;
        }else if((GPIO_IRQ_EDGE_FALL & events) && receiver_data.previous_pin_state[0]){
            receiver_data.pulse_width[0] = time_us_32() - receiver_data.pulse_start[0];
            receiver_data.previous_pin_state[0] = false;
        }
        break;
    case GPIO_CH_1:
        if((GPIO_IRQ_EDGE_RISE & events) && !receiver_data.previous_pin_state[1]){
            receiver_data.pulse_start[1] = time_us_32();
            receiver_data.previous_pin_state[1] = true;
        }else if((GPIO_IRQ_EDGE_FALL & events) && receiver_data.previous_pin_state[1]){
            receiver_data.pulse_width[1] = time_us_32() - receiver_data.pulse_start[1];
            receiver_data.previous_pin_state[1] = false;
        }
        break;
    case GPIO_CH_2:
        if((GPIO_IRQ_EDGE_RISE & events) && !receiver_data.previous_pin_state[2]){
            receiver_data.pulse_start[2] = time_us_32();
            receiver_data.previous_pin_state[2] = true;
        }else if((GPIO_IRQ_EDGE_FALL & events) && receiver_data.previous_pin_state[2]){
            receiver_data.pulse_width[2] = time_us_32() - receiver_data.pulse_start[2];
            receiver_data.previous_pin_state[2] = false;
        }
        break;
    case GPIO_CH_3:
        if((GPIO_IRQ_EDGE_RISE & events) && !receiver_data.previous_pin_state[3]){
            receiver_data.pulse_start[3] = time_us_32();
            receiver_data.previous_pin_state[3] = true;
        }else if((GPIO_IRQ_EDGE_FALL & events) && receiver_data.previous_pin_state[3]){
            receiver_data.pulse_width[3] = time_us_32() - receiver_data.pulse_start[3];
            receiver_data.previous_pin_state[3] = false;
        }
        break;
    default:
        break;
    }
}

void receiver_setup_gpio_interrupts_pins()
{
    gpio_init(GPIO_CH_0);
    gpio_set_dir(GPIO_CH_0, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(GPIO_CH_0, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, receiver_gpio_callback);

    gpio_init(GPIO_CH_1);
    gpio_set_dir(GPIO_CH_1, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(GPIO_CH_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, receiver_gpio_callback);

    gpio_init(GPIO_CH_2);
    gpio_set_dir(GPIO_CH_2, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(GPIO_CH_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, receiver_gpio_callback);

    gpio_init(GPIO_CH_3);
    gpio_set_dir(GPIO_CH_3, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(GPIO_CH_3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, receiver_gpio_callback);
}

void receiver_init_struct(recStruct* rec_data){
    //Initialise the struct
    for(int i = 0; i < 4; i++){
        rec_data->previous_pin_state[i] = false;
        rec_data->pulse_start[i] = 0;
        rec_data->pulse_width[i] = 0;
    }
}

void receiver_init(recStruct* rec_data)
{
    receiver_setup_gpio_interrupts_pins();
    receiver_init_struct(rec_data);
}

void receiver_print_data(){
    PRINTNUM("PWM0: %d", receiver_data.pulse_width[0]); 
    PRINTNUM(". PWM1: %d", receiver_data.pulse_width[1]);
    PRINTNUM(". PWM2: %d", receiver_data.pulse_width[2]);
    PRINTNUM(". PWM3: %d\n", receiver_data.pulse_width[3]);
}
