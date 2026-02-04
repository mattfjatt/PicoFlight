#include "headers/Receiver.h"

recStruct receiverData;

//Will use the following pins:
#define GPIO_CH_0 10
#define GPIO_CH_1 11
#define GPIO_CH_2 12
#define GPIO_CH_3 13


void Receiver_gpio_callback(uint gpio, uint32_t events){
    
    switch (gpio)
    {
    case GPIO_CH_0:
        if((GPIO_IRQ_EDGE_RISE & events) && !receiverData.previous_pin_state[0]){
            receiverData.pulse_start[0] = time_us_32();
            receiverData.previous_pin_state[0] = true;
        }else if((GPIO_IRQ_EDGE_FALL & events) && receiverData.previous_pin_state[0]){
            receiverData.pulse_width[0] = time_us_32() - receiverData.pulse_start[0];
            receiverData.previous_pin_state[0] = false;
        }
        break;
    case GPIO_CH_1:
        if((GPIO_IRQ_EDGE_RISE & events) && !receiverData.previous_pin_state[1]){
            receiverData.pulse_start[1] = time_us_32();
            receiverData.previous_pin_state[1] = true;
        }else if((GPIO_IRQ_EDGE_FALL & events) && receiverData.previous_pin_state[1]){
            receiverData.pulse_width[1] = time_us_32() - receiverData.pulse_start[1];
            receiverData.previous_pin_state[1] = false;
        }
        break;
    case GPIO_CH_2:
        if((GPIO_IRQ_EDGE_RISE & events) && !receiverData.previous_pin_state[2]){
            receiverData.pulse_start[2] = time_us_32();
            receiverData.previous_pin_state[2] = true;
        }else if((GPIO_IRQ_EDGE_FALL & events) && receiverData.previous_pin_state[2]){
            receiverData.pulse_width[2] = time_us_32() - receiverData.pulse_start[2];
            receiverData.previous_pin_state[2] = false;
        }
        break;
    case GPIO_CH_3:
        if((GPIO_IRQ_EDGE_RISE & events) && !receiverData.previous_pin_state[3]){
            receiverData.pulse_start[3] = time_us_32();
            receiverData.previous_pin_state[3] = true;
        }else if((GPIO_IRQ_EDGE_FALL & events) && receiverData.previous_pin_state[3]){
            receiverData.pulse_width[3] = time_us_32() - receiverData.pulse_start[3];
            receiverData.previous_pin_state[3] = false;
        }
        break;
    default:
        break;
    }
}

void Receiver_setup_gpio_interrupts_pins()
{
    gpio_init(GPIO_CH_0);
    gpio_set_dir(GPIO_CH_0, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(GPIO_CH_0, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, Receiver_gpio_callback);

    gpio_init(GPIO_CH_1);
    gpio_set_dir(GPIO_CH_1, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(GPIO_CH_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, Receiver_gpio_callback);

    gpio_init(GPIO_CH_2);
    gpio_set_dir(GPIO_CH_2, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(GPIO_CH_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, Receiver_gpio_callback);

    gpio_init(GPIO_CH_3);
    gpio_set_dir(GPIO_CH_3, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(GPIO_CH_3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, Receiver_gpio_callback);
}

void Receiver_init_struct(recStruct* recData){
    //Initialise the struct
    for(int i = 0; i < 4; i++){
        recData->previous_pin_state[i] = false;
        recData->pulse_start[i] = 0;
        recData->pulse_width[i] = 0;
    }
}

void Receiver_init(recStruct* recData)
{
    Receiver_setup_gpio_interrupts_pins();
    Receiver_init_struct(recData);
}

void Receiver_print_data(){
    PRINTNUM("PWM0: %d", receiverData.pulse_width[0]); 
    PRINTNUM(". PWM1: %d", receiverData.pulse_width[1]);
    PRINTNUM(". PWM2: %d", receiverData.pulse_width[2]);
    PRINTNUM(". PWM3: %d\n", receiverData.pulse_width[3]);
}
