#ifndef BMP388_H
#define BMP388_H

#include "headers/logging.h"
#include "headers/pins.h"
#include "headers/bus.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "headers/linalg.h"

//TODO: Right now each sensor module has its own spi-comms functions, put this into bus.c/h 

typedef struct{
    float p11;
    float p10;
    float p9;
    float p8;
    float p7;
    float p6;
    float p5;
    float p4;
    float p3;
    float p2;
    float p1;

    //Temperature compensation
    float t3;
    float t2;
    float t1;

    float t_lin;
    
    //TODO:
    //The pressure comp depends on the temp, as such we may want to ensure that a temp measurement has been performed
    //before computing the pressure. This bit can be set to true when a temp conversion has been done, then set it 
    //back to false after a pressure conversion? Maybe it is  simpler to not have the two conversions in separate functions?
    int t_measured_flag;

}bmpCompensationParameters;

void bmp388_setup();

void bmp388_init();

void bmp388_get_raw_pressure_temp(uint32_t* pres_raw, uint32_t* temp_raw);

void bmp388_get_compensated_pressure_temp(float* pres_comp, float* temp_comp, bmpCompensationParameters* comp_params);

float bmp388_compensate_temperature(uint32_t uncomp_temp, bmpCompensationParameters* comp_params);

float bmp388_compensate_pressure(uint32_t uncomp_press, bmpCompensationParameters* comp_params);

void bmp388_load_compensation_params(bmpCompensationParameters* comp);

void bmp388_int_callback(uint gpio, uint32_t events);

void bmp388_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin);

void bmp388_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

void bmp388_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

//CSB has integrated pull-up resistor

//REGISTERS
#define BMP388_CMD              0x7E
#define BMP388_CONFIG           0x1F
#define BMP388_ODR              0x1D
#define BMP388_OSR              0x1C //Oversampling selection
#define BMP388_PWR_CTRL         0x1B //mode, temp_en, press_en
#define BMP388_IF_CONF          0x1A
#define BMP388_INT_CTRL         0x19 //Int pin setups etc
#define BMP388_FIFO_CONFIG_2    0x18
#define BMP388_FIFO_CONFIG_1    0x17
#define BMP388_FIFO_WTM_1       0x16
#define BMP388_FIFO_WTM_0       0x15
#define BMP388_FIFO_DATA        0x14
#define BMP388_FIFO_LENGTH_1    0x13
#define BMP388_FIFO_LENGTH_0    0x12
#define BMP388_INT_STATUS       0x11 //Contains info about which source that triggered an interrupt
#define BMP388_EVENT            0x10

#define BMP388_SENSOR_TIME_2    0x0E
#define BMP388_SENSOR_TIME_1    0x0D
#define BMP388_SENSOR_TIME_0    0x0C

#define BMP388_DATA_5           0x09
#define BMP388_DATA_4           0x08
#define BMP388_DATA_3           0x07
#define BMP388_DATA_2           0x06
#define BMP388_DATA_1           0x05
#define BMP388_DATA_0           0x04
#define BMP388_STATUS           0x03
#define BMP388_ERR_REG          0x02
#define BMP388_CHIP_ID          0x00

//Registers for trimming coefficients
#define BMP388_NVM_PAR_P11_7_0  0x45 //8 bit signed

#define BMP388_NVM_PAR_P10_7_0  0x44 //8 bit signed

#define BMP388_NVM_PAR_P9_15_8  0x43 //16 bit signed
#define BMP388_NVM_PAR_P9_7_0   0x42 

#define BMP388_NVM_PAR_P8_7_0   0x41 //8 bit signed

#define BMP388_NVM_PAR_P7_7_0   0x40 //8 bit signed

#define BMP388_NVM_PAR_P6_15_8  0x3F //16 bit unsigned
#define BMP388_NVM_PAR_P6_7_0   0x3E

#define BMP388_NVM_PAR_P5_15_8  0x3D //16 bit unsigned
#define BMP388_NVM_PAR_P5_7_0   0x3C

#define BMP388_NVM_PAR_P4_7_0   0x3B //8 bit signed

#define BMP388_NVM_PAR_P3_7_0   0x3A //8 bit signed

#define BMP388_NVM_PAR_P2_15_8  0x39 //16 bit signed
#define BMP388_NVM_PAR_P2_7_0   0x38 

#define BMP388_NVM_PAR_P1_15_8  0x37 //16 bit signed
#define BMP388_NVM_PAR_P1_7_0   0x36



#define BMP388_NVM_PAR_T3_7_0   0x35 //8 bit signed

#define BMP388_NVM_PAR_T2_15_8  0x34 //16 bit unsigned
#define BMP388_NVM_PAR_T2_7_0   0x33

#define BMP388_NVM_PAR_T1_15_8  0x32 //16 bit unsigned
#define BMP388_NVM_PAR_T1_7_0   0x31

#define BMP388_COMP_PARAMS_START BMP388_NVM_PAR_T1_7_0

//BITS

//BMP388_PWR_CTRL
#define BMP388_PRESS_EN_MASK    (1 << 0)
#define BMP388_PRESS_EN         (1 << 0)

#define BMP388_TEMP_EN_MASK     (1 << 1)
#define BMP388_TEMP_EN          (1 << 1)

#define BMP388_MODE_MASK        (1 << 5 | 1 << 4)
#define BMP388_MODE             (1 << 5 | 1 << 4) //Set the mode  here. 11 = normal mode


//BMP388_INT_CTRL
#define BMP388_INT_OD_MASK      (1 << 0) //Output drive config
#define BMP388_INT_OD           (1 << 0) //0: push pull. 1: open drain

#define BMP388_INT_LEVEL_MASK   (1 << 1) //level of int pin
#define BMP388_INT_LEVEL        (1 << 1) //0: active low. 1: active high

#define BMP388_INT_LATCH_MASK   (1 << 2) //Latching of interrupts for INT pin and INT_STATUS reg
#define BMP388_INT_LATCH        (1 << 2) //0: disabled. 1: enabled

#define BMP388_FWTM_EN_MASK     (1 << 3) //Enable FIFO watermark reached for INT pin and INT_STATUS
#define BMP388_FWTM_EN          (1 << 3) //0: disabled. 1: enabled

#define BMP388_FFULL_EN_MASK    (1 << 4) //Enable FIFO full interrupt for INT pin and INT_STATUS
#define BMP388_FFULL_EN         (1 << 4) //0: disabled. 1: enabled

#define BMP388_DRDY_EN_MASK     (1 << 6) //Enable temp/pressure data ready interrupt for INT pin and INT_STATUS
#define BMP388_DRDY_EN          (1 << 6) //0: disabled. 1: enabled


//BMP388_INT_STATUS
#define BMP388_FWM_INT          (1 << 0) //FIFO watermark interrupt

#define BMP388_FFULL_INT        (1 << 1) //FIFO full interrupt

#define BMP388_DRDY             (1 << 3) //Data ready interrupt


#endif
