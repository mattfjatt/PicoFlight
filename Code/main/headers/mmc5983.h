#ifndef MMC5983_H
#define MMC5983_H

#include "headers/logging.h"
#include "headers/pins.h"
#include "headers/bus.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "headers/linalg.h"
#include "headers/optimizer.h"

void mmc5983_init();
void mmc5983_setup();

void mmc5983_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin);

void mmc5983_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

void mmc5983_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

//REGISTERS

#define MMC5983_XOUT0                   0x0
#define MMC5983_XOUT1                   0x1
#define MMC5983_YOUT0                   0x2
#define MMC5983_YOUT1                   0x3
#define MMC5983_ZOUT0                   0x4
#define MMC5983_ZOUT1                   0x5
#define MMC5983_XYZOUT2                 0x6
#define MMC5983_TOUT                    0x7
#define MMC5983_STATUS                  0x8
#define MMC5983_INTERNAL_CONTROL0       0x9
#define MMC5983_INTERNAL_CONTROL1       0xA
#define MMC5983_INTERNAL_CONTROL2       0xB
#define MMC5983_INTERNAL_CONTROL3       0xC
#define MMC5983_PRODUCT_ID              0x2F

//BITS

//MMC5983_STATUS
#define MMC5983_OTP_READ_DONE_MASK      (1 << 4)
#define MMC5983_OTP_READ_DONE           (1 << 4)

#define MMC5983_MEAS_T_DONE_MASK        (1 << 1)
#define MMC5983_MEAS_T_DONE             (1 << 1)

#define MMC5983_MEAS_M_DONE_MASK        (1 << 0)
#define MMC5983_MEAS_M_DONE             (1 << 0)

//MMC5983_INTERNAL_CONTROL0
#define MMC5983_TM_M_MASK               (1 << 0)
#define MMC5983_TM_M                    (1 << 0) //Set to 1 to take magnetic measurement, bit resets to 0 at the end of each measurement

#define MMC5983_TM_T_MASK               (1 << 1)
#define MMC5983_TM_T                    (1 << 1) //Set to 1 to take temperature measurement, bit resets to 0 at the end of each measurement

#define MMC5983_INT_MEAS_DONE_EN_MASK   (1 << 2)
#define MMC5983_INT_MEAS_DONE_EN        (1 << 2) //Set to 1 to enable interrupt for completed measurements. Triggers for both mag and temp 

#define MMC5983_SET_MASK                (1 << 3)
#define MMC5983_SET                     (1 << 3) //Set operation, 500ns

#define MMC5983_RESET_MASK              (1 << 4)
#define MMC5983_RESET                   (1 << 4) //Reset operation, 500ns

#define MMC5983_AUTO_SR_EN_MASK         (1 << 5)
#define MMC5983_AUTO_SR_EN              (1 << 5) //Automatic Set/reset

#define MMC5983_OTP_READ_MASK           (1 << 6)
#define MMC5983_OTP_READ                (1 << 6) //OTP read



//MMC5983_INTERNAL_CONTROL1
#define MMC5983_BW0_MASK                (1 << 0)
#define MMC5983_BW0                     (1 << 0) //Set BW

#define MMC5983_BW1_MASK                (1 << 1)
#define MMC5983_BW1                     (1 << 1) //Set BW

#define MMC5983_X_INHIBIT_MASK          (1 << 2) 
#define MMC5983_X_INHIBIT               (1 << 2) //Writing 1 will disable the X channel

#define MMC5983_YZ_INHIBIT_MASK         (1 << 4 | 1 << 3)
#define MMC5983_YZ_INHIBIT              (1 << 4 | 1 << 3) //Writing 1 to both bits will disable Y and Z channel

#define MMC5983_SW_RST_MASK             (1 << 7)
#define MMC5983_SW_RST                  (1 << 7) //Writing 1 resets the part similar to a power up. Will clear all registers and re-read OTP. Takes 10ms

//MMC5983_INTERNAL_CONTROL2
#define MMC5983_CMM_FREQ_MASK           (1 << 2 | 1 << 1 | 1 << 0) //CMM: Continuous Measurement Mode
#define MMC5983_CMM_FREQ_OFF            (0 << 2 | 0 << 1 | 0 << 0)
#define MMC5983_CMM_FREQ_1_HZ           (0 << 2 | 0 << 1 | 1 << 0)
#define MMC5983_CMM_FREQ_10_HZ          (0 << 2 | 1 << 1 | 0 << 0)
#define MMC5983_CMM_FREQ_20_HZ          (0 << 2 | 1 << 1 | 1 << 0)
#define MMC5983_CMM_FREQ_50_HZ          (1 << 2 | 0 << 1 | 0 << 0)
#define MMC5983_CMM_FREQ_100_HZ         (1 << 2 | 0 << 1 | 1 << 0)
#define MMC5983_CMM_FREQ_200_HZ         (1 << 2 | 1 << 1 | 0 << 0) //BW = 01
#define MMC5983_CMM_FREQ_1000_HZ        (1 << 2 | 1 << 1 | 1 << 0) //BW = 11

#define MMC5983_EN_MASK                 (1 << 3)
#define MMC5983_EN                      (1 << 3) //Writing 1 will enable CMM. In order to enter CMM, CM_FREQ[2:0] cannot be 000

#define MMC5983_PRD_SET_MASK            (1 << 6 | 1 << 5 | 1 << 4) //Determines how often the chip will do a set operation (per measurement?)
#define MMC5983_PRD_SET_1               (0 << 6 | 0 << 5 | 0 << 4)
#define MMC5983_PRD_SET_25              (0 << 6 | 0 << 5 | 1 << 4)
#define MMC5983_PRD_SET_75              (0 << 6 | 1 << 5 | 0 << 4)
#define MMC5983_PRD_SET_100             (0 << 6 | 1 << 5 | 1 << 4)
#define MMC5983_PRD_SET_250             (1 << 6 | 0 << 5 | 0 << 4)
#define MMC5983_PRD_SET_500             (1 << 6 | 0 << 5 | 1 << 4)
#define MMC5983_PRD_SET_1000            (1 << 6 | 1 << 5 | 0 << 4)
#define MMC5983_PRD_SET_2000            (1 << 6 | 1 << 5 | 1 << 4)

#define MMC5983_EN_PRD_SET_MASK         (1 << 7)
#define MMC5983_EN_PRD_SET              (1 << 7) //Writing 1 to this bit enables periodic set

//MMC5983_INTERNAL_CONTROL3
//Contains fucntionality for testing if the sensor is magnetically saturated,
//as well as 3-wire SPI setup

#endif
