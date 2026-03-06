#ifndef MMC5603_H
#define MMC5603_H

#include "headers/logging.h"
#include "headers/pins.h"
#include "headers/bus.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "headers/linalg.h"
#include "headers/optimizer.h"
//This is the magnetometer that will be used. It has a two modes for reading data:
//1: Single-Shot mode: You tell the sensor over i2c to take store a measurement, then a bit will be set to 1 in status 1 when the reading is done.
//2: Continuous mode: As the name implies, you don't need to tell the sensor to take a measurement, it will always store the newest. 

//The breakoutboard from Adafruit has the magnetometer axes printed on the board, however, they are wrong. Consult the datasheet for the MMC5603 instead

//Register map
#define MMC5603_I2C_ADDRESS 0x30

#define MMC5603_XOUT0 0x00
#define MMC5603_XOUT1 0x01

#define MMC5603_YOUT0 0x02
#define MMC5603_YOUT1 0x03

#define MMC5603_ZOUT0 0x04
#define MMC5603_ZOUT1 0x05

#define MMC5603_XOUT2 0x06
#define MMC5603_YOUT2 0x07
#define MMC5603_ZOUT2 0x08

#define MMC5603_TOUT 0x09
#define MMC5603_STATUS1 0x18
#define MMC5603_ODR 0x1A
#define MMC5603_INTERNAL_CONTROL_0 0x1B
#define MMC5603_INTERNAL_CONTROL_1 0x1C
#define MMC5603_INTERNAL_CONTROL_2 0x1D

#define MMC5603_ST_X_TH 0x1E
#define MMC5603_ST_Y_TH 0x1F
#define MMC5603_ST_Z_TH 0x20

#define MMC5603_ST_X 0x27
#define MMC5603_ST_Y 0x28
#define MMC5603_ST_Z 0x29

#define MMC5603_PRODUCT_ID 0x39

//Bit map
#define MMC5603_TAKE_MAG_MEAS 0
#define MMC5603_MEAS_MAG_DONE 6

#define MMC5603_TAKE_TMP_MEAS 1
#define MMC5603_MEAS_TMP_DONE 7

#define MMC_5603_DO_SET 3
#define MMC_5603_DO_RESET 4
#define MMC_5603_AUTO_SR_EN 5

#define MMC5603_CMM_FREQ_EN 7
#define MMC5603_CMM_EN 4

#define MMC5603_SW_RESET 7

#define MMC5603_BW0 0
#define MMC5603_BW1 1

void mmc5603_init();

void mmc5603_setup();

void mmc5603_read_temp();

void mmc5603_gather_samples_for_calib();

void mmc5603_calibrate_magnetometer();

void mmc5603_get_mag_reading(Sample* si);

void mmc5603_get_magnetometer_calib(double theta[9], double correction_matrix[3][3], double correction_vector[3]);

void mmc5603_adjust_mag_vector(double correction_matrix[3][3], double correction_vector[3], Sample si, double m_corr[3]);

void mmc5603_get_corrected_mag_reading(double m_corr[3]);

void i2c_read_register(i2c_inst_t* i2c, uint8_t dev_address, uint8_t reg_address, uint8_t* read_buffer, uint8_t len);

void i2c_write_register(i2c_inst_t* i2c, uint8_t dev_address, uint8_t reg_address, uint8_t reg_value);

#endif
