#ifndef ICM20948_H
#define ICM20948_H
//Couldn't find a breakout board for the ICM42688-P, will instead use the 20948 to get familiar with SPI.
//The breakoutboard from Adafruit has the IMU axes printed on the board, however, they are wrong. Consult the datasheet for the ICM20948 instead. Yes, same problem as with the MMC5603.
//The ICM20948 has four register banks, you can select which bank to use by writing to register address 127, REG_BANK_SEL, in each bank to select bank.

#include "headers/logging.h"
#include "headers/pins.h"
#include "headers/linalg.h"
#include "pico/binary_info.h"
#include "headers/bus.h"

void icm20948_init();

void icm20948_set_measurement_ranges(uint8_t gyro_fs, uint8_t accel_fs);

void icm20948_set_register_user_bank(uint8_t bank);

void icm20948_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

void icm20948_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

void icm20948_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin);

void icm20948_get_imu_data(double acc[3], double gyr[3]);

#define ICM20948_CS 5

//Register map

#define ICM20948_USER_BANK_0 0
#define ICM20948_USER_BANK_1 1
#define ICM20948_USER_BANK_2 2
#define ICM20948_USER_BANK_3 3

//User bank 0
#define ICM20948_WHO_AM_I 0

#define ICM20948_PWR_MGMT_1 0x6

#define ICM20948_ACCEL_XOUT_H 0x2D

#define ICM20948_TEMP_OUT_H 0x39
#define ICM20948_TEMP_OUT_L 0x3A

#define ICM20948_REG_BANK_SEL 0x7F

//User bank 1

//User bank 2
#define ICM20948_GYRO_CONFIG_1 0x1
#define ICM20948_ACCEL_CONFIG_1 0x14

//Bit maps

#define ICM20948_SLEEP 6 //Chip seems to have this bit set, need to clear it at startup
#define ICM20948_DEVICE_RESET 7

#define ICM20948_GYRO_FS_BITMASK 0b00000110
#define ICM20948_GYRO_FS_SEL 1
#define ICM20948_GYRO_FS_250 0b00
#define ICM20948_GYRO_FS_500 0b01
#define ICM20948_GYRO_FS_1000 0b10
#define ICM20948_GYRO_FS_2000 0b11


#define ICM20948_ACCEL_FS_BITMASK 0b00000110
#define ICM20948_ACCEL_FS_SEL 1
#define ICM20948_ACCEL_FS_2G 0b00
#define ICM20948_ACCEL_FS_4G 0b01
#define ICM20948_ACCEL_FS_8G 0b10
#define ICM20948_ACCEL_FS_16G 0b11

#endif
