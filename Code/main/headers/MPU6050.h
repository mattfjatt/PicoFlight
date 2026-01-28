#ifndef MPU6050_H
#define MPU6050_H

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "headers/LinAlg.h"

//Registers
#define MPU6050_I2C_ADDRESS 0x68

#define MPU6050_GYRO_CONFIG 27
#define MPU6050_ACCEL_CONFIG 28

#define MPU6050_ACCEL_START 59 //59 to 64
#define MPU6050_TEMP_START 65  //65 to 66
#define MPU6050_GYRO_START 67  //67 to 72

#define MPU6050_USER_CTRL 106
#define MPU6050_POWER_MANAGEMENT_1 107 //The datasheet register description mentions that a differet clock source should be selected for stability!
#define MPU6050_WHO_AM_I 117

static uint8_t read_buffer[1]; // Buffer for read operation
static uint8_t write_buffer[2]; //Buffer holding some address and a data byte

void MPU6050_init();

void MPU6050_reset();

void MPU6050_setup();

void MPU6050_get_imu_data(double acc[3], double gyr[3]);

void MPU6050_i2c_setup();

void MPU6050_read_byte_register_I2C(uint8_t i2c_dev_add, uint8_t i2c_dev_reg);

void MPU6050_print_binary(uint32_t num);

#endif
