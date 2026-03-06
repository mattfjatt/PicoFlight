#ifndef MPU6050_H
#define MPU6050_H

#include "headers/logging.h"
#include "headers/pins.h"
#include "headers/config.h"
#include "pico/stdlib.h"
#include "headers/logging.h"
#include "headers/bus.h"
#include "pico/binary_info.h"
#include "headers/linalg.h"

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

void mpu6050_init();

void mpu6050_reset();

void mpu6050_setup();

void mpu6050_get_imu_data(double acc[3], double gyr[3]);

void mpu6050_six_point_accel_correction(double acc[3]); //This is a rough calibration that can be applied to the accelerometer data

void mpu6050_read_byte_register_I2C(uint8_t i2c_dev_add, uint8_t i2c_dev_reg);

void mpu6050_print_binary(uint32_t num);

#endif
