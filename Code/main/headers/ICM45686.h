#ifndef ICM45686_H
#define ICM45686_H

#include "headers/Logging.h"
#include "headers/Bus.h"
#include "headers/LinAlg.h"
//This is the IMU that will be used later

void ICM45686_init();

void ICM45686_get_imu_data(double acc[3], double gyr[3]);

void ICM45686_set_measurement_ranges(uint8_t gyro_fs, uint8_t accel_fs);

void ICM45686_set_odr_frequency(uint8_t gyro_odr, uint8_t accel_odr);

void ICM45686_set_power_modes(uint8_t gyro_pwr_mode, uint8_t accel_pwr_mode);

void ICM45686_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

void ICM45686_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

void ICM45686_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin);

void ICM45686_access_indirect_register(); //See ICM45686 datasheet section 14 "Indirect register access" for more info

#define ICM45686_CS 6

//Registers
#define ICM45686_ACCEL_DATA_X1 0x00

//Config registers that may be relevant
#define ICM45686_PWR_MGMT0 0x10

//The slew rate for int1 can be set in DRIVE_CONFIG2
#define ICM45686_INT1_CONFIG0 0x16 //Sets int1 source
#define ICM45686_INT1_CONFIG1 0x17 //Not relevant
#define ICM45686_INT1_CONFIG2 0x18 //Configures int1 pin like polarity and drive

#define ICM45686_INT1_STATUS0 0x19 //Might have to read these regs to let the IMU
#define ICM45686_INT1_STATUS1 0x1A //know the interrupt has been acknowledged


#define ICM45686_ACCEL_CONFIG0 0x1B //FS and ODR selection
#define ICM45686_GYRO_CONFIG0 0x1C //FS and ODR selection

#define ICM45686_FIFO_CONFIG0 0x1D
#define ICM45686_FIFO_CONFIG1_0 0x1E
#define ICM45686_FIFO_CONFIG1_1 0x1F
#define ICM45686_FIFO_CONFIG2 0x20
#define ICM45686_FIFO_CONFIG3 0x21
#define ICM45686_FIFO_CONFIG4 0x22

#define ICM45686_ODR_DECIMATE_CONFIG 0x28

#define ICM45686_REG_MISC1 0x35 //MCLK source 

#define ICM45686_INT2_CONFIG0 0x56
#define ICM45686_INT2_CONFIG1 0x57
#define ICM45686_INT2_CONFIG2 0x58

#define ICM45686_INT2_STATUS0 0x59
#define ICM45686_INT2_STATUS1 0x5A

#define ICM45686_WHO_AM_I 0x72

#define ICM45686_IREG_ADDR_15_8 0x7C
#define ICM45686_IREG_ADDR_7_0  0x7D
#define ICM45686_IREG_DATA      0x7E

//Bits and bitmasks

//ACCELEROMETER CONFIG
#define ICM45686_ACCEL_FS_MASK (0b111 << 4)
#define ICM45686_ACCEL_FS_2G   (0b100 << 4)
#define ICM45686_ACCEL_FS_4G   (0b011 << 4)
#define ICM45686_ACCEL_FS_8G   (0b010 << 4)
#define ICM45686_ACCEL_FS_16G  (0b001 << 4)
#define ICM45686_ACCEL_FS_32G  (0b000 << 4)

#define ICM45686_ACCEL_ODR_MASK 0b1111
#define ICM45686_ACCEL_ODR_6K4  0b0011
#define ICM45686_ACCEL_ODR_3K2  0b0100
#define ICM45686_ACCEL_ODR_1K6  0b0101
#define ICM45686_ACCEL_ODR_0K8  0b0110

//GYROSCOPE CONFIG
#define ICM45686_GYRO_FS_MASK  (0b1111 << 4)
#define ICM45686_GYRO_FS_4000  (0b0000 << 4)
#define ICM45686_GYRO_FS_2000  (0b0001 << 4)
#define ICM45686_GYRO_FS_1000  (0b0010 << 4)
#define ICM45686_GYRO_FS_500   (0b0011 << 4)
#define ICM45686_GYRO_FS_250   (0b0100 << 4)
#define ICM45686_GYRO_FS_125   (0b0101 << 4)
#define ICM45686_GYRO_FS_62    (0b0110 << 4) //62.5
#define ICM45686_GYRO_FS_31    (0b0111 << 4) //31.25
#define ICM45686_GYRO_FS_15    (0b1000 << 4) //15.625

#define ICM45686_GYRO_ODR_MASK 0b1111
#define ICM45686_GYRO_ODR_6K4  0b0011
#define ICM45686_GYRO_ODR_3K2  0b0100
#define ICM45686_GYRO_ODR_1K6  0b0101
#define ICM45686_GYRO_ODR_0K8  0b0110

//POWER CONFIG
#define ICM45686_GYRO_MODE_MASK (0b11 << 2)
#define ICM45686_GYRO_OFF       (0b00 << 2)
#define ICM45686_GYRO_STANDBY   (0b01 << 2)
#define ICM45686_GYRO_LOW_POWER (0b10 << 2)
#define ICM45686_GYRO_LOW_NOISE (0b11 << 2)

#define ICM45686_ACCEL_MODE_MASK 0b11
#define ICM45686_ACCEL_OFF       0b00
#define ICM45686_ACCEL_OFFX      0b01 //Does the same as the previous one
#define ICM45686_ACCEL_LOW_POWER 0b10
#define ICM45686_ACCEL_LOW_NOISE 0b11

#endif
