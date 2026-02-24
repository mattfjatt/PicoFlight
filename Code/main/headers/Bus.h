#ifndef BUS_H
#define BUS_H

#include "headers/Logging.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

//10 11 12 13 used by receiver.
//18 19 20 21 used by servo
//8 9 used by i2c0
//26 27 used by i2c1
//2 3 4 5 are free to use for spi0

//SPI. CS pins are set in relevant modules
#define SPI_MOSI 3
#define SPI_MISO 4
#define SPI_SCK  2

#define SET_SPI_READ 0x80
#define SET_SPI_WRITE 0x0


//I2C
//These pins are used such that the ICM20498 can use pins 2 3 4 5, but this will not work on the drone 
//as the pins are hardwired in the pcb! Use the default values for the drone of SDA = 4 and SCL = 5
#define I2C0_SDA 8
#define I2C0_SCL 9

#define I2C1_SDA 26
#define I2C1_SCL 27

void Bus_i2c_init();

void Bus_spi_init();

#endif
