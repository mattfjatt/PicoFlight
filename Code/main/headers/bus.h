#ifndef BUS_H
#define BUS_H

#include "headers/logging.h"
#include "headers/pins.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define SET_SPI_READ 0x80
#define SET_SPI_WRITE 0x0

void bus_i2c_init();

void bus_spi_init();

#endif
