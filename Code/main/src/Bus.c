#include "headers/Bus.h"

void Bus_spi_init()
{
    //Setup spi at 1 MHz
    spi_init(spi0, 1000*1000); //Max seems to be 37.5 Mhz
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);  //RX on master is MISO
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);  //TX on master is MOSI


    //SPI format: 8 bits
    //CPOL: Clock Polarity, CPOL = 0 means active high. 
    //CPHA: Clock Phase, this indicates where the data is sampled. CPHA = 0 means the data is sampled/latched on the rising edge
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

void Bus_i2c_init()
{
    //Setup i2c0 at 400kHz, currently used by MPU6050
    i2c_init(i2c0, 400 * 1000); //i2c0 = i2c_default
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);


    //Setup i2c1 at 400kHz, currently used by MMC5603
    i2c_init(i2c1, 400 * 1000); //Fast mode plus at 1MHz is the fastest supported mode on the RP2350
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA);
    gpio_pull_up(I2C1_SCL);
}
