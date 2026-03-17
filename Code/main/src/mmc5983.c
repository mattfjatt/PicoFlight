#include "headers/mmc5983.h"

void mmc5983_init()
{
    mmc5983_setup();
}

void mmc5983_setup()
{   

    gpio_init(MMC5983_CS);
    gpio_set_dir(MMC5983_CS, GPIO_OUT);
    gpio_put(MMC5983_CS,1); //CS is set to high, this is the idle state
    sleep_ms(50);

    uint8_t tx_buf[2];
    uint8_t rx_buf[2];


    while(1){
        mmc5983_read_from_register(MMC5983_PRODUCT_ID,tx_buf,rx_buf,2,MMC5983_CS);
        PRINTNUM("Product ID: %u\n", rx_buf[1]);
        sleep_ms(1000);
    }
}

void mmc5983_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin)
{
    //The mask signifies which bits to update. mask = 0b00000001 means leave all bits as-is except maybe bit 0
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    mmc5983_read_from_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf),cs_pin);
    uint8_t value_from_register = rx_buf[1];
    value_from_register &= ~(mask); //This sets all bits we want to update to 0
    uint8_t value_to_register = value_from_register | (bits_to_update & mask);
    tx_buf[1] = value_to_register;
    mmc5983_write_to_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf), cs_pin);
}

void mmc5983_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register | SET_SPI_READ; //Sets the read bit, bit 7, to 1
    gpio_put(cs_pin,0); //Chip select is active low
    spi_write_read_blocking(spi1, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}

void mmc5983_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register; //Leaves the read bit at 0
    gpio_put(cs_pin,0); //Chip select is active low
    spi_write_read_blocking(spi1, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}

