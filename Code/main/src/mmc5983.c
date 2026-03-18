#include "headers/mmc5983.h"

static volatile uint32_t int_counter = 0;
static volatile uint32_t pin = 0;
static volatile uint32_t data_ready = 0;

//This source file is not ready, but all pin-functionality of the sensor has been confirmed 
//to work and can now be routed on the PCB

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

    //Control 0 setup
    mmc5983_read_modify_write_register(MMC5983_INTERNAL_CONTROL0, MMC5983_INT_MEAS_DONE_EN |
                                                                  MMC5983_AUTO_SR_EN,
                                                                  MMC5983_INT_MEAS_DONE_EN_MASK |
                                                                  MMC5983_AUTO_SR_EN_MASK,
                                                                  MMC5983_CS);
    
    //Control 1 setup
    mmc5983_read_modify_write_register(MMC5983_INTERNAL_CONTROL1, MMC5983_BW0 |
                                                                  MMC5983_BW1,
                                                                  MMC5983_BW0_MASK |
                                                                  MMC5983_BW1_MASK |
                                                                  MMC5983_X_INHIBIT_MASK, //Need to write 0 to x-inhibit for some reason, 
                                                                  MMC5983_CS);

    


    //Control 2 setup
    mmc5983_read_modify_write_register(MMC5983_INTERNAL_CONTROL2, MMC5983_CMM_FREQ_100_HZ |
                                                                  MMC5983_EN_PRD_SET |
                                                                  MMC5983_PRD_SET_75 |
                                                                  MMC5983_CMM_EN,
                                                                  MMC5983_CMM_FREQ_MASK |
                                                                  MMC5983_EN_PRD_SET_MASK |
                                                                  MMC5983_PRD_SET_MASK |
                                                                  MMC5983_CMM_EN_MASK,
                                                                  MMC5983_CS);
    
    sleep_ms(50);
  

    gpio_init(MMC5983_INT_PIN);
    gpio_set_dir(MMC5983_INT_PIN, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(MMC5983_INT_PIN, GPIO_IRQ_EDGE_RISE, true, mmc5983_int_callback);

    mmc5983_get_mag_reading(NULL);
    mmc5983_clear_mag_interrupt();

    while(1){

        if(data_ready){
            data_ready = 0;
            //Need to clear the interrupt in Status 
            if(!mmc5983_mag_measurement_ready()){
                LOG("Interrupt triggered but data not ready in sensor!\n");
            }
            mmc5983_clear_mag_interrupt();
            mmc5983_get_mag_reading(NULL);
            PRINTNUM("Ints = %ld\n", int_counter);
        }
        sleep_ms(1);
    }
}

void mmc5983_int_callback(uint gpio, uint32_t events)
{
    int_counter++;
    pin = gpio;
    data_ready = 1;
}

int mmc5983_mag_measurement_ready()
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    mmc5983_read_from_register(MMC5983_STATUS, tx_buf, rx_buf, sizeof(tx_buf), MMC5983_CS);
    if(rx_buf[1] & MMC5983_MEAS_M_DONE){
        return 1;
    }else{
        return 0;
    }
}

int mmc5983_temp_measurement_ready()
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    mmc5983_read_from_register(MMC5983_STATUS, tx_buf, rx_buf, sizeof(tx_buf), MMC5983_CS);
    if(rx_buf[1] & MMC5983_MEAS_T_DONE){
        return 1;
    }else{
        return 0;
    }
}

void mmc5983_clear_mag_interrupt()
{
    //Writing 1 to this bit will clear the mag ready interrupt, I assume this means resetting the int pin to low again.
    //If this function is not called after data is ready, the int pin remains at 3 volts
    mmc5983_read_modify_write_register(MMC5983_STATUS, MMC5983_MEAS_M_DONE, MMC5983_MEAS_M_DONE_MASK, MMC5983_CS);
}

void mmc5983_get_mag_reading(Sample* si)
{
    //Want to read seven bytes for 18bit xyz measurements:
    uint8_t tx_buf[8];
    uint8_t rx_buf[8];
    mmc5983_read_from_register(MMC5983_XOUT0, tx_buf, rx_buf,sizeof(tx_buf), MMC5983_CS);
    
    if(!si){
        return;
    }

    uint32_t lsb_x, msb_x;
    uint32_t lsb_y, msb_y;
    uint32_t lsb_z, msb_z;
    uint32_t lower2_xyz; //lower2_xyz = [XOUT[1:0], YOUT[1:0], ZOUT[1:0], 0 0]

    msb_x = rx_buf[1];
    lsb_x = rx_buf[2];
    msb_y = rx_buf[3];
    lsb_y = rx_buf[4];
    msb_z = rx_buf[5];
    lsb_z = rx_buf[6];
    lower2_xyz = rx_buf[7];



    uint32_t raw_x = (msb_x << 10) | (lsb_x << 2) | ((lower2_xyz >> 6) & 3);
    uint32_t raw_y = (msb_y << 10) | (lsb_y << 2) | ((lower2_xyz >> 4) & 3);
    uint32_t raw_z = (msb_z << 10) | (lsb_z << 2) | ((lower2_xyz >> 2) & 3);
    double test_arr[3] = {(double)raw_x,(double)raw_y,(double)raw_z};
    //linalg_printvec(3,test_arr);
    //Need to convert this to NED frame as with the IMU.
    //Note that the frame printed on the Adafruit MMC5603 is wrong!
    si->x =   ((int32_t)raw_x - 524288)*0.0625/1000.0;
    si->y = - ((int32_t)raw_y - 524288)*0.0625/1000.0;
    si->z = - ((int32_t)raw_z - 524288)*0.0625/1000.0;
}

void mmc5983_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin)
{
    //The mask signifies which bits to update. mask = 0b00000001 means leave all bits as-is except maybe bit 0
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};

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
    tx_buf[0] = dev_register & 0x7F; //Explicitly sets the read bit to 0
    gpio_put(cs_pin,0); //Chip select is active low
    //spi_write_read_blocking(spi1, tx_buf, rx_buf, n_bytes);
    spi_write_blocking(spi1, tx_buf, n_bytes);
    gpio_put(cs_pin,1);
}

