#include "headers/icm20948.h"

//Default sensitivity values
static double gyro_sensitivity = 131.0;
static double accel_sensitivity = 16384.0;

void icm20948_init()
{
    //On the breakout board (slave) from Adafruit, the SPI pins are as follows
    //MISO: SDO (Serial Data Out), this connects to MISO on RP2350
    //MOSI: SDA (Serial Data In), this connects to MOSI on RP2350
    //CS  : CS
    //SCLK: SCL

    //Max SCLK frequency is 7MHz
    //Initiate the CS pin
    gpio_init(ICM20948_CS);
    gpio_set_dir(ICM20948_CS, GPIO_OUT);
    gpio_put(ICM20948_CS,1); //CS is set to high, this is the idle state

    //Turn off sleep mode
    icm20948_read_modify_write_register(ICM20948_PWR_MGMT_1, 0, 1 << ICM20948_SLEEP,ICM20948_CS);


    //Next, set gyro and acc full scale ranges. These registers are in user bank 2:
    icm20948_set_register_user_bank(ICM20948_USER_BANK_2);
    icm20948_set_measurement_ranges(ICM20948_GYRO_FS_1000, ICM20948_ACCEL_FS_2G);
    icm20948_set_register_user_bank(ICM20948_USER_BANK_0); //Always set user bank back to 0, otherwise can't read sensor registers later
}

void icm20948_set_measurement_ranges(uint8_t gyro_fs, uint8_t accel_fs)
{
    uint8_t valid_gyro_fs = 1;
    uint8_t valid_accel_fs = 1;

    switch (gyro_fs)
    {
    case ICM20948_GYRO_FS_250:
        gyro_sensitivity = 131.0;
        break;

    case ICM20948_GYRO_FS_500:
        gyro_sensitivity = 65.5;
        break;

    case ICM20948_GYRO_FS_1000:
        gyro_sensitivity = 32.8;
        break;

    case ICM20948_GYRO_FS_2000:
        gyro_sensitivity = 16.4;
        break;
    
    default:
        LOG("Invalid gyro FS\n");
        valid_gyro_fs = 0;
        break;
    }

    switch (accel_fs)
    {
    case ICM20948_ACCEL_FS_2G:
        accel_sensitivity = 16384.0; 
        break;
    
    case ICM20948_ACCEL_FS_4G:
        accel_sensitivity = 8192.0; 
        break;

    case ICM20948_ACCEL_FS_8G:
        accel_sensitivity = 4096.0; 
        break;

    case ICM20948_ACCEL_FS_16G:
        accel_sensitivity = 2048.0; 
        break;

    default:
        LOG("Invalid accel FS\n");
        valid_accel_fs = 0;
        break;
    }
    
    if(valid_gyro_fs){
        icm20948_read_modify_write_register(ICM20948_GYRO_CONFIG_1, gyro_fs << ICM20948_GYRO_FS_SEL, ICM20948_GYRO_FS_BITMASK,ICM20948_CS);
    }
    
    if(valid_accel_fs){
        icm20948_read_modify_write_register(ICM20948_ACCEL_CONFIG_1, accel_fs << ICM20948_ACCEL_FS_SEL, ICM20948_ACCEL_FS_BITMASK,ICM20948_CS);
    }
}

void icm20948_set_register_user_bank(uint8_t bank)
{
    //Register bank 127, 0x7F, in any bank is the bank selection register
    if(bank <= 3){
        icm20948_read_modify_write_register(ICM20948_REG_BANK_SEL, bank << 4, 0b11 << 4,ICM20948_CS);
    }else{
        LOG("Invalid USER BANK selected!\n");
    }
}

void icm20948_get_imu_data(double acc[3], double gyr[3])
{
    //12 bytes of relevant data, the buffers need to be 13 bytes long
    uint8_t tx_buf[13];
    uint8_t rx_buf[13];
    uint8_t cs_pin = ICM20948_CS;

    //Get the data. Comes in two's complement
    icm20948_read_from_register(ICM20948_ACCEL_XOUT_H, tx_buf, rx_buf,sizeof(tx_buf),cs_pin);
    uint16_t msb_x = rx_buf[1];
    uint16_t lsb_x = rx_buf[2];
    uint16_t msb_y = rx_buf[3];
    uint16_t lsb_y = rx_buf[4];
    uint16_t msb_z = rx_buf[5];
    uint16_t lsb_z = rx_buf[6];

    int16_t acc_x = (msb_x << 8) | lsb_x;
    int16_t acc_y = (msb_y << 8) | lsb_y;
    int16_t acc_z = (msb_z << 8) | lsb_z;

    //Note that the frame printed on the Adafruit ICM20498 is wrong!
    //Convert to NED frame
    acc[0] =   acc_x/accel_sensitivity;
    acc[1] = - acc_y/accel_sensitivity;
    acc[2] = - acc_z/accel_sensitivity;

    msb_x = rx_buf[7];
    lsb_x = rx_buf[8];
    msb_y = rx_buf[9];
    lsb_y = rx_buf[10];
    msb_z = rx_buf[11];
    lsb_z = rx_buf[12];

    int16_t gyr_x = (msb_x << 8) | lsb_x;
    int16_t gyr_y = (msb_y << 8) | lsb_y;
    int16_t gyr_z = (msb_z << 8) | lsb_z;

    double d2r = 3.14159265/180;
    //Note that the frame printed on the Adafruit ICM20498 is wrong!
    //Convert to NED frame
    gyr[0] =   gyr_x/gyro_sensitivity*d2r;
    gyr[1] = - gyr_y/gyro_sensitivity*d2r;
    gyr[2] = - gyr_z/gyro_sensitivity*d2r;
}

void icm20948_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin)
{
    //The mask signifies which bits to update. mask = 0b00000001 means leave all bits as-is except maybe bit 0
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    icm20948_read_from_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf),cs_pin);
    uint8_t value_from_register = rx_buf[1];
    value_from_register &= ~(mask); //This sets all bits we want to update to 0
    uint8_t value_to_register = value_from_register | (bits_to_update & mask);
    tx_buf[1] = value_to_register;
    icm20948_write_to_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf), cs_pin);
}

void icm20948_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register | SET_SPI_READ; //Sets the read bit, bit 7, to 1
    gpio_put(cs_pin,0); //Chip select is active low
    int num = spi_write_read_blocking(spi0, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}

void icm20948_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register; //Leaves the read bit at 0
    gpio_put(cs_pin,0); //Chip select is active low
    spi_write_read_blocking(spi0, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}
