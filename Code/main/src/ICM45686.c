#include "headers/ICM45686.h"

//Default sensitivity values are +- 4000dps and +- 32G
static double gyro_sensitivity = 8.2;
static double accel_sensitivity = 1024.0;

void ICM45686_init()
{
    //Max SCLK frequency is 24MHz
    //Initiate the CS pin
    gpio_init(ICM45686_CS);
    gpio_set_dir(ICM45686_CS, GPIO_OUT);
    gpio_put(ICM45686_CS,1); //CS is set to high, this is the idle state

    //Read the WHO_AM_I register
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    ICM45686_set_measurement_ranges(ICM45686_GYRO_FS_250,ICM45686_ACCEL_FS_2G);
    ICM45686_set_odr_frequency(ICM45686_GYRO_ODR_0K8, ICM45686_ACCEL_ODR_6K4);
    ICM45686_set_power_modes(ICM45686_GYRO_LOW_NOISE, ICM45686_ACCEL_LOW_NOISE);
    ICM45686_access_indirect_register(); //Sets to Big Endian data format 

    double acc[3];
    double gyr[3];
    

    while(1){
        //PRINTNUM("Val = %u\n", rx_buf[1]);
        ICM45686_get_imu_data(acc,gyr);
        LinAlg_printvec(3,gyr);
        sleep_ms(50);
    }

}

void ICM45686_set_measurement_ranges(uint8_t gyro_fs, uint8_t accel_fs)
{
    uint8_t valid_gyro_fs = 1;
    uint8_t valid_accel_fs = 1;

    switch (gyro_fs)
    {
    case ICM45686_GYRO_FS_15:
        gyro_sensitivity = 2097.2;
        break;

    case ICM45686_GYRO_FS_31:
        gyro_sensitivity = 1048.6;
        break;

    case ICM45686_GYRO_FS_62:
        gyro_sensitivity = 524.3;
        break;

    case ICM45686_GYRO_FS_125:
        gyro_sensitivity = 262.0;
        break;

    case ICM45686_GYRO_FS_250:
        gyro_sensitivity = 131.0;
        break;

    case ICM45686_GYRO_FS_500:
        gyro_sensitivity = 65.5;
        break;

    case ICM45686_GYRO_FS_1000:
        gyro_sensitivity = 32.8;
        break;
    
    case ICM45686_GYRO_FS_2000:
        gyro_sensitivity = 16.4;
        break;

    case ICM45686_GYRO_FS_4000:
        gyro_sensitivity = 8.2;
        break;
    
    default:
        LOG("Invalid gyro FS\n");
        valid_gyro_fs = 0;
        break;
    }

    switch (accel_fs)
    {
    case ICM45686_ACCEL_FS_2G:
        accel_sensitivity = 16384.0; 
        break;
    
    case ICM45686_ACCEL_FS_4G:
        accel_sensitivity = 8192.0; 
        break;

    case ICM45686_ACCEL_FS_8G:
        accel_sensitivity = 4096.0; 
        break;

    case ICM45686_ACCEL_FS_16G:
        accel_sensitivity = 2048.0; 
        break;
    
    case ICM45686_ACCEL_FS_32G:
        accel_sensitivity = 1024.0; 
        break;

    default:
        LOG("Invalid accel FS\n");
        valid_accel_fs = 0;
        break;
    }


    if(valid_gyro_fs){
        ICM45686_read_modify_write_register(ICM45686_GYRO_CONFIG0, gyro_fs, ICM45686_GYRO_FS_MASK,ICM45686_CS);
    }

    if(valid_accel_fs){
        ICM45686_read_modify_write_register(ICM45686_ACCEL_CONFIG0, accel_fs, ICM45686_ACCEL_FS_MASK,ICM45686_CS);
    }
}

void ICM45686_set_odr_frequency(uint8_t gyro_odr, uint8_t accel_odr)
{
    if( 1 << gyro_odr & (1 << ICM45686_GYRO_ODR_0K8 | 1 << ICM45686_GYRO_ODR_1K6 | 1 << ICM45686_GYRO_ODR_3K2 | 1 << ICM45686_GYRO_ODR_6K4)){
        ICM45686_read_modify_write_register(ICM45686_GYRO_CONFIG0, gyro_odr, ICM45686_GYRO_ODR_MASK, ICM45686_CS);
    }else{
        LOG("Invalid gyro ODR\n");
    }

    if( 1 << accel_odr & (1 << ICM45686_ACCEL_ODR_0K8 | 1 << ICM45686_ACCEL_ODR_1K6 | 1 << ICM45686_ACCEL_ODR_3K2 | 1 << ICM45686_ACCEL_ODR_6K4)){
        ICM45686_read_modify_write_register(ICM45686_ACCEL_CONFIG0, accel_odr, ICM45686_ACCEL_ODR_MASK, ICM45686_CS);
    }else{
        LOG("Invalid accel ODR\n");
    }
}

void ICM45686_set_power_modes(uint8_t gyro_pwr_mode, uint8_t accel_pwr_mode)
{
    if(1 << gyro_pwr_mode & (1 << ICM45686_GYRO_OFF | 1 << ICM45686_GYRO_STANDBY | 1 << ICM45686_GYRO_LOW_NOISE | 1 << ICM45686_GYRO_LOW_POWER)){
        ICM45686_read_modify_write_register(ICM45686_PWR_MGMT0, gyro_pwr_mode, ICM45686_GYRO_MODE_MASK, ICM45686_CS);
    }else{
        LOG("Invalid gyro power mode selected\n");
    }

    if(1 << accel_pwr_mode & (1 << ICM45686_ACCEL_OFF | 1 << ICM45686_ACCEL_OFFX | 1 << ICM45686_ACCEL_LOW_NOISE | 1 << ICM45686_ACCEL_LOW_POWER)){
        ICM45686_read_modify_write_register(ICM45686_PWR_MGMT0, accel_pwr_mode, ICM45686_ACCEL_MODE_MASK, ICM45686_CS);
    }else{
        LOG("Invalid accel power mode selected\n");
    }
}

void ICM45686_access_indirect_register()
{
    //Host indirect access register (IREG) can only be addressed using an internal 16 bit address
    //A minimum wait-time of 4us between read/writes is required.

    //For starters, we want to access register SREG_CTRL in bank IPREG_TOP1. To do this,
    //add the base address 0xA200 to the 8 bit register address for SREG_CTRL and write the result to
    //IREG_ADDR_7_0, IREG_ADDR_15_8
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];

    uint16_t ipreg_top1_offset = 0xA200;
    uint16_t sreg_ctrl_16bit = 0x67 + ipreg_top1_offset; //41575
    uint8_t ireg_7_0 = (uint8_t)(sreg_ctrl_16bit & 0x00FF);
    uint8_t ireg_15_8 = (uint8_t)(sreg_ctrl_16bit >> 8);
    uint8_t ireg_value = 0b10;
    tx_buf[1] = ireg_15_8;
    tx_buf[2] = ireg_7_0;
    tx_buf[3] = ireg_value;
    
    //Set to Big Endian format
    ICM45686_write_to_register(ICM45686_IREG_ADDR_15_8,tx_buf,rx_buf,4,ICM45686_CS);

    //Want to read the data in SREG_CTRL
    ICM45686_read_modify_write_register(ICM45686_IREG_ADDR_7_0, ireg_7_0, 0xFF, ICM45686_CS);
    ICM45686_read_modify_write_register(ICM45686_IREG_ADDR_15_8, ireg_15_8, 0xFF, ICM45686_CS);
    //Now the data is loaded into IREG_DATA and we can read the reg from this location
    ICM45686_read_from_register(ICM45686_IREG_DATA, tx_buf, rx_buf, 2, ICM45686_CS);
    PRINTNUM("SREG_CTRL = %u\n", rx_buf[1]);

}

void ICM45686_get_imu_data(double acc[3], double gyr[3])
{
    //12 bytes of relevant data, the buffers need to be 13 bytes long
    uint8_t tx_buf[13];
    uint8_t rx_buf[13];
    uint8_t cs_pin = ICM45686_CS;

    //Get the data. Comes in two's complement
    ICM45686_read_from_register(ICM45686_ACCEL_DATA_X1, tx_buf, rx_buf,sizeof(tx_buf),cs_pin);
    uint16_t MSB_x = rx_buf[1];
    uint16_t LSB_x = rx_buf[2];
    uint16_t MSB_y = rx_buf[3];
    uint16_t LSB_y = rx_buf[4];
    uint16_t MSB_z = rx_buf[5];
    uint16_t LSB_z = rx_buf[6];

    int16_t ACC_x = (MSB_x << 8) | LSB_x;
    int16_t ACC_y = (MSB_y << 8) | LSB_y;
    int16_t ACC_z = (MSB_z << 8) | LSB_z;


    acc[0] =   ACC_x/accel_sensitivity;
    acc[1] =   ACC_y/accel_sensitivity;
    acc[2] =   ACC_z/accel_sensitivity;

    MSB_x = rx_buf[7];
    LSB_x = rx_buf[8];
    MSB_y = rx_buf[9];
    LSB_y = rx_buf[10];
    MSB_z = rx_buf[11];
    LSB_z = rx_buf[12];

    int16_t GYR_x = (MSB_x << 8) | LSB_x;
    int16_t GYR_y = (MSB_y << 8) | LSB_y;
    int16_t GYR_z = (MSB_z << 8) | LSB_z;

    double d2r = 1.0;//3.14159265/180;

    gyr[0] =   GYR_x/gyro_sensitivity*d2r;
    gyr[1] =   GYR_y/gyro_sensitivity*d2r;
    gyr[2] =   GYR_z/gyro_sensitivity*d2r;
}

void ICM45686_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin)
{
    //The mask signifies which bits to update. mask = 0b00000001 means leave all bits as-is except maybe bit 0
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    ICM45686_read_from_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf),cs_pin);
    uint8_t value_from_register = rx_buf[1];
    value_from_register &= ~(mask); //This sets all bits we want to update to 0
    uint8_t value_to_register = value_from_register | (bits_to_update & mask);
    tx_buf[1] = value_to_register;
    ICM45686_write_to_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf), cs_pin);
}

void ICM45686_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register | SET_SPI_READ; //Sets the read bit, bit 7, to 1
    gpio_put(cs_pin,0); //Chip select is active low
    int num = spi_write_read_blocking(spi0, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}

void ICM45686_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register; //Leaves the read bit at 0
    gpio_put(cs_pin,0); //Chip select is active low
    spi_write_read_blocking(spi0, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}
