#include "headers/icm45686.h"

//Default sensitivity values are +- 4000dps and +- 32G
static double gyro_sensitivity = 8.2;
static double accel_sensitivity = 1024.0;

static const float CLOCK_DIVIDER = 1.25f;
static const uint16_t PWM_WRAP_VALUE = 3749;


void icm45686_init()
{
    //Max SCLK frequency is 24MHz
    //Initiate the CS pin
    gpio_init(ICM45686_CS);
    gpio_set_dir(ICM45686_CS, GPIO_OUT);
    gpio_put(ICM45686_CS,1); //CS is set to high, this is the idle state
    sleep_ms(50);
    //Read the WHO_AM_I register
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    icm45686_read_from_register(ICM45686_WHO_AM_I,tx_buf,rx_buf,2,ICM45686_CS);
    PRINTNUM("WHO_AM_I = %u\n", rx_buf[1]);

    icm45686_set_power_modes(ICM45686_GYRO_OFF, ICM45686_ACCEL_OFF);
    sleep_ms(50);

    //icm45686_set_rp2350_pwm_signal();
    icm45686_set_rp2350_clock_out();
    sleep_ms(200); //Let clock stabilize
    icm45686_set_clock_source();

    icm45686_set_measurement_ranges(ICM45686_GYRO_FS_1000,ICM45686_ACCEL_FS_2G);
    icm45686_set_odr_frequency(ICM45686_GYRO_ODR_1K6, ICM45686_ACCEL_ODR_1K6);
    icm45686_set_data_endianness();
    icm45686_set_power_modes(ICM45686_GYRO_LOW_NOISE, ICM45686_ACCEL_LOW_NOISE);
    sleep_ms(100); //Takes the gyro 35ms to start
}

void icm45686_set_measurement_ranges(uint8_t gyro_fs, uint8_t accel_fs)
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
        icm45686_read_modify_write_register(ICM45686_GYRO_CONFIG0, gyro_fs, ICM45686_GYRO_FS_MASK,ICM45686_CS);
    }

    if(valid_accel_fs){
        icm45686_read_modify_write_register(ICM45686_ACCEL_CONFIG0, accel_fs, ICM45686_ACCEL_FS_MASK,ICM45686_CS);
    }
}

void icm45686_set_odr_frequency(uint8_t gyro_odr, uint8_t accel_odr)
{
    if( 1 << gyro_odr & (1 << ICM45686_GYRO_ODR_0K8 | 1 << ICM45686_GYRO_ODR_1K6 | 1 << ICM45686_GYRO_ODR_3K2 | 1 << ICM45686_GYRO_ODR_6K4)){
        icm45686_read_modify_write_register(ICM45686_GYRO_CONFIG0, gyro_odr, ICM45686_GYRO_ODR_MASK, ICM45686_CS);
    }else{
        LOG("Invalid gyro ODR\n");
    }

    if( 1 << accel_odr & (1 << ICM45686_ACCEL_ODR_0K8 | 1 << ICM45686_ACCEL_ODR_1K6 | 1 << ICM45686_ACCEL_ODR_3K2 | 1 << ICM45686_ACCEL_ODR_6K4)){
        icm45686_read_modify_write_register(ICM45686_ACCEL_CONFIG0, accel_odr, ICM45686_ACCEL_ODR_MASK, ICM45686_CS);
    }else{
        LOG("Invalid accel ODR\n");
    }
}

void icm45686_set_power_modes(uint8_t gyro_pwr_mode, uint8_t accel_pwr_mode)
{
    if(1 << gyro_pwr_mode & (1 << ICM45686_GYRO_OFF | 1 << ICM45686_GYRO_STANDBY | 1 << ICM45686_GYRO_LOW_NOISE | 1 << ICM45686_GYRO_LOW_POWER)){
        icm45686_read_modify_write_register(ICM45686_PWR_MGMT0, gyro_pwr_mode, ICM45686_GYRO_MODE_MASK, ICM45686_CS);
    }else{
        LOG("Invalid gyro power mode selected\n");
    }

    if(1 << accel_pwr_mode & (1 << ICM45686_ACCEL_OFF | 1 << ICM45686_ACCEL_OFFX | 1 << ICM45686_ACCEL_LOW_NOISE | 1 << ICM45686_ACCEL_LOW_POWER)){
        icm45686_read_modify_write_register(ICM45686_PWR_MGMT0, accel_pwr_mode, ICM45686_ACCEL_MODE_MASK, ICM45686_CS);
    }else{
        LOG("Invalid accel power mode selected\n");
    }
}

void icm45686_set_data_endianness()
{
    icm45686_read_modify_write_indirect_register(ICM45686_IPREG_TOP1, ICM45686_SREG_CTRL, 1 << ICM45686_SREG_DATA_ENDIAN_SEL , 1 << ICM45686_SREG_DATA_ENDIAN_SEL);
}

void icm45686_set_rp2350_clock_out()
{
    gpio_set_function(ICM45686_PICO_CLOCK_PIN, GPIO_FUNC_GPCK);
    gpio_set_drive_strength(ICM45686_PICO_CLOCK_PIN, GPIO_DRIVE_STRENGTH_2MA);
    clock_gpio_init_int_frac16(ICM45686_PICO_CLOCK_PIN,CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,375,0);
}

void icm45686_set_rp2350_pwm_signal()
{
    gpio_set_function(ICM45686_PICO_CLOCK_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(ICM45686_PICO_CLOCK_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, CLOCK_DIVIDER);
    pwm_config_set_wrap(&config, PWM_WRAP_VALUE);
    pwm_init(slice, &config, true);
    pwm_set_enabled(slice, true);
    pwm_set_chan_level(slice, ICM45686_PICO_CLOCK_PIN % 2, PWM_WRAP_VALUE/2); //Set to 50% duty
}

void icm45686_read_indirect_register(uint16_t bank, uint8_t ireg, uint8_t* ireg_value)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    if( bank == ICM45686_IMEM_SRAM ||
        bank == ICM45686_IPREG_BAR ||
        bank == ICM45686_IPREG_SYS1 ||
        bank == ICM45686_IPREG_SYS2 ||
        bank == ICM45686_IPREG_TOP1){
        uint16_t ireg_16bit_add = bank + ireg;
        uint8_t ireg_7_0  = (uint8_t)(ireg_16bit_add & 0x00FF);
        uint8_t ireg_15_8 = (uint8_t)(ireg_16bit_add >> 8);
        tx_buf[1] = ireg_7_0;
        icm45686_write_to_register(ICM45686_IREG_ADDR_7_0,tx_buf,rx_buf,2,ICM45686_CS);
        tx_buf[1] = ireg_15_8;
        icm45686_write_to_register(ICM45686_IREG_ADDR_15_8,tx_buf,rx_buf,2,ICM45686_CS);
        //Read back the data stored in IREG_DATA
        sleep_ms(1);
        icm45686_read_from_register(ICM45686_IREG_DATA, tx_buf,rx_buf,2,ICM45686_CS);
        *ireg_value = rx_buf[1];
    }else{
        LOG("Invalid internal register bank selected\n");
    }
} 

void icm45686_write_indirect_register(uint16_t bank, uint8_t ireg, uint8_t ireg_value)
{
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];
    if( bank == ICM45686_IMEM_SRAM ||
        bank == ICM45686_IPREG_BAR ||
        bank == ICM45686_IPREG_SYS1 ||
        bank == ICM45686_IPREG_SYS2 ||
        bank == ICM45686_IPREG_TOP1){
        uint16_t ireg_16bit_add = bank + ireg;
        uint8_t ireg_7_0  = (uint8_t)(ireg_16bit_add & 0x00FF);
        uint8_t ireg_15_8 = (uint8_t)(ireg_16bit_add >> 8);
        tx_buf[1] = ireg_15_8;
        tx_buf[2] = ireg_7_0;
        tx_buf[3] = ireg_value;
        icm45686_write_to_register(ICM45686_IREG_ADDR_15_8,tx_buf,rx_buf,4,ICM45686_CS);
    }else{
        LOG("Invalid internal register bank selected\n");
    }
    sleep_ms(1);
}

void icm45686_read_modify_write_indirect_register(uint16_t bank, uint8_t ireg, uint8_t ireg_value, uint8_t mask)
{
    uint8_t value_from_reg;
    uint8_t value_to_reg;
    icm45686_read_indirect_register(bank, ireg, &value_from_reg);
    value_to_reg = value_from_reg & ~mask;
    value_to_reg |= (ireg_value & mask);
    icm45686_write_indirect_register(bank, ireg, value_to_reg);
}

void icm45686_set_clock_source()
{
    //Will use INT2 pin for CLKIN, how to configure it for this? Section 7.3 of ICM45686 user guide:
    //To use pin 9 as CLKIN, the PADS_INT2_CFG_OVRD_VAL must be set to 2 in
    //----->IOC_PAD_SCENARIO_OVRD, user bank 0
    //Must first set the OVRD bit, else it appears the VAL can not be written to
    icm45686_read_modify_write_register(ICM45686_IOC_PAD_SCENARIO_OVRD,
                                        ICM45686_PADS_INT2_CFG_OVRD,
                                        ICM45686_PADS_INT2_CFG_OVRD_MASK,
                                        ICM45686_CS);

    icm45686_read_modify_write_register(ICM45686_IOC_PAD_SCENARIO_OVRD,
                                        ICM45686_PADS_INT2_CFG_OVRD_VAL,
                                        ICM45686_PADS_INT2_CFG_OVRD_VAL_MASK,
                                        ICM45686_CS);

    //Set sync timing control for accel
    icm45686_read_modify_write_indirect_register(ICM45686_IPREG_TOP1,
                                                 ICM45686_SMC_CONTROL_0,
                                                 ICM45686_ACCEL_LP_CLK_SEL,
                                                 ICM45686_ACCEL_LP_CLK_SEL_MASK);

    //Next, to enable the CLKIN function, the RTC_MODE bit must be set to 1 in
    //----->RTC_CONFIG, user bank 0
    icm45686_read_modify_write_register(ICM45686_RTC_CONFIG,
                                        ICM45686_RTC_MODE,
                                        ICM45686_RTC_MODE_MASK,
                                        ICM45686_CS);

    
    //I3C STC and CLKIN use the same interpolator but I3C has higher priority. To use CLKIN, I3C_STC_MODE must be set to 0 on
    //----->SIFS_I3C_STC_CFG, user bank IPREG_TOP1
    icm45686_read_modify_write_indirect_register(ICM45686_IPREG_TOP1,
                                                 ICM45686_SIFS_I3C_STC_CFG,
                                                 ICM45686_I3C_STC_MODE,
                                                 ICM45686_I3C_STC_MODE_MASK);

    //ACCEL_SRC_CTRL[1:0] must be set to 0b10 (FIR and interpolator on) in
    //----->IPREG_SYS2_REG_123, user bank IPREG_SYS2
    icm45686_read_modify_write_indirect_register(ICM45686_IPREG_SYS2,
                                                 ICM45686_IPREG_SYS2_REG_123,
                                                 ICM45686_ACCEL_SRC_CTRL,
                                                 ICM45686_ACCEL_SRC_CTRL_MASK);

    //GYRO_SRC_CTRL[1:0] must be set to 0b10 (FIR and interpolator on) in
    //----->IPREG_SYS1_REG_166, user bank IPREG_SYS1
    icm45686_read_modify_write_indirect_register(ICM45686_IPREG_SYS1,
                                                 ICM45686_IPREG_SYS1_REG_166,
                                                 ICM45686_GYRO_SRC_CTRL,
                                                 ICM45686_GYRO_SRC_CTRL_MASK);
}

void icm45686_get_imu_data(double acc[3], double gyr[3])
{
    //12 bytes of relevant data, the buffers need to be 13 bytes long
    uint8_t tx_buf[13];
    uint8_t rx_buf[13];
    uint8_t cs_pin = ICM45686_CS;

    //Get the data. Comes in two's complement
    icm45686_read_from_register(ICM45686_ACCEL_DATA_X1, tx_buf, rx_buf,sizeof(tx_buf),cs_pin);
    uint16_t msb_x = rx_buf[1];
    uint16_t lsb_x = rx_buf[2];
    uint16_t msb_y = rx_buf[3];
    uint16_t lsb_y = rx_buf[4];
    uint16_t msb_z = rx_buf[5];
    uint16_t lsb_z = rx_buf[6];

    int16_t acc_x = (msb_x << 8) | lsb_x;
    int16_t acc_y = (msb_y << 8) | lsb_y;
    int16_t acc_z = (msb_z << 8) | lsb_z;


    acc[0] =   acc_y/accel_sensitivity;
    acc[1] =   acc_x/accel_sensitivity;
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

    double d2r = 3.14159265/180.0;

    gyr[0] =   gyr_y/gyro_sensitivity*d2r;
    gyr[1] =   gyr_x/gyro_sensitivity*d2r;
    gyr[2] = - gyr_z/gyro_sensitivity*d2r;
}

void icm45686_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin)
{
    //The mask signifies which bits to update. mask = 0b00000001 means leave all bits as-is except maybe bit 0
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    icm45686_read_from_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf),cs_pin);
    uint8_t value_from_register = rx_buf[1];
    value_from_register &= ~(mask); //This sets all bits we want to update to 0
    uint8_t value_to_register = value_from_register | (bits_to_update & mask);
    tx_buf[1] = value_to_register;
    icm45686_write_to_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf), cs_pin);
}

void icm45686_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register | SET_SPI_READ; //Sets the read bit, bit 7, to 1
    gpio_put(cs_pin,0); //Chip select is active low
    spi_write_read_blocking(spi0, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}

void icm45686_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register; //Leaves the read bit at 0
    gpio_put(cs_pin,0); //Chip select is active low
    spi_write_read_blocking(spi0, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}
