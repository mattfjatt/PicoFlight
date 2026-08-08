#include "headers/icm45686.h"


static const float CLOCK_DIVIDER = 1.25f;
static const uint16_t PWM_WRAP_VALUE = 3749;

static volatile uint32_t int1_counter = 0;
static volatile uint32_t pin = 0;
static volatile uint8_t fifo_flag = 0;

imu imu0;
imu imu1;
imu imu2;

uint8_t test_fifo_buf_tx[3000];
uint8_t test_fifo_buf_rx[3000];



void icm45686_int1_callback(uint gpio, uint32_t events)
{
    
    if(gpio == imu0.imu_pins.int_pin){
        imu0.imu_data.interrupt1_flag = true;
    }
    
    if(gpio == imu1.imu_pins.int_pin){
        imu1.imu_data.interrupt1_flag = true;
    }
    
    if(gpio == imu2.imu_pins.int_pin){
        imu2.imu_data.interrupt1_flag = true;
    }

    int1_counter++;
    pin = gpio;
  
}

void icm45686_configure_int_for_fifo(const imu* imu_dev)
{
    icm45686_read_modify_write_register(ICM45686_INT1_CONFIG0, 0x0, 0xFF, imu_dev->imu_pins.cs_pin); // Zero the register

    icm45686_read_modify_write_register(ICM45686_INT1_CONFIG1, 0x0, 0xFF, imu_dev->imu_pins.cs_pin); // Zero the register

    icm45686_read_modify_write_register(ICM45686_INT1_CONFIG2, 0x0, 0x07, imu_dev->imu_pins.cs_pin); // Zero the register

    icm45686_read_modify_write_register(ICM45686_INT1_CONFIG0, ICM45686_INT1_STATUS_EN_FIFO_THS, ICM45686_INT1_STATUS_EN_FIFO_THS_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_INT1_CONFIG2, ICM45686_INT1_DRIVE | 
                                                               ICM45686_INT1_MODE | 
                                                               ICM45686_INT1_POLARITY, 
                                                               ICM45686_INT1_DRIVE_MASK | 
                                                               ICM45686_INT1_MODE_MASK | 
                                                               ICM45686_INT1_POLARITY_MASK, 
                                                               imu_dev->imu_pins.cs_pin); // INT1 pin behavior setup
}

void icm45686_init()
{

    icm45686_set_rp2350_clock_out(); // This is not set on a per-imu basis as they all share clock, will keep it separate from per-imu setup
    sleep_ms(200);                   // Let clock stabilize

    icm45686_configure_pins(&imu0, PF_ICM45686_0_CS, PF_ICM45686_0_INT);
    icm45686_configure_pins(&imu1, PF_ICM45686_1_CS, PF_ICM45686_1_INT);
    icm45686_configure_pins(&imu2, PF_ICM45686_2_CS, PF_ICM45686_2_INT);

    icm45686_set_cs_pin(&imu0);
    icm45686_set_cs_pin(&imu1);
    icm45686_set_cs_pin(&imu2);
    sleep_ms(50);

    icm45686_configure_default_config(&imu0, imu_fifo);
    icm45686_configure_default_config(&imu1, imu_fifo);
    icm45686_configure_default_config(&imu2, imu_fifo);

    // Apply the config struct
    icm45686_set_config(&imu0);
    icm45686_set_config(&imu1);
    icm45686_set_config(&imu2);

    sleep_ms(2500);

    int test = 0;
    //Flush the fifo
    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG2, ICM45686_FIFO_FLUSH, ICM45686_FIFO_FLUSH_MASK, imu0.imu_pins.cs_pin);
    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG2, ICM45686_FIFO_FLUSH, ICM45686_FIFO_FLUSH_MASK, imu1.imu_pins.cs_pin);
    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG2, ICM45686_FIFO_FLUSH, ICM45686_FIFO_FLUSH_MASK, imu2.imu_pins.cs_pin);
    while (1)
    {
        // sleep_ms(10);
        // icm45686_get_imu_data(&imu0);
        // linalg_printvec(3, imu0.imu_data.gyro_data);

        if(imu0.imu_data.interrupt1_flag){
            imu0.imu_data.interrupt1_flag = false;
            test++;
            icm45686_get_fifo_buffer(&imu0);
            PRINT("IMU0\n");
            for(int i = 0; i < ICM45686_FIFO_PACKET_COUNT; i++){
                // linalg_vecscalmult(3,imu0.imu_data.fifo_array_20[i].gyro_data, imu0.imu_data.fifo_array_20[i].gyro_data, 180.0/3.14);
                // linalg_printvec(3, imu0.imu_data.fifo_array_20[i].gyro_data);
                PRINTNUM("i = %u: ", i);
                PRINTNUM("t = %u\n", (uint16_t)(imu0.imu_data.fifo_array_20[i].sample_timestamp));
            }
        }

        if(imu1.imu_data.interrupt1_flag){
            imu1.imu_data.interrupt1_flag = false;
            test++;
            icm45686_get_fifo_buffer(&imu1);

            PRINT("IMU1\n");
            for(int i = 0; i < ICM45686_FIFO_PACKET_COUNT; i++){
                // linalg_vecscalmult(3,imu0.imu_data.fifo_array_20[i].gyro_data, imu0.imu_data.fifo_array_20[i].gyro_data, 180.0/3.14);
                // linalg_printvec(3, imu0.imu_data.fifo_array_20[i].gyro_data);
                PRINTNUM("i = %u: ", i);
                PRINTNUM("t = %u\n", (uint16_t)(imu1.imu_data.fifo_array_20[i].sample_timestamp));
            }
        }

        if(imu2.imu_data.interrupt1_flag){
            imu2.imu_data.interrupt1_flag = false;
            test-=2;
            icm45686_get_fifo_buffer(&imu2);

            PRINT("IMU2\n");
            for(int i = 0; i < ICM45686_FIFO_PACKET_COUNT; i++){
                // linalg_vecscalmult(3,imu0.imu_data.fifo_array_20[i].gyro_data, imu0.imu_data.fifo_array_20[i].gyro_data, 180.0/3.14);
                // linalg_printvec(3, imu0.imu_data.fifo_array_20[i].gyro_data);
                PRINTNUM("i = %u: ", i);
                PRINTNUM("t = %u\n", (uint16_t)(imu2.imu_data.fifo_array_20[i].sample_timestamp));
            }
        }
        PRINTNUM("test = %d\n", test);
    }
}

void icm45686_parse_fifo_frame(imu *imu_dev, uint8_t bytes[], uint16_t byte_count)
{
    for(int i = 0; i < byte_count/imu_dev->imu_data.fifo_packet_size; i++){
        int j = imu_dev->imu_data.fifo_packet_size * i + 1; //Must start at 1 because the first byte is a dummy

        //Header
        imu_dev->imu_data.fifo_array_20[i].header = bytes[j];

        //Accel upper and middle
        uint32_t accel_x_upper = bytes[j+1];
        uint32_t accel_x_middle = bytes[j+2];

        uint32_t accel_y_upper = bytes[j+3];
        uint32_t accel_y_middle = bytes[j+4];

        uint32_t accel_z_upper = bytes[j+5];
        uint32_t accel_z_middle = bytes[j+6];

        
        uint32_t gyro_x_upper = bytes[j+7];
        uint32_t gyro_x_middle = bytes[j+8];

        uint32_t gyro_y_upper = bytes[j+9];
        uint32_t gyro_y_middle = bytes[j+10];

        uint32_t gyro_z_upper = bytes[j+11];
        uint32_t gyro_z_middle = bytes[j+12];

        
        uint16_t temp_upper = bytes[j+13];
        uint16_t temp_lower = bytes[j+14];

        
        uint16_t timestamp_upper = bytes[j+15];
        uint16_t timestamp_lower = bytes[j+16];

        uint32_t gyro_x_lower = bytes[j+17] & 0xF;
        uint32_t gyro_y_lower = bytes[j+18] & 0xF;
        uint32_t gyro_z_lower = bytes[j+19] & 0xF;
        
        uint32_t accel_x_lower = bytes[j+17] >> 4;
        uint32_t accel_y_lower = bytes[j+18] >> 4;
        uint32_t accel_z_lower = bytes[j+19] >> 4;


        //Reconstruct the data

        //There is a more elegant looking way to do the sign
        //extension by using bit shifts, but it is implementation defined

        if(accel_x_upper & (1 << 7)){
            accel_x_upper = 0xFFFFFF00 | accel_x_upper;
        }

        if(accel_y_upper & (1 << 7)){
            accel_y_upper = 0xFFFFFF00 | accel_y_upper;
        }

        if(accel_z_upper & (1 << 7)){
            accel_z_upper = 0xFFFFFF00 | accel_z_upper;
        }

        int32_t acc_x =  (accel_x_upper << 12) | (accel_x_middle << 4) | (accel_x_lower);
        int32_t acc_y =  (accel_y_upper << 12) | (accel_y_middle << 4) | (accel_y_lower);
        int32_t acc_z =  (accel_z_upper << 12) | (accel_z_middle << 4) | (accel_z_lower);

        //Always scaled to +-32g in fifo hires mode
        imu_dev->imu_data.fifo_array_20[i].accel_data[0] =   (double)(acc_x/16384.0);
        imu_dev->imu_data.fifo_array_20[i].accel_data[1] =   (double)(acc_y/16384.0);
        imu_dev->imu_data.fifo_array_20[i].accel_data[2] = - (double)(acc_z/16384.0);

        if(gyro_x_upper & (1 << 7)){
            gyro_x_upper = 0xFFFFFF00 | gyro_x_upper;
        }

        if(gyro_y_upper & (1 << 7)){
            gyro_y_upper = 0xFFFFFF00 | gyro_y_upper;
        }

        if(gyro_z_upper & (1 << 7)){
            gyro_z_upper = 0xFFFFFF00 | gyro_z_upper;
        }

        int32_t gyro_x =  (gyro_x_upper << 12) | (gyro_x_middle << 4) | (gyro_x_lower);
        int32_t gyro_y =  (gyro_y_upper << 12) | (gyro_y_middle << 4) | (gyro_y_lower);
        int32_t gyro_z =  (gyro_z_upper << 12) | (gyro_z_middle << 4) | (gyro_z_lower);

        //Always scaled to +-4000dps in fifo hires mode
        double d2r = 3.14159265 / 180.0;
        imu_dev->imu_data.fifo_array_20[i].gyro_data[0] =   gyro_x/131.1*d2r;
        imu_dev->imu_data.fifo_array_20[i].gyro_data[1] =   gyro_y/131.1*d2r;
        imu_dev->imu_data.fifo_array_20[i].gyro_data[2] = - gyro_z/131.1*d2r;

        //Timestamp
        imu_dev->imu_data.fifo_array_20[i].sample_timestamp = (timestamp_upper << 8) | timestamp_lower;


    }
}

//Getters

void icm45686_get_imu_data(imu *imu_dev)
{
    // 12 bytes of relevant data, the buffers need to be 13 bytes long
    uint8_t tx_buf[13];
    uint8_t rx_buf[13];

    // Get the data. Comes in two's complement
    icm45686_read_from_register(ICM45686_ACCEL_DATA_X1, tx_buf, rx_buf, sizeof(tx_buf), imu_dev->imu_pins.cs_pin);
    uint16_t msb_x = rx_buf[1];
    uint16_t lsb_x = rx_buf[2];
    uint16_t msb_y = rx_buf[3];
    uint16_t lsb_y = rx_buf[4];
    uint16_t msb_z = rx_buf[5];
    uint16_t lsb_z = rx_buf[6];

    int16_t acc_x = (msb_x << 8) | lsb_x;
    int16_t acc_y = (msb_y << 8) | lsb_y;
    int16_t acc_z = (msb_z << 8) | lsb_z;

    imu_dev->imu_data.accel_data[0] = acc_y / imu_dev->imu_cfg.accel_sensitivity;
    imu_dev->imu_data.accel_data[1] = acc_x / imu_dev->imu_cfg.accel_sensitivity;
    imu_dev->imu_data.accel_data[2] = -acc_z / imu_dev->imu_cfg.accel_sensitivity;

    msb_x = rx_buf[7];
    lsb_x = rx_buf[8];
    msb_y = rx_buf[9];
    lsb_y = rx_buf[10];
    msb_z = rx_buf[11];
    lsb_z = rx_buf[12];

    int16_t gyr_x = (msb_x << 8) | lsb_x;
    int16_t gyr_y = (msb_y << 8) | lsb_y;
    int16_t gyr_z = (msb_z << 8) | lsb_z;

    double d2r = 3.14159265 / 180.0;

    imu_dev->imu_data.gyro_data[0] = gyr_y / imu_dev->imu_cfg.gyro_sensitivity * d2r;
    imu_dev->imu_data.gyro_data[1] = gyr_x / imu_dev->imu_cfg.gyro_sensitivity * d2r;
    imu_dev->imu_data.gyro_data[2] = -gyr_z / imu_dev->imu_cfg.gyro_sensitivity * d2r;
}

uint16_t icm45686_get_fifo_packet_count(const imu *imu_dev)
{
    uint8_t tx_buf[3];
    uint8_t rx_buf[3];
    uint16_t count = 0;
    icm45686_read_from_register(ICM45686_FIFO_COUNT_0, tx_buf, rx_buf, sizeof(rx_buf), imu_dev->imu_pins.cs_pin);
    count = ((uint16_t)rx_buf[1] << 8) | rx_buf[2];
    return count;
}

error_code_t icm45686_get_fifo_buffer(imu* imu_dev)
{
    error_code_t return_code = no_error;

    uint16_t packet_count = icm45686_get_fifo_packet_count(imu_dev);
    uint16_t packet_size = imu_dev->imu_data.fifo_packet_size;

    if(packet_count > ICM45686_FIFO_PACKET_COUNT){
        return_code = fifo_size_error;
    }else{
        icm45686_read_from_register(ICM45686_FIFO_DATA, imu_dev->imu_data.dummy_tx_bytes, imu_dev->imu_data.raw_fifo_data, packet_size*packet_count + 1, imu_dev->imu_pins.cs_pin);
        icm45686_parse_fifo_frame(imu_dev, imu_dev->imu_data.raw_fifo_data, packet_size*packet_count);
    }

    return return_code;
}

//Setters

void icm45686_set_fifo(const imu *imu_dev)
{
    // A bit-setup sequence is provided in InvenSense's 45686 User Guide under section 3.5

    //Get fifo watermark
    uint16_t wm = imu_dev->imu_cfg.fifo_watermark_threshold;
    uint8_t lower_8 = (uint8_t)(wm & 0xFF);
    uint8_t upper_8 = (uint8_t)(wm >> 8);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG3, 0, ICM45686_FIFO_HIRES_EN_MASK |
                                                                  ICM45686_FIFO_GYRO_EN_MASK | 
                                                                  ICM45686_FIFO_ACCEL_EN_MASK |
                                                                  ICM45686_FIFO_IF_EN_MASK,
                                                                  imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG0, 0, ICM45686_FIFO_MODE_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_indirect_register(ICM45686_IPREG_TOP1, ICM45686_SMC_CONTROL_0, ICM45686_TMST_EN, ICM45686_TMST_EN_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG1_0, lower_8, ICM45686_FIFO_WM_TH_7_0_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG1_1, upper_8, ICM45686_FIFO_WM_TH_15_8_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG2, ICM45686_FIFO_WR_WM_GT_TH, ICM45686_FIFO_WR_WM_GT_TH_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG3, ICM45686_FIFO_GYRO_EN, ICM45686_FIFO_GYRO_EN_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG3, ICM45686_FIFO_ACCEL_EN, ICM45686_FIFO_ACCEL_EN_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG3, ICM45686_FIFO_HIRES_EN, ICM45686_FIFO_HIRES_EN_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG0, ICM45686_FIFO_DEPTH_2K, ICM45686_FIFO_DEPTH_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG0, ICM45686_FIFO_MODE , ICM45686_FIFO_MODE_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG4, ICM45686_FIFO_TMST_FSYNC_EN, ICM45686_FIFO_TMST_FSYNC_EN_MASK, imu_dev->imu_pins.cs_pin);

    icm45686_read_modify_write_register(ICM45686_FIFO_CONFIG3, ICM45686_FIFO_IF_EN, ICM45686_FIFO_IF_EN_MASK, imu_dev->imu_pins.cs_pin);
}

void icm45686_set_interrupt_pin_and_callback(const imu *imu_dev, gpio_irq_callback_t callback)
{
    if(imu_dev->imu_cfg.interrupt_type == enable_data_ready_interrupt || imu_dev->imu_cfg.interrupt_type == enable_fifo_ready_interrupt)
    {
        gpio_init(imu_dev->imu_pins.int_pin);
        gpio_set_dir(imu_dev->imu_pins.int_pin, GPIO_IN); // high impedance mode
        gpio_set_irq_enabled_with_callback(imu_dev->imu_pins.int_pin, GPIO_IRQ_EDGE_RISE, true, callback);
    }
}

void icm45686_set_cs_pin(const imu *imu_dev)
{
    gpio_init(imu_dev->imu_pins.cs_pin);
    gpio_set_dir(imu_dev->imu_pins.cs_pin, GPIO_OUT);
    gpio_put(imu_dev->imu_pins.cs_pin, 1); // CS is set to high, this is the idle state
}

void icm45686_set_measurement_ranges(const imu *imu_dev)
{
    uint8_t valid_gyro_fs = 1;
    uint8_t valid_accel_fs = 1;

    uint8_t gyro_fs_reg = 0;
    uint8_t accel_fs_reg = 0;

    switch (imu_dev->imu_cfg.gyro_fs)
    {
    case gyro_15dps:
        gyro_fs_reg = ICM45686_GYRO_FS_15;
        break;

    case gyro_31dps:
        gyro_fs_reg = ICM45686_GYRO_FS_31;
        break;

    case gyro_62dps:
        gyro_fs_reg = ICM45686_GYRO_FS_62;
        break;

    case gyro_125dps:
        gyro_fs_reg = ICM45686_GYRO_FS_125;
        break;

    case gyro_250dps:
        gyro_fs_reg = ICM45686_GYRO_FS_250;
        break;

    case gyro_500dps:
        gyro_fs_reg = ICM45686_GYRO_FS_500;
        break;

    case gyro_1000dps:
        gyro_fs_reg = ICM45686_GYRO_FS_1000;
        break;

    case gyro_2000dps:
        gyro_fs_reg = ICM45686_GYRO_FS_2000;
        break;

    case gyro_4000dps:
        gyro_fs_reg = ICM45686_GYRO_FS_4000;
        break;

    default:
        LOG("Invalid gyro FS\n");
        valid_gyro_fs = 0;
        break;
    }

    switch (imu_dev->imu_cfg.accel_fs)
    {
    case accel_2g:
        accel_fs_reg = ICM45686_ACCEL_FS_2G;
        break;

    case accel_4g:
        accel_fs_reg = ICM45686_ACCEL_FS_4G;
        break;

    case accel_8g:
        accel_fs_reg = ICM45686_ACCEL_FS_8G;
        break;

    case accel_16g:
        accel_fs_reg = ICM45686_ACCEL_FS_16G;
        break;

    case accel_32g:
        accel_fs_reg = ICM45686_ACCEL_FS_32G;
        break;

    default:
        LOG("Invalid accel FS\n");
        valid_accel_fs = 0;
        break;
    }

    if (valid_gyro_fs)
    {
        icm45686_read_modify_write_register(ICM45686_GYRO_CONFIG0, gyro_fs_reg, ICM45686_GYRO_FS_MASK, imu_dev->imu_pins.cs_pin);
    }

    if (valid_accel_fs)
    {
        icm45686_read_modify_write_register(ICM45686_ACCEL_CONFIG0, accel_fs_reg, ICM45686_ACCEL_FS_MASK, imu_dev->imu_pins.cs_pin);
    }
}

void icm45686_set_odr_frequency(const imu *imu_dev)
{
    uint8_t valid_gyro_odr = 1;
    uint8_t valid_accel_odr = 1;

    uint8_t gyro_odr = 0;
    uint8_t accel_odr = 0;

    switch (imu_dev->imu_cfg.gyro_odr)
    {
    case odr_0k8:
        gyro_odr = ICM45686_GYRO_ODR_0K8;
        break;
    
    case odr_1k6:
        gyro_odr = ICM45686_GYRO_ODR_1K6;
        break;

    case odr_3k2:
        gyro_odr = ICM45686_GYRO_ODR_3K2;
        break;

    case odr_6k4:
        gyro_odr = ICM45686_GYRO_ODR_6K4;
        break;

    default:
        valid_gyro_odr = 0;
        break;
    }


    switch (imu_dev->imu_cfg.accel_odr)
    {
    case odr_0k8:
        accel_odr = ICM45686_ACCEL_ODR_0K8;
        break;
    
    case odr_1k6:
        accel_odr = ICM45686_ACCEL_ODR_1K6;
        break;

    case odr_3k2:
        accel_odr = ICM45686_ACCEL_ODR_3K2;
        break;

    case odr_6k4:
        accel_odr = ICM45686_ACCEL_ODR_6K4;
        break;

    default:
        valid_accel_odr = 0;
        break;
    }

    if(valid_gyro_odr){
        icm45686_read_modify_write_register(ICM45686_GYRO_CONFIG0, gyro_odr, ICM45686_GYRO_ODR_MASK, imu_dev->imu_pins.cs_pin);
    }else{
        LOG("Invalid gyro ODR\n");
    }

    if(valid_accel_odr){
        icm45686_read_modify_write_register(ICM45686_ACCEL_CONFIG0, accel_odr, ICM45686_ACCEL_ODR_MASK, imu_dev->imu_pins.cs_pin);
    }else{
        LOG("Invalid accel ODR\n");
    }
}

void icm45686_set_power_modes(const imu *imu_dev)
{
    uint8_t valid_power_mode_gyro = 1;
    uint8_t valid_power_mode_accel = 1;

    uint8_t gyro_power_mode = 0;
    uint8_t accel_power_mode = 0;

    switch (imu_dev->imu_cfg.gyro_pwr_mode)
    {
    case power_off:
        gyro_power_mode = ICM45686_GYRO_OFF;
        break;

    case power_standby:
        gyro_power_mode = ICM45686_GYRO_STANDBY;
        break;

    case power_low_power:
        gyro_power_mode = ICM45686_GYRO_LOW_POWER;
        break;

    case power_low_noise:
        gyro_power_mode = ICM45686_GYRO_LOW_NOISE;
        break;
    
    default:
        valid_power_mode_gyro = 0;
        break;
    }


    switch (imu_dev->imu_cfg.accel_pwr_mode)
    {
    case power_off:
        accel_power_mode = ICM45686_ACCEL_OFF;
        break;

    case power_standby:
        accel_power_mode = ICM45686_ACCEL_OFFX;
        break;

    case power_low_power:
        accel_power_mode = ICM45686_ACCEL_LOW_POWER;
        break;

    case power_low_noise:
        accel_power_mode = ICM45686_ACCEL_LOW_NOISE;
        break;
    
    default:
        valid_power_mode_accel = 0;
        break;
    }

    if(valid_power_mode_gyro){
        icm45686_read_modify_write_register(ICM45686_PWR_MGMT0, gyro_power_mode, ICM45686_GYRO_MODE_MASK, imu_dev->imu_pins.cs_pin);
    }else{
        LOG("Invalid gyro power mode selected\n");
    }


    if(valid_power_mode_accel){
        icm45686_read_modify_write_register(ICM45686_PWR_MGMT0, accel_power_mode, ICM45686_ACCEL_MODE_MASK, imu_dev->imu_pins.cs_pin);
    }else{
        LOG("Invalid accel power mode selected\n");
    }
}

void icm45686_set_data_endianness(const imu *imu_dev)
{
    if (imu_dev->imu_cfg.data_endianness == use_big_endian)
    {
        icm45686_read_modify_write_indirect_register(ICM45686_IPREG_TOP1, ICM45686_SREG_CTRL, 1 << ICM45686_SREG_DATA_ENDIAN_SEL, 1 << ICM45686_SREG_DATA_ENDIAN_SEL, imu_dev->imu_pins.cs_pin);
    }
    else if (imu_dev->imu_cfg.data_endianness == use_little_endian)
    {
        icm45686_read_modify_write_indirect_register(ICM45686_IPREG_TOP1, ICM45686_SREG_CTRL, 0 << ICM45686_SREG_DATA_ENDIAN_SEL, 1 << ICM45686_SREG_DATA_ENDIAN_SEL, imu_dev->imu_pins.cs_pin); // Not tested if this reverts back to little endian after big endian has been set
    }
    else
    {
        LOG("Invalid argument\n");
    }
}

void icm45686_set_interrupt1(const imu* imu_dev)
{
    //This function configures the interrupt on the icm45686

    switch (imu_dev->imu_cfg.interrupt_type)
    {
    case enable_data_ready_interrupt:
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG0, 0x0, 0xFF, imu_dev->imu_pins.cs_pin); // Zero the register
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG1, 0x0, 0xFF, imu_dev->imu_pins.cs_pin); // Zero the register

        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG0, ICM45686_INT1_STATUS_EN_DRDY, ICM45686_INT1_STATUS_EN_DRDY_MASK, imu_dev->imu_pins.cs_pin); // Set data ready interrupt
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG2, ICM45686_INT1_DRIVE, ICM45686_INT1_DRIVE_MASK, imu_dev->imu_pins.cs_pin);                   // INT1 pin behavior setup
        break;

    case enable_fifo_ready_interrupt:
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG0, 0x0, 0xFF, imu_dev->imu_pins.cs_pin); // Zero the register
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG1, 0x0, 0xFF, imu_dev->imu_pins.cs_pin); // Zero the register
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG2, 0x0, 7, imu_dev->imu_pins.cs_pin); // Zero the register

        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG0, ICM45686_INT1_STATUS_EN_FIFO_THS, ICM45686_INT1_STATUS_EN_FIFO_THS_MASK, imu_dev->imu_pins.cs_pin);
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG2, ICM45686_INT1_DRIVE | 
                                                                   ICM45686_INT1_MODE | 
                                                                   ICM45686_INT1_POLARITY, 
                                                                   ICM45686_INT1_DRIVE_MASK | 
                                                                   ICM45686_INT1_MODE_MASK | 
                                                                   ICM45686_INT1_POLARITY_MASK, 
                                                                   imu_dev->imu_pins.cs_pin); // INT1 pin behavior setup
        break;

    case disable_interrupt:
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG0, 0x0, 0xFF, imu_dev->imu_pins.cs_pin); // Zero the register
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG1, 0x0, 0xFF, imu_dev->imu_pins.cs_pin); // Zero the register
        icm45686_read_modify_write_register(ICM45686_INT1_CONFIG2, 0x0, 7, imu_dev->imu_pins.cs_pin); // Zero the register
        break;
    
    default:
        LOG("Invalid interrupt option requested\n");
        break;
    }
};

void icm45686_set_rp2350_clock_out()
{
    gpio_set_function(PF_ICM45686_CLOCK, GPIO_FUNC_GPCK);
    gpio_set_drive_strength(PF_ICM45686_CLOCK, GPIO_DRIVE_STRENGTH_2MA);
    clock_gpio_init_int_frac16(PF_ICM45686_CLOCK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 375, 0); //375,0
}

void icm45686_set_rp2350_pwm_signal()
{
    gpio_set_function(PF_ICM45686_CLOCK, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PF_ICM45686_CLOCK);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, CLOCK_DIVIDER);
    pwm_config_set_wrap(&config, PWM_WRAP_VALUE);
    pwm_init(slice, &config, true);
    pwm_set_enabled(slice, true);
    pwm_set_chan_level(slice, PF_ICM45686_CLOCK % 2, PWM_WRAP_VALUE / 2); // Set to 50% duty
}

void icm45686_set_clock_source(const imu *imu_dev)
{
    switch (imu_dev->imu_cfg.clock_source)
    {
    case use_internal_clock:
        /* code */
        break;

    case use_external_clock:
        // Will use INT2 pin for CLKIN, how to configure it for this? Section 7.3 of ICM45686 user guide:
        // To use pin 9 as CLKIN, the PADS_INT2_CFG_OVRD_VAL must be set to 2 in
        //----->IOC_PAD_SCENARIO_OVRD, user bank 0
        // Must first set the OVRD bit, else it appears the VAL can not be written to
        icm45686_read_modify_write_register(ICM45686_IOC_PAD_SCENARIO_OVRD,
                                            ICM45686_PADS_INT2_CFG_OVRD,
                                            ICM45686_PADS_INT2_CFG_OVRD_MASK,
                                            imu_dev->imu_pins.cs_pin);

        icm45686_read_modify_write_register(ICM45686_IOC_PAD_SCENARIO_OVRD,
                                            ICM45686_PADS_INT2_CFG_OVRD_VAL,
                                            ICM45686_PADS_INT2_CFG_OVRD_VAL_MASK,
                                            imu_dev->imu_pins.cs_pin);

        // Set sync timing control for accel
        icm45686_read_modify_write_indirect_register(ICM45686_IPREG_TOP1,
                                                    ICM45686_SMC_CONTROL_0,
                                                    ICM45686_ACCEL_LP_CLK_SEL,
                                                    ICM45686_ACCEL_LP_CLK_SEL_MASK,
                                                    imu_dev->imu_pins.cs_pin);

        // Next, to enable the CLKIN function, the RTC_MODE bit must be set to 1 in
        //----->RTC_CONFIG, user bank 0
        icm45686_read_modify_write_register(ICM45686_RTC_CONFIG,
                                            ICM45686_RTC_MODE,
                                            ICM45686_RTC_MODE_MASK,
                                            imu_dev->imu_pins.cs_pin);

        // I3C STC and CLKIN use the same interpolator but I3C has higher priority. To use CLKIN, I3C_STC_MODE must be set to 0 on
        //----->SIFS_I3C_STC_CFG, user bank IPREG_TOP1
        icm45686_read_modify_write_indirect_register(ICM45686_IPREG_TOP1,
                                                    ICM45686_SIFS_I3C_STC_CFG,
                                                    ICM45686_I3C_STC_MODE,
                                                    ICM45686_I3C_STC_MODE_MASK,
                                                    imu_dev->imu_pins.cs_pin);

        // ACCEL_SRC_CTRL[1:0] must be set to 0b10 (FIR and interpolator on) in
        //----->IPREG_SYS2_REG_123, user bank IPREG_SYS2
        icm45686_read_modify_write_indirect_register(ICM45686_IPREG_SYS2,
                                                    ICM45686_IPREG_SYS2_REG_123,
                                                    ICM45686_ACCEL_SRC_CTRL,
                                                    ICM45686_ACCEL_SRC_CTRL_MASK,
                                                    imu_dev->imu_pins.cs_pin);

        // GYRO_SRC_CTRL[1:0] must be set to 0b10 (FIR and interpolator on) in
        //----->IPREG_SYS1_REG_166, user bank IPREG_SYS1
        icm45686_read_modify_write_indirect_register(ICM45686_IPREG_SYS1,
                                                    ICM45686_IPREG_SYS1_REG_166,
                                                    ICM45686_GYRO_SRC_CTRL,
                                                    ICM45686_GYRO_SRC_CTRL_MASK,
                                                    imu_dev->imu_pins.cs_pin);
        break;
    
    default:
        break;
    }
}

void icm45686_set_config(const imu* imu_dev)
{
    //imu_data_ready and imu_fifo require callback functions, these must be attached later

    //Power down sensors before setting configs
    imu imu_power_off = *imu_dev;
    icm45686_configure_power_modes(&imu_power_off, power_off, power_off);
    icm45686_set_power_modes(&imu_power_off);

    switch (imu_dev->imu_cfg.mode)
    {
    case imu_polling:
        icm45686_set_measurement_ranges(imu_dev);
        icm45686_set_odr_frequency(imu_dev);
        icm45686_set_data_endianness(imu_dev);
        icm45686_set_clock_source(imu_dev);
        icm45686_set_power_modes(imu_dev);
        break;

    case imu_data_ready:
        icm45686_set_measurement_ranges(imu_dev);
        icm45686_set_odr_frequency(imu_dev);
        icm45686_set_data_endianness(imu_dev);
        icm45686_set_clock_source(imu_dev);
        icm45686_set_interrupt1(imu_dev);
        icm45686_set_interrupt_pin_and_callback(imu_dev, icm45686_int1_callback);
        icm45686_set_power_modes(imu_dev);
        break;

    case imu_fifo:
        icm45686_set_measurement_ranges(imu_dev);
        icm45686_set_odr_frequency(imu_dev);
        icm45686_set_data_endianness(imu_dev);
        icm45686_set_clock_source(imu_dev);
        icm45686_set_interrupt1(imu_dev);
        icm45686_set_interrupt_pin_and_callback(imu_dev, icm45686_int1_callback);
        icm45686_set_fifo(imu_dev);
        icm45686_set_power_modes(imu_dev);
        break;
    
    default:
        break;
    }
    sleep_ms(100); //Takes the gyro 35 ms to start
}

//Configuration

error_code_t icm45686_configure_default_config(imu* imu_dev, imu_mode_t default_mode)
{
    error_code_t return_code = no_error;

    switch (default_mode)
    {
    case imu_polling:
        icm45686_configure_main_imu_mode(imu_dev, imu_polling);
        icm45686_configure_clock_cource(imu_dev, use_internal_clock);
        icm45686_configure_data_endianness(imu_dev, use_big_endian);
        icm45686_configure_odr(imu_dev, odr_1k6, odr_1k6);
        icm45686_configure_measurement_ranges(imu_dev, gyro_2000dps, accel_2g);
        icm45686_configure_power_modes(imu_dev, power_low_noise, power_low_noise);
        break;

    case imu_data_ready:
        icm45686_configure_main_imu_mode(imu_dev, imu_data_ready);
        icm45686_configure_clock_cource(imu_dev, use_external_clock);
        icm45686_configure_data_endianness(imu_dev, use_big_endian);
        icm45686_configure_odr(imu_dev, odr_6k4, odr_6k4);
        icm45686_configure_measurement_ranges(imu_dev, gyro_2000dps, accel_2g);
        icm45686_configure_interrupt(imu_dev, enable_data_ready_interrupt);
        icm45686_configure_power_modes(imu_dev, power_low_noise, power_low_noise);
        break;

    case imu_fifo:
        icm45686_configure_main_imu_mode(imu_dev, imu_fifo);
        icm45686_configure_clock_cource(imu_dev, use_external_clock);
        icm45686_configure_data_endianness(imu_dev, use_big_endian);
        icm45686_configure_odr(imu_dev, odr_0k8, odr_0k8);
        icm45686_configure_measurement_ranges(imu_dev, gyro_4000dps, accel_32g);
        icm45686_configure_fifo_frame_contents_and_watermark(imu_dev, fifo_accel_gyro_hires_20, ICM45686_FIFO_PACKET_COUNT);
        icm45686_configure_interrupt(imu_dev, enable_fifo_ready_interrupt);
        icm45686_configure_power_modes(imu_dev, power_low_noise, power_low_noise);
        break;
    
    default:
        return_code = default_config_invalid;
        break;
    }
    return return_code;
}

error_code_t icm45686_configure_measurement_ranges(imu* imu_dev, const gyro_measurement_range_t gyro_range, const accel_measurement_range_t accel_range)
{
    imu_dev->imu_cfg.gyro_fs = gyro_range;
    imu_dev->imu_cfg.accel_fs = accel_range;
    error_code_t return_code = no_error;
    
    switch (gyro_range)
    {
    case gyro_15dps:
        imu_dev->imu_cfg.gyro_sensitivity = 2097.2;
        break;

    case gyro_31dps:
        imu_dev->imu_cfg.gyro_sensitivity = 1048.6;
        break;

    case gyro_62dps:
        imu_dev->imu_cfg.gyro_sensitivity = 524.3;
        break;

    case gyro_125dps:
        imu_dev->imu_cfg.gyro_sensitivity = 262.0;
        break;

    case gyro_250dps:
        imu_dev->imu_cfg.gyro_sensitivity = 131.0;
        break;

    case gyro_500dps:
        imu_dev->imu_cfg.gyro_sensitivity = 65.5;
        break;

    case gyro_1000dps:
        imu_dev->imu_cfg.gyro_sensitivity = 32.8;
        break;

    case gyro_2000dps:
        imu_dev->imu_cfg.gyro_sensitivity = 16.4;
        break;

    case gyro_4000dps:
        imu_dev->imu_cfg.gyro_sensitivity = 8.2;
        break;

    default:
        LOG("Invalid gyro FS\n");
        return_code = fs_invalid;
        break;
    }

    switch (accel_range)
    {
    case accel_2g:
        imu_dev->imu_cfg.accel_sensitivity = 16384.0;
        break;

    case accel_4g:
        imu_dev->imu_cfg.accel_sensitivity = 8192.0;
        break;

    case accel_8g:
        imu_dev->imu_cfg.accel_sensitivity = 4096.0;
        break;

    case accel_16g:
        imu_dev->imu_cfg.accel_sensitivity = 2048.0;
        break;

    case accel_32g:
        imu_dev->imu_cfg.accel_sensitivity = 1024.0;
        break;

    default:
        LOG("Invalid accel FS\n");
        return_code = fs_invalid;
        break;
    }
    return return_code;
}

error_code_t icm45686_configure_odr(imu* imu_dev, const imu_odr_t gyro_odr, const imu_odr_t accel_odr)
{
    error_code_t return_code = no_error;

    if(gyro_odr == odr_0k8 || gyro_odr == odr_1k6 || gyro_odr == odr_3k2 || gyro_odr == odr_6k4){
        imu_dev->imu_cfg.gyro_odr = gyro_odr;
    }else{
        return_code = odr_invalid;
    }

    if(accel_odr == odr_0k8 || accel_odr == odr_1k6 || accel_odr == odr_3k2 || accel_odr == odr_6k4){
        imu_dev->imu_cfg.accel_odr = accel_odr;
    }else{
        return_code = odr_invalid;
    }

    return return_code;
}

error_code_t icm45686_configure_power_modes(imu* imu_dev, const sensor_power_mode_t gyro_pwr_mode, const sensor_power_mode_t accel_pwr_mode)
{
    error_code_t return_code = no_error;

    if(gyro_pwr_mode == power_off || gyro_pwr_mode == power_standby || gyro_pwr_mode == power_low_power || gyro_pwr_mode == power_low_noise){
        imu_dev->imu_cfg.gyro_pwr_mode = gyro_pwr_mode;
    }else{
        return_code = power_mode_invalid;
    }

    if(accel_pwr_mode == power_off || accel_pwr_mode == power_standby || accel_pwr_mode == power_low_power || accel_pwr_mode == power_low_noise){
        imu_dev->imu_cfg.accel_pwr_mode = accel_pwr_mode;
    }else{
        return_code = power_mode_invalid;
    }

    return return_code;
}

error_code_t icm45686_configure_data_endianness(imu* imu_dev, const data_endian_t endian)
{
    error_code_t return_code = no_error;

    if(endian == use_big_endian || endian == use_little_endian){
        imu_dev->imu_cfg.data_endianness = endian;
    }else{
        return_code = endian_invalid;
    }

    return return_code;
}

error_code_t icm45686_configure_clock_cource(imu* imu_dev, const clock_source_t clock_source)
{
    error_code_t return_code = no_error;

    if(clock_source == use_internal_clock || clock_source == use_external_clock){
        imu_dev->imu_cfg.clock_source = clock_source;
    }else{
        return_code = clock_source_invalid;
    }
    
    return return_code;
}

error_code_t icm45686_configure_interrupt(imu* imu_dev, const interrupt_t interrupt)
{
    error_code_t return_code = no_error;

    switch (interrupt)
    {
    case enable_data_ready_interrupt:
        imu_dev->imu_cfg.interrupt_type = enable_data_ready_interrupt;
        break;

    case enable_fifo_ready_interrupt:
        imu_dev->imu_cfg.interrupt_type = enable_fifo_ready_interrupt;
        break;

    case disable_interrupt:
        imu_dev->imu_cfg.interrupt_type = disable_interrupt;
        break;
    
    default:
        return_code = interrupt_invalid;
        break;
    }

    return return_code;
}

error_code_t icm45686_configure_fifo_frame_contents_and_watermark(imu* imu_dev, fifo_frame_contents_t contents, uint16_t watermark)
{
    error_code_t return_code = no_error;

    if(contents == fifo_accel_only_8 || contents == fifo_gyro_only_8 || contents == fifo_accel_gyro_16 || contents == fifo_accel_gyro_hires_20){
        imu_dev->imu_cfg.fifo_frame_contents = contents;
    }else{
        return_code = fifo_frame_invalid;
    }

    //Update later to support other packet sizes if necessary
    if(contents == fifo_accel_gyro_hires_20){
        imu_dev->imu_data.fifo_packet_size = 20;
    }

    //Currently using a 2kB fifo buffer, the watermark must be set such that it doesn't overflow
    uint16_t bytes_in_fifo = 0;
    uint16_t fifo_size = 2000;
    imu_dev->imu_cfg.fifo_watermark_threshold = watermark;

    if(contents == fifo_accel_only_8 || contents == fifo_gyro_only_8){
        bytes_in_fifo = watermark*8;
    }else if(contents == fifo_accel_gyro_16){
        bytes_in_fifo = watermark*16;
    }else if(contents == fifo_accel_gyro_hires_20){
        bytes_in_fifo = watermark*20;
    }

    if(bytes_in_fifo > fifo_size){
        return_code = fifo_buffer_overflow;
    }

    return return_code;
}

error_code_t icm45686_configure_main_imu_mode(imu* imu_dev, const imu_mode_t mode)
{
    error_code_t return_code = no_error;

    if(mode == imu_polling || mode == imu_data_ready || mode == imu_fifo){
        imu_dev->imu_cfg.mode = mode;
    }else{
        return_code = main_imu_mode_invalid;
    }
    
    return return_code;
}

error_code_t icm45686_configure_pins(imu* imu_dev, const picoflight_pins_t cs_pin, const picoflight_pins_t int_pin)
{
    error_code_t return_code = no_error;

    if(cs_pin == PF_ICM45686_0_CS || cs_pin == PF_ICM45686_1_CS || cs_pin == PF_ICM45686_2_CS){
        imu_dev->imu_pins.cs_pin = cs_pin;
    }else{
        return_code = pin_invalid;
    }

    if(int_pin == PF_ICM45686_0_INT || int_pin == PF_ICM45686_1_INT || int_pin == PF_ICM45686_2_INT){
        imu_dev->imu_pins.int_pin = int_pin;
    }else{
        return_code = pin_invalid;
    }

    return return_code;
}

void icm45686_print_error(error_code_t error)
{
    switch (error)
    {
    case no_error:
        PRINT("ICM NO ERROR\n");
        break;

    case fs_invalid:
        PRINT("ICM FS INVALID\n");
        break;

    case odr_invalid:
        PRINT("ICM ODR INVALID\n");
        break;

    case power_mode_invalid:
        PRINT("ICM POWER MODE INVALID\n");
        break;

    case endian_invalid:
        PRINT("ICM ENDIANN INVALID\n");
        break;

    case clock_source_invalid:
        PRINT("ICM CLOCK SOURCE INVALID\n");
        break;

    case main_imu_mode_invalid:
        PRINT("ICM MAIN IMU MODE INVALID\n");
        break;

    case fifo_frame_invalid:
        PRINT("ICM FIFO FRAME INVALID\n");
        break;

    case fifo_buffer_overflow:
        PRINT("ICM FIFO BUFFER OVERFLOW\n");
        break;

    case fifo_size_error:
        PRINT("ICM FIFO SIZE ERROR\n");
        break;

    case pin_invalid:
        PRINT("ICM PIN INVALID\n");
        break;

    case interrupt_invalid:
        PRINT("ICM INTERRUPT INVALID\n");
        break;

    case default_config_invalid:
        PRINT("ICM DEFAULT CONFIG INVALID\n");
        break;
    
    default:
        PRINT("ICM UNKNOWN ERROR\n");
        break;
    }
}

//SPI comms

void icm45686_read_indirect_register(uint16_t bank, uint8_t ireg, uint8_t *ireg_value, uint8_t cs_pin)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    if (bank == ICM45686_IMEM_SRAM ||
        bank == ICM45686_IPREG_BAR ||
        bank == ICM45686_IPREG_SYS1 ||
        bank == ICM45686_IPREG_SYS2 ||
        bank == ICM45686_IPREG_TOP1)
    {
        uint16_t ireg_16bit_add = bank + ireg;
        uint8_t ireg_7_0 = (uint8_t)(ireg_16bit_add & 0x00FF);
        uint8_t ireg_15_8 = (uint8_t)(ireg_16bit_add >> 8);
        tx_buf[1] = ireg_7_0;
        icm45686_write_to_register(ICM45686_IREG_ADDR_7_0, tx_buf, rx_buf, 2, cs_pin);
        tx_buf[1] = ireg_15_8;
        icm45686_write_to_register(ICM45686_IREG_ADDR_15_8, tx_buf, rx_buf, 2, cs_pin);
        // Read back the data stored in IREG_DATA
        sleep_ms(1);
        icm45686_read_from_register(ICM45686_IREG_DATA, tx_buf, rx_buf, 2, cs_pin);
        *ireg_value = rx_buf[1];
    }
    else
    {
        LOG("Invalid internal register bank selected\n");
    }
}

void icm45686_write_indirect_register(uint16_t bank, uint8_t ireg, uint8_t ireg_value, uint8_t cs_pin)
{
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];
    if (bank == ICM45686_IMEM_SRAM ||
        bank == ICM45686_IPREG_BAR ||
        bank == ICM45686_IPREG_SYS1 ||
        bank == ICM45686_IPREG_SYS2 ||
        bank == ICM45686_IPREG_TOP1)
    {
        uint16_t ireg_16bit_add = bank + ireg;
        uint8_t ireg_7_0 = (uint8_t)(ireg_16bit_add & 0x00FF);
        uint8_t ireg_15_8 = (uint8_t)(ireg_16bit_add >> 8);
        tx_buf[1] = ireg_15_8;
        tx_buf[2] = ireg_7_0;
        tx_buf[3] = ireg_value;
        icm45686_write_to_register(ICM45686_IREG_ADDR_15_8, tx_buf, rx_buf, 4, cs_pin);
    }
    else
    {
        LOG("Invalid internal register bank selected\n");
    }
    sleep_ms(1);
}

void icm45686_read_modify_write_indirect_register(uint16_t bank, uint8_t ireg, uint8_t ireg_value, uint8_t mask, uint8_t cs_pin)
{
    uint8_t value_from_reg;
    uint8_t value_to_reg;
    icm45686_read_indirect_register(bank, ireg, &value_from_reg, cs_pin);
    value_to_reg = value_from_reg & ~mask;
    value_to_reg |= (ireg_value & mask);
    icm45686_write_indirect_register(bank, ireg, value_to_reg, cs_pin);
}

void icm45686_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin)
{
    // The mask signifies which bits to update. mask = 0b00000001 means leave all bits as-is except maybe bit 0
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    icm45686_read_from_register(dev_register, tx_buf, rx_buf, sizeof(tx_buf), cs_pin);
    uint8_t value_from_register = rx_buf[1];
    value_from_register &= ~(mask); // This sets all bits we want to update to 0
    uint8_t value_to_register = value_from_register | (bits_to_update & mask);
    tx_buf[1] = value_to_register;
    icm45686_write_to_register(dev_register, tx_buf, rx_buf, sizeof(tx_buf), cs_pin);
}

void icm45686_read_from_register(uint8_t dev_register, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register | SET_SPI_READ; // Sets the read bit, bit 7, to 1
    gpio_put(cs_pin, 0);                     // Chip select is active low
    uint length = spi_write_read_blocking(spi0, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin, 1);
}

void icm45686_write_to_register(uint8_t dev_register, uint8_t *tx_buf, uint8_t *rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register; // Leaves the read bit at 0
    gpio_put(cs_pin, 0);      // Chip select is active low
    spi_write_read_blocking(spi0, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin, 1);
}
