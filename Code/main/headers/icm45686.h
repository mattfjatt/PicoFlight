#ifndef ICM45686_H
#define ICM45686_H

#include "headers/logging.h"
#include "headers/pins.h"
#include "headers/bus.h"
#include "headers/linalg.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "string.h"

typedef enum error_code_t{
    no_error,
    gyro_fs_invalid,
    accel_fs_invalid,
    gyro_odr_invalid,
    accel_odr_invalid,
    gyro_power_mode_invalid,
    accel_power_mode_invalid,
    endian_invalid,
    clock_source_invalid,
    main_imu_mode_invalid,
    fifo_frame_invalid,
    fifo_buffer_overflow,
    pin_invalid
}error_code_t;

typedef enum imu_mode_t{
    imu_polling = 0, //The most basic mode where you poll the data registers whenever
    imu_data_ready = 1, //An interrupt is fired when new data is available
    imu_fifo = 2 //An interrupt is fired when the FIFO buffer is ready
}imu_mode_t;

typedef enum clock_source_t{
    use_internal_clock = 0,
    use_external_clock = 1
}clock_source_t;

typedef enum data_endian_t{
    use_big_endian,
    use_little_endian
}data_endian_t;

typedef enum accel_measurement_range_t{
    accel_2g,
    accel_4g,
    accel_8g,
    accel_16g,
    accel_32g
}accel_measurement_range_t;

typedef enum gyro_measurement_range_t{
    gyro_15dps,
    gyro_31dps,
    gyro_62dps,
    gyro_125dps,
    gyro_250dps,
    gyro_500dps,
    gyro_1000dps,
    gyro_2000dps,
    gyro_4000dps
}gyro_measurement_range_t;

typedef enum imu_odr_t{
    odr_0k8,
    odr_1k6,
    odr_3k2,
    odr_6k4
}imu_odr_t;

typedef enum sensor_power_mode_t{
    power_off,
    power_standby, //No standby mode for the accelerometer
    power_low_noise,
    power_low_power
}sensor_power_mode_t;

typedef enum fifo_frame_contents_t{
    fifo_accel_only_8,
    fifo_gyro_only_8,
    fifo_accel_gyro_16,
    fifo_accel_gyro_hires_20
}fifo_frame_contents_t;


typedef struct imu_fifo_20bit{
    uint8_t accel_x_upper, accel_x_middle;
    uint8_t accel_y_upper, accel_y_middle;
    uint8_t accel_z_upper, accel_z_middle;

    uint8_t gyro_x_upper, gyro_x_middle;
    uint8_t gyro_y_upper, gyro_y_middle;
    uint8_t gyro_z_upper, gyro_z_middle;

    uint8_t temp_upper, temp_lower;

    uint8_t timestamp_upper, timestamp_lower;

    //Contains lower 4 bits for 19 bit accel. LSB is always 0
    uint8_t accel_x_lower;
    uint8_t accel_y_lower;
    uint8_t accel_z_lower;

    //Contains lower 4 bits for 20 bit gyro.
    uint8_t gyro_x_lower;
    uint8_t gyro_y_lower;
    uint8_t gyro_z_lower;

}imu_fifo_20bit;

typedef struct imu_pins{
    uint8_t cs_pin;
    uint8_t int_pin;
}imu_pins;

typedef struct fifo_config{
    fifo_frame_contents_t frame_contents;
    
    // //"Enable/Disables" are used as booleans, others are used as full bytes.

    // //No fifo setup will be performed unless this is true.
    // uint8_t fifo_enabled;

    // //FIFO_CONFIG0
    // uint8_t fifo_mode;
    // uint8_t fifo_depth;

    // //FIFO_CONFIG1_0
    // uint8_t fifo_wm_th_7_0;

    // //FIFO_CONFIG1_1
    // uint8_t fifo_wm_th_15_8;

    // //FIFO_CONFIG2
    // uint8_t fifo_flush;
    // uint8_t fifo_wr_wm_gt_th;

    // //FIFO_CONFIG3
    // uint8_t fifo_es1_en;
    // uint8_t fifo_es0_en;
    // uint8_t fifo_hires_en;
    // uint8_t fifo_gyro_en;
    // uint8_t fifo_accel_en;
    // uint8_t fifo_if_en;

    // //FIFO_CONFIG4
    // uint8_t fifo_comp_nc_flow_cfg;
    // uint8_t fifo_comp_en;
    // uint8_t fifo_tmst_fsync_en;
    // uint8_t fifo_es0_6b_9b;

}fifo_config;

typedef struct imu_config{
    gyro_measurement_range_t gyro_fs;
    accel_measurement_range_t accel_fs;
    double gyro_sensitivity;// = 8.2;
    double accel_sensitivity;// = 1024.0;
    imu_odr_t gyro_odr;
    imu_odr_t accel_odr;
    sensor_power_mode_t gyro_pwr_mode;
    sensor_power_mode_t accel_pwr_mode;
    data_endian_t data_endianness; //Big or Little
    clock_source_t clock_source; //Internal or external
    uint8_t data_ready_int;
    // fifo_config fifo_cfg;
    imu_mode_t mode;
    fifo_frame_contents_t fifo_frame_contents;
    uint32_t fifo_watermark_threshold;
}imu_config;

typedef struct imu_data{
    //New data available flag if data-ready interrupt is enabled. Data getter function must set this flag back to zero
    double gyro_data[3];
    double accel_data[3];
    imu_fifo_20bit fifo_array[0x06]; //<-- Length of array tied to ICM45686_FIFO_WM_TH_7_0/15_8?
}imu_data;

typedef struct imu{
    imu_pins imu_pins;
    imu_config imu_cfg;
    imu_data imu_data;
}imu;

extern imu imu0;
extern imu imu1;
extern imu imu2;

void icm45686_init();

void icm45686_set_config_2(const imu* imu_dev);

void icm45686_configure_int_for_fifo(const imu* imu_dev);

void icm45686_setup_fifo(const imu* imu_dev); // Update this?

uint16_t icm45686_get_fifo_packet_count(const imu* imu_dev); //Returns amount of data frames ready to be read

void icm45686_parse_20bit_fifo_frame(imu_fifo_20bit* parsed_frame, uint8_t raw_frame[19]);

void icm45686_get_imu_data(imu* imu_dev);

//The "configure" functions update the struct while the "set" functions write the value to the IMU

imu_config get_default_config(imu_mode_t default_mode); //Returns a default config struct that can be used

uint16_t icm45686_configure_measurement_ranges(imu* imu_dev, const gyro_measurement_range_t gyro_range, const accel_measurement_range_t accel_range);

uint16_t icm45686_configure_odr(imu* imu_dev, const imu_odr_t gyro_odr, const imu_odr_t accel_odr);

uint16_t icm45686_configure_power_modes(imu* imu_dev, const sensor_power_mode_t gyro_pwr_mode, const sensor_power_mode_t accel_pwr_mode);

uint16_t icm45686_configure_data_endianness(imu* imu_dev, const data_endian_t endian);

uint16_t icm45686_configure_fifo_frame_contents_and_watermark(imu* imu_dev, fifo_frame_contents_t contents, uint16_t watermark);

uint16_t icm45686_configure_clock_cource(imu* imu_dev, const clock_source_t clock_source);

uint16_t icm45686_configure_main_imu_mode(imu* imu_dev, const imu_mode_t mode);

uint16_t icm45686_set_pins(imu* imu_dev, const uint16_t cs_pin, const uint16_t int_pin);

void icm45686_set_config(const imu* imu); //Applies all config values

void icm45686_set_measurement_ranges(imu* imu_dev); // Update this

void icm45686_set_odr_frequency(const imu* imu_dev); // Update this

void icm45686_set_power_modes(const imu* imu_dev); // Update this

void icm45686_set_data_endianness(const imu* imu_dev); // Update this

void icm45686_set_external_clock_source(const imu* imu_dev); // Update this

void icm45686_set_rp2350_pwm_signal(); //Sets PWM frequency at 50% duty cycle

void icm45686_set_rp2350_clock_out();

void icm45686_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint32_t n_bytes, uint8_t cs_pin);

void icm45686_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin);

void icm45686_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin);

void icm45686_read_indirect_register(uint16_t bank, uint8_t ireg, uint8_t* ireg_value, uint8_t cs_pin); //See ICM45686 datasheet section 14 "Indirect register access" for more info

void icm45686_write_indirect_register(uint16_t bank, uint8_t ireg, uint8_t ireg_value, uint8_t cs_pin);

void icm45686_read_modify_write_indirect_register(uint16_t bank, uint8_t ireg, uint8_t ireg_value, uint8_t mask, uint8_t cs_pin);

void icm45686_configure_int1_pin(const imu* imu_dev);

void icm45686_set_interrupt_pin_and_callback(const imu* imu_dev, gpio_irq_callback_t callback);

void icm45686_set_cs_pin(const imu* imu_dev);

void icm45686_int1_callback(uint gpio, uint32_t events);

void icm45686_TEST_FIFO();

//Some configs 
#define ICM45686_USE_BIG_ENDIAN     0x20
#define ICM45686_USE_LITTLE_ENDIAN  0x21

#define ICM45686_USE_DATA_READY_INT 0x22
#define ICM45686_USE_EXT_CLKIN      0x23


//Registers
#define ICM45686_ACCEL_DATA_X1  0x00

//Config registers that may be relevant
#define ICM45686_PWR_MGMT0      0x10

//The slew rate for int1 can be set in DRIVE_CONFIG2
#define ICM45686_INT1_CONFIG0   0x16 //Sets int1 source
#define ICM45686_INT1_CONFIG1   0x17 //Not relevant
#define ICM45686_INT1_CONFIG2   0x18 //Configures int1 pin like polarity and drive

#define ICM45686_INT1_STATUS0   0x19 //Might have to read these regs to let the IMU
#define ICM45686_INT1_STATUS1   0x1A //know the interrupt has been acknowledged


#define ICM45686_ACCEL_CONFIG0  0x1B //FS and ODR selection
#define ICM45686_GYRO_CONFIG0   0x1C //FS and ODR selection

#define ICM45686_FIFO_COUNT_0   0x12 //High bits
#define ICM45686_FIFO_COUNT_1   0x13 //Low bits
#define ICM45686_FIFO_DATA      0x14 //FIFO data port

#define ICM45686_FIFO_CONFIG0   0x1D
#define ICM45686_FIFO_CONFIG1_0 0x1E
#define ICM45686_FIFO_CONFIG1_1 0x1F
#define ICM45686_FIFO_CONFIG2   0x20
#define ICM45686_FIFO_CONFIG3   0x21
#define ICM45686_FIFO_CONFIG4   0x22

#define ICM45686_ODR_DECIMATE_CONFIG 0x28

#define ICM45686_INT2_CONFIG0   0x56
#define ICM45686_INT2_CONFIG1   0x57
#define ICM45686_INT2_CONFIG2   0x58

#define ICM45686_INT2_STATUS0   0x59
#define ICM45686_INT2_STATUS1   0x5A

#define ICM45686_WHO_AM_I       0x72

#define ICM45686_IREG_ADDR_15_8 0x7C
#define ICM45686_IREG_ADDR_7_0  0x7D
#define ICM45686_IREG_DATA      0x7E

#define ICM45686_SREG_CTRL      0x67

//Regs for clock source setup
#define ICM45686_IOC_PAD_SCENARIO_OVRD  0x31
#define ICM45686_RTC_CONFIG             0x26
#define ICM45686_SIFS_I3C_STC_CFG       0x68
#define ICM45686_IPREG_SYS2_REG_123     0x7B
#define ICM45686_IPREG_SYS1_REG_166     0xA6
#define ICM45686_REG_MISC1              0x35 //MCLK source 
#define ICM45686_SMC_CONTROL_0          0x58 //IPREG_TOP1

//Pullup register for pin 9 (int 2)
#define ICM45686_IPREG_BAR_REG_62   0x3E

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

//CLOCK SOURCE CONFIG
#define ICM45686_PADS_INT2_CFG_OVRD_MASK     0b100
#define ICM45686_PADS_INT2_CFG_OVRD_VAL_MASK 0b011
#define ICM45686_PADS_INT2_CFG_OVRD          0b100
#define ICM45686_PADS_INT2_CFG_OVRD_VAL      0b010 //0: INT2. 1: FSYNC. 2: CLKIN

#define ICM45686_RTC_MODE_MASK         (0b1 << 5)
#define ICM45686_RTC_ALIGN_MASK        (0b1 << 6)
#define ICM45686_RTC_ALIGN             (0b1 << 6)
#define ICM45686_RTC_MODE              (0b1 << 5)

#define ICM45686_I3C_STC_MODE_MASK     (0b1 << 2)
#define ICM45686_I3C_STC_MODE          (0b0 << 2)

#define ICM45686_ACCEL_SRC_CTRL_MASK    0b11
#define ICM45686_ACCEL_SRC_CTRL         0b10

#define ICM45686_GYRO_SRC_CTRL_MASK    (0b11 << 5)
#define ICM45686_GYRO_SRC_CTRL         (0b10 << 5)

#define ICM45686_OSC_ID_OVRD_MASK       0b1111
#define ICM45686_OSC_ID_OVRD_DEFAULT    0b0000
#define ICM45686_OSC_ID_OVRD_RELAXATION 0b0010
#define ICM45686_OSC_ID_OVRD_EXTERNAL   0b1000

#define ICM45686_ACCEL_LP_CLK_SEL_MASK  0b10000
#define ICM45686_ACCEL_LP_CLK_SEL       0b10000

//INT1 CONFIG
#define ICM45686_INT1_STATUS_EN_DRDY_MASK   0b100
#define ICM45686_INT1_STATUS_EN_DRDY        0b100

#define ICM45686_INT1_DRIVE_MASK            0b100
#define ICM45686_INT1_DRIVE                 0b000 //0: push-pull, 1: open drain

#define ICM45686_INT1_MODE_MASK             0b10
#define ICM45686_INT1_MODE                  0b00  //0: pulse mode, 1: latch mode

#define ICM45686_INT1_POLARITY_MASK         0b1
#define ICM45686_INT1_POLARITY              0b1   //0: active low, 1: active high

#define ICM45686_INT1_STATUS_EN_FIFO_THS_MASK (1 << 1)
#define ICM45686_INT1_STATUS_EN_FIFO_THS      (1 << 1)

//FIFO CONFIG.
//Not all FIFO config is contained in the "FIFO" prefixed registers. The remaining regs with bits are:
//INT1_CONFIG0: INT1_STATUS_EN_FIFO_THS. Enables FIFO treshold interrupt (basically the data ready int for FIFO)
//SMC_CONTROL_0(IPREG_TOP1): TMST_en. Timestamp enable for FIFO 

//FIFO_CONFIG_0
#define ICM45686_FIFO_MODE_MASK       ((1 << 7) | (1 << 6))
#define ICM45686_FIFO_MODE            ((0 << 7) | (1 << 6)) //00: Bypass. 01: Stream. 10: Stop on full. 11: Reserved

#define ICM45686_FIFO_DEPTH_MASK      ((1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0))
#define ICM45686_FIFO_DEPTH_2K        ((0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | (1 << 1) | (1 << 0)) //000111:2kB depth. 011111: 8kB depth, must disaple APEX
#define ICM45686_FIFO_DEPTH_8K        ((0 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0))


//The following two registers set the FIFO watermark threshold value.
//Settig both to zero will disable the watermark.
//FIFO_CONFIG1_0
#define ICM45686_FIFO_WM_TH_7_0_MASK    0xFF
#define ICM45686_FIFO_WM_TH_7_0         0x06 //0xA0, value from user guide

//FIFO_CONFIG1_1
#define ICM45686_FIFO_WM_TH_15_8_MASK   0xFF 
#define ICM45686_FIFO_WM_TH_15_8        0x00   //Default is 0

//FIFO_CONFIG2
#define ICM45686_FIFO_FLUSH_MASK       (1 << 7)
#define ICM45686_FIFO_FLUSH            (1 << 7) //Flushing the FIFO, pointers and control logic resets (not sure what that means)

#define ICM45686_FIFO_WR_WM_GT_TH_MASK (1 << 3) //Condition for generating interrupt. 
#define ICM45686_FIFO_WR_WM_GT_TH      (1 << 3) //0: Int when FIFO data count is EQUAL to watermark. 1: Same but EQUAL or GREATER 

//FIFO_CONFIG3
#define ICM45686_FIFO_ES1_EN_MASK      (1 << 5) //Enable External Sensor 1 data insertion into FIFO frame 
#define ICM45686_FIFO_ES1_EN           (0 << 5)

#define ICM45686_FIFO_ES0_EN_MASK      (1 << 4) //Enable External Sensor 0 data insertion into FIFO frame 
#define ICM45686_FIFO_ES0_EN           (0 << 4)

#define ICM45686_FIFO_HIRES_EN_MASK    (1 << 3) //Enable high resolution accel and gyro data in FIFO frame
#define ICM45686_FIFO_HIRES_EN         (1 << 3)

#define ICM45686_FIFO_GYRO_EN_MASK     (1 << 2) //Enable insertion of gyro data in FIFO frame
#define ICM45686_FIFO_GYRO_EN          (1 << 2)

#define ICM45686_FIFO_ACCEL_EN_MASK     (1 << 1) //Enable insertion of accel data in FIFO frame
#define ICM45686_FIFO_ACCEL_EN          (1 << 1)

#define ICM45686_FIFO_IF_EN_MASK        (1 << 0) //Not sure what this does but should be enabled when FIFO is enabled (not in bypass mode)
#define ICM45686_FIFO_IF_EN             (1 << 0) //To prevent power drain, FIFO_IF_EN should be disabled if bypass mode is used


//FIFO_CONFIG4
#define ICM45686_COMP_NC_FLOW_CFG_MASK ((1 << 5) | (1 << 4) | (1 << 3))
#define ICM45686_COMP_NC_FLOW_CFG      ((0 << 5) | (0 << 4) | (0 << 3)) //000: Non-compressed packet-flow is disabled

#define ICM45686_COMP_EN_MASK           (1 << 2)
#define ICM45686_COMP_EN                (0 << 2) //0: FIFO compression is disabled

#define ICM45686_FIFO_TMST_FSYNC_EN_MASK    (1 << 1) //Enable timestamp insertion in FIFO frame 
#define ICM45686_FIFO_TMST_FSYNC_EN         (1 << 1) //1: Inserts timestamp

#define ICM45686_FIFO_ES0_6B_9B_MASK    (1 << 0) //Number of bytes provided by external. Not relevant
#define ICM45686_FIFO_ES0_6B_9B         (0 << 0) 


//BANKS
#define ICM45686_IMEM_SRAM   0x0000
#define ICM45686_IPREG_BAR   0xA000
#define ICM45686_IPREG_SYS1  0xA400
#define ICM45686_IPREG_SYS2  0xA500
#define ICM45686_IPREG_TOP1  0xA200
#define ICM45686_USER_BANK_0 0x0069 //Just some number, not used for address offset like the others

//IREGS bits and masks
#define ICM45686_SREG_DATA_ENDIAN_SEL 1

#define ICM45686_TMST_EN_MASK  (1 << 0)
#define ICM45686_TMST_EN       (1 << 0)

#endif
