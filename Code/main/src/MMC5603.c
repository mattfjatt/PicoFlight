#include "headers/MMC5603.h"


void MMC5603_init()
{
    printf("File path = %s. Line number = %d. Function name = %s\n", __FILE__, __LINE__, __func__);
    MMC5603_i2c_setup();
    MMC5603_setup();
    MMC5603_calibrate_magnetometer();
}

void MMC5603_setup()
{
    
    //This chip uses a sensor type that needs to be recalibrated internally *a lot*, this can be handled by the chip automatically
    //by setting the Auto_SR_en bit in internal control 0
    i2c_write_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_INTERNAL_CONTROL_0, (1 << MMC_5603_AUTO_SR_EN));


    //Set the bandwidth selection bits in internal control 1. The default measurement time is 6.6ms which caps out at 150 Hz
    //Setting BW1 = 1 and BW0 = 0 allows up to 255Hz ODR
    i2c_write_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_INTERNAL_CONTROL_1, (1 << MMC5603_BW1) | (0 << MMC5603_BW0));

    //Set ODR. Last arg is the ODR in Hz
    i2c_write_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_ODR, 250);

    //CMM_FREQ_EN calculates measurement period according to ODR. NOTE: This must be set AFTER ODR has been set, otherwise the sensor behaves strange
    i2c_write_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_INTERNAL_CONTROL_0, (1 << MMC_5603_AUTO_SR_EN) | (1 << MMC5603_CMM_FREQ_EN));

    //Next, we want to set the sensor to continuous mode to avoid having to ask the sensor to take a measurement each time
    i2c_write_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_INTERNAL_CONTROL_2, (1 << MMC5603_CMM_EN));

}

void MMC5603_i2c_setup()
{
    // This example will use I2C1 on SDA and SCL pins (GP26, GP27 on a Pico)
    int ret = i2c_init(i2c1, 400 * 1000); //Fast mode plus at 1MHz is the fastest supported mode on the RP2350
    uint8_t GPIO_26_SDA = 26;
    uint8_t GPIO_27_SCL = 27;
    gpio_set_function(GPIO_26_SDA, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_27_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_26_SDA);
    gpio_pull_up(GPIO_27_SCL);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(26, 27, GPIO_FUNC_I2C));
}

void MMC5603_calibrate_magnetometer()
{
    printf("Gathering data points in\n");

    for(int i = 3; i >= 0; i--){
        printf("%d second(s)\n", i);
        sleep_ms(1000);
    }

    MMC5603_gather_samples_for_calib(); //Takes roughly 10 seconds
    printf("Optimizing...");
    Namespace_LM_solver();
}

void MMC5603_gather_samples_for_calib()
{
    //Gather 1000 samples at approximately 100Hz, gives 10 seconds to give a decent point cloud
    Sample si;
    for(int i = 0; i < SAMPLE_COUNT; i++){
        MMC5603_get_mag_reading(&si);
        Samples[i] = si;
        sleep_ms(10);
        if(!(i % (SAMPLE_COUNT/10))){
            printf("%d samples gathered\n", i);
        }
    }
    printf("%d samples gathered\n", SAMPLE_COUNT);
}

void MMC5603_get_mag_reading(Sample* si)
{
    //Want to read nine bytes for 20bit xyz measurements:
    uint8_t mag_data_buffer[9];
    i2c_read_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_XOUT0, mag_data_buffer, sizeof(mag_data_buffer));

    uint32_t LSB_x, MSB_x, Lower4_x;
    uint32_t LSB_y, MSB_y, Lower4_y;
    uint32_t LSB_z, MSB_z, Lower4_z;

    MSB_x = mag_data_buffer[0];
    LSB_x = mag_data_buffer[1];
    MSB_y = mag_data_buffer[2];
    LSB_y = mag_data_buffer[3];
    MSB_z = mag_data_buffer[4];
    LSB_z = mag_data_buffer[5];

    Lower4_x = mag_data_buffer[6];
    Lower4_y = mag_data_buffer[7];
    Lower4_z = mag_data_buffer[8];

    uint32_t raw_x = (MSB_x << 12) | (LSB_x << 4) | (Lower4_x >> 4);
    uint32_t raw_y = (MSB_y << 12) | (LSB_y << 4) | (Lower4_y >> 4);
    uint32_t raw_z = (MSB_z << 12) | (LSB_z << 4) | (Lower4_z >> 4);

    si->x = ((int32_t)raw_x - 524288)*0.0625/1000.0;
    si->y = ((int32_t)raw_y - 524288)*0.0625/1000.0;
    si->z = ((int32_t)raw_z - 524288)*0.0625/1000.0;
}

void MMC5603_read_temp()
{
    //Make the sensor take a new temp measurement
    i2c_write_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_INTERNAL_CONTROL_0, 1 << MMC5603_TAKE_TMP_MEAS);
    uint32_t start, duration;
    while(1){
        uint8_t read_buffer;
        start = time_us_32();
        //Wait for the data in TOUT to be ready:
        i2c_read_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_STATUS1, &read_buffer,sizeof(read_buffer));

        if(read_buffer & 1 << MMC5603_MEAS_TMP_DONE){
            //Read the data that is ready in TOUT
            i2c_read_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_TOUT, &read_buffer,sizeof(read_buffer));
            
            //Make the sensor take a new temp measurement
            i2c_write_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_INTERNAL_CONTROL_0, 1 << MMC5603_TAKE_TMP_MEAS);
            duration = time_us_32() - start;
            printf("Read time is us = %d. Temp is = %f\n", duration, read_buffer*0.8 - 75.0);
        }
        sleep_ms(100);
    }
}

void i2c_read_register(i2c_inst_t* i2c, uint8_t dev_address, uint8_t reg_address, uint8_t* read_buffer, uint8_t len)
{
    //If we want to read more than 1 byte, the first byte of read_buffer must be set to reg_address
    //If we only want to read one byte, set read_buffer = reg_address
    read_buffer[0] = reg_address;
    int err;
    err = i2c_write_blocking(i2c, MMC5603_I2C_ADDRESS, read_buffer, 1,true);
    err = i2c_read_blocking(i2c, MMC5603_I2C_ADDRESS, read_buffer, len,false);
}

void i2c_write_register(i2c_inst_t* i2c, uint8_t dev_address, uint8_t reg_address, uint8_t reg_value)
{
    uint8_t write_buffer[2];
    write_buffer[0] = reg_address;
    write_buffer[1] = reg_value;
    int err = i2c_write_blocking(i2c, dev_address, write_buffer,sizeof(write_buffer),true);
}
