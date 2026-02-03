#include "headers/MPU6050.h"

/*
   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050 board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/


void MPU6050_i2c_setup()
{
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c0, 400 * 1000); //i2c0 = i2c_default
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void MPU6050_read_byte_register_I2C(uint8_t i2c_dev_add, uint8_t i2c_dev_reg)
{
    uint8_t read_buffer = i2c_dev_reg;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &read_buffer, 1,true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, &read_buffer, 1,true);
    
    printf("0b");
    MPU6050_print_binary(read_buffer);
}

void MPU6050_reset() {
    uint8_t buf[] = {MPU6050_POWER_MANAGEMENT_1, 0b10000000}; //This sets bit7 to 1, resetting the MPU6050
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0b00000000;  // Clear sleep mode by writing 0 to the power management register
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up
}


void MPU6050_setup()
{
    int err;
    //Setup gyro:
    write_buffer[0] = MPU6050_GYRO_CONFIG;
    write_buffer[1] = 0b00001000;
    err = i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, write_buffer,2,true);

    printf("MPU_GYRO_CONFIG: ");
    MPU6050_read_byte_register_I2C(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG);
    sleep_ms(50);

    //Setup accel:
    write_buffer[0] = MPU6050_ACCEL_CONFIG;
    write_buffer[1] = 0b00001000;
    err = i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, write_buffer,2,true);
    printf("MPU_ACCEL_CONFIG: ");
    MPU6050_read_byte_register_I2C(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG);
    sleep_ms(50);  
}

void MPU6050_init()
{
    MPU6050_i2c_setup();
    MPU6050_reset();
    MPU6050_setup();
}



void MPU6050_get_imu_data(double acc[3], double gyr[3]){
    int err;
	uint8_t buff[14]; //14 bytes in total
	buff[0] = MPU6050_ACCEL_START;
	//err = i2c_write(i2c_dev,buff,1,MPU6050_I2C_ADDRESS);
    err = i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, buff, 1, true);
	//err = i2c_read(i2c_dev,buff,14,MPU6050_I2C_ADDRESS); 
    err = i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, buff, 14, false);
    uint16_t LSB_x, MSB_x, LSB_y, MSB_y, LSB_z, MSB_z;

    //Acc x:
	MSB_x = buff[0];
	LSB_x = buff[1];
    //Acc y:
    MSB_y = buff[2];
	LSB_y = buff[3];
    //Acc z:
    MSB_z = buff[4];
	LSB_z = buff[5];

    int16_t ACC_x = LSB_x | MSB_x << 8;
    int16_t ACC_y = LSB_y | MSB_y << 8;
	int16_t ACC_z = LSB_z | MSB_z << 8;
    //Scaling factor values(not from data sheet, it only provides sacling for 2000 dps):
    //ACCEL_FS_SEL = 0b00011000, +-16g: 1/2048
    //ACCEL_FS_SEL = 0b00010000,  +-8g: 1/4096
    //ACCEL_FS_SEL = 0b00001000,  +-4g: 1/8192
    //ACCEL_FS_SEL = 0b00000000,  +-2g: 1/16384
    double scaling_factor_acc = 1/8192.0;
    //Default MPU6050 coordinate frame...
    // acc[0] = ACC_x*scaling_factor_acc;
    // acc[1] = ACC_y*scaling_factor_acc;
    // acc[2] = ACC_z*scaling_factor_acc;
    //Rotating to NED:
    acc[0] =  ACC_y*scaling_factor_acc;
    acc[1] =  ACC_x*scaling_factor_acc;
    acc[2] = -ACC_z*scaling_factor_acc;



    //Temp: Not used atm
	MSB_x = buff[6];
	LSB_x = buff[7];

    int16_t temp = LSB_x | MSB_x << 8;

    //Gyro x:
	MSB_x = buff[8];
	LSB_x = buff[9];
    //Gyro y:
    MSB_y = buff[10];
	LSB_y = buff[11];
    //Gyro z:
    MSB_z = buff[12];
	LSB_z = buff[13];

    int16_t ARS_x = LSB_x | MSB_x << 8;
    int16_t ARS_y = LSB_y | MSB_y << 8;
	int16_t ARS_z = LSB_z | MSB_z << 8;
    //Scaling factor values(not from data sheet, it only provides sacling for 2000 dps):
    //GYRO_FS_SEL = 0b00011000, 2000 dps: 1/16.4
    //GYRO_FS_SEL = 0b00010000, 1000 dps: 1/32.8
    //GYRO_FS_SEL = 0b00001000, 500  dps: 1/65.5
    //GYRO_FS_SEL = 0b00000000, 250  dps: 1/131
    double scaling_factor_gyr = 1/65.5;
    double d2r = 3.14159265/180;
    //This register mapping to x,y,z is the default coordinate frame on the MPU6050...
    // gyr[0] = ARS_x*scaling_factor_gyr*d2r; //Gyro x
    // gyr[1] = ARS_y*scaling_factor_gyr*d2r; //Gyro y
    // gyr[2] = ARS_z*scaling_factor_gyr*d2r; //Gyro z
    //...but we want the north-east-down (NED) frame. Note how x and y are switched and z is flipped
    gyr[0] =  ARS_y*scaling_factor_gyr*d2r; //Gyro x in NED
    gyr[1] =  ARS_x*scaling_factor_gyr*d2r; //Gyro y in NED
    gyr[2] =  - ARS_z*scaling_factor_gyr*d2r; //Gyro z in NED
}

void MPU6050_print_binary(uint32_t num) {
    for (int i = 7; i >= 0; i--) {
        if (num & (1 << i))
            printf("1");
        else
            printf("0");

        if (i % 8 == 0)
            printf(" ");
    }
    printf("\n");
}
