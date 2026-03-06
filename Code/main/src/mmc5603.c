#include "headers/mmc5603.h"

static double mag_correction_mat[3][3];
static double offset[3];
static double solution[9];

void mmc5603_init()
{
    mmc5603_setup();
    mmc5603_calibrate_magnetometer();
}

void mmc5603_setup()
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


void mmc5603_calibrate_magnetometer()
{
    PRINT("Gathering data points in\n");

    for(int i = 3; i >= 1; i--){
        PRINTNUM("%d second(s)\n", i);
        sleep_ms(1000);
    }

    mmc5603_gather_samples_for_calib(); //Takes roughly 10 seconds
    PRINT("Optimizing...\n");
    optimizer_LM_solver(solution);
    mmc5603_get_magnetometer_calib(solution,mag_correction_mat,offset);
}

void mmc5603_gather_samples_for_calib()
{
    //Gather 1000 samples at approximately 100Hz, gives 10 seconds to give a decent point cloud
    Sample si;
    for(int i = 0; i < SAMPLE_COUNT; i++){
        mmc5603_get_mag_reading(&si);
        samples[i] = si;
        sleep_ms(10);
        if(!(i % (SAMPLE_COUNT/10))){
            PRINTNUM("%d samples gathered\n", i);
        }
    }
    PRINTNUM("%d samples gathered\n", SAMPLE_COUNT);
}

void mmc5603_get_mag_reading(Sample* si)
{
    //Want to read nine bytes for 20bit xyz measurements:
    uint8_t mag_data_buffer[9];
    i2c_read_register(i2c1, MMC5603_I2C_ADDRESS, MMC5603_XOUT0, mag_data_buffer, sizeof(mag_data_buffer));

    uint32_t lsb_x, msb_x, lower4_x;
    uint32_t lsb_y, msb_y, lower4_y;
    uint32_t lsb_z, msb_z, lower4_z;

    msb_x = mag_data_buffer[0];
    lsb_x = mag_data_buffer[1];
    msb_y = mag_data_buffer[2];
    lsb_y = mag_data_buffer[3];
    msb_z = mag_data_buffer[4];
    lsb_z = mag_data_buffer[5];

    lower4_x = mag_data_buffer[6];
    lower4_y = mag_data_buffer[7];
    lower4_z = mag_data_buffer[8];

    uint32_t raw_x = (msb_x << 12) | (lsb_x << 4) | (lower4_x >> 4);
    uint32_t raw_y = (msb_y << 12) | (lsb_y << 4) | (lower4_y >> 4);
    uint32_t raw_z = (msb_z << 12) | (lsb_z << 4) | (lower4_z >> 4);

    //Need to convert this to NED frame as with the IMU.
    //Note that the frame printed on the Adafruit MMC5603 is wrong!
    si->x =   ((int32_t)raw_x - 524288)*0.0625/1000.0;
    si->y = - ((int32_t)raw_y - 524288)*0.0625/1000.0;
    si->z = - ((int32_t)raw_z - 524288)*0.0625/1000.0;
}

void mmc5603_read_temp()
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
        }
        sleep_ms(100);
    }
}

void mmc5603_get_magnetometer_calib(double theta[9], double correction_matrix[3][3], double correction_vector[3])
{
    //theta = [Ba = 1/a^2 Bb = 1/b^2 Bc = 1/c^2 px py pz S0 S1 S2]
    //Cholesky decompose
    double L[3][3]; 
    L[0][0] = sqrt(theta[0]);
    L[1][0] = theta[6]/L[0][0];
    L[2][0] = theta[7]/L[0][0];
    L[1][1] = sqrt(theta[1] - L[1][0]*L[1][0]);
    L[2][1] = (theta[8] - L[1][0]*L[2][0])/L[1][1];
    L[2][2] = sqrt(theta[2] - L[2][0]*L[2][0] - L[2][1]*L[2][1]);
    linalg_mattranspose(3,3,L,correction_matrix);
    correction_vector[0] = theta[3];
    correction_vector[1] = theta[4];
    correction_vector[2] = theta[5];
}

void mmc5603_adjust_mag_vector(double correction_matrix[3][3], double correction_vector[3], Sample si, double m_corr[3])
{
    int N = 3;
    double m_raw[3];
    m_raw[0] = si.x;
    m_raw[1] = si.y;
    m_raw[2] = si.z;
    linalg_vecvecsub(N,m_raw,correction_vector,m_corr); //m_corr = m_raw - correction_vector
    linalg_matvecmul(N,N,correction_matrix, m_corr, m_corr); //m_corr <- A*m_corr
}

void mmc5603_get_corrected_mag_reading(double m_corr[3])
{
    Sample si;
    mmc5603_get_mag_reading(&si);
    mmc5603_adjust_mag_vector(mag_correction_mat,offset,si,m_corr);
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
