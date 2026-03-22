#include "headers/bmp388.h"

bmpCompensationParameters bmp388_comp;

static volatile uint32_t int_counter = 0;
static volatile uint32_t pin = 0;
static volatile uint32_t data_ready = 0;

//This source file is not ready, but all pin-functionality of the sensor has been confirmed 
//to work and can now be routed on the PCB

void bmp388_init()
{
    bmp388_setup();
}


void bmp388_setup()
{
    uint8_t tx_buf[3] = {0};
    uint8_t rx_buf[3] = {0};

    gpio_init(BMP388_CS);
    gpio_set_dir(BMP388_CS, GPIO_OUT);
    gpio_put(BMP388_CS,1); //CS is set to high, this is the idle state
    sleep_ms(50);

    bmp388_read_modify_write_register(BMP388_PWR_CTRL, BMP388_PRESS_EN |
                                                       BMP388_TEMP_EN |
                                                       BMP388_MODE,
                                                       BMP388_PRESS_EN_MASK |
                                                       BMP388_TEMP_EN_MASK |
                                                       BMP388_MODE_MASK,
                                                       BMP388_CS);

    //Setup interrupt for data ready.
    //When setting bit 6 to 1, the entire reg changes to 0b01011111
    bmp388_read_modify_write_register(BMP388_INT_CTRL,
                                      BMP388_INT_LEVEL |
                                      BMP388_DRDY_EN,
                                      BMP388_INT_OD_MASK |
                                      BMP388_INT_LEVEL_MASK |
                                      BMP388_INT_LATCH_MASK |
                                      BMP388_FWTM_EN_MASK |
                                      BMP388_FFULL_EN_MASK |
                                      BMP388_DRDY_EN_MASK, BMP388_CS);

    bmp388_read_from_register(BMP388_INT_CTRL,tx_buf,rx_buf,sizeof(tx_buf), BMP388_CS);

    bmp388_load_compensation_params(&bmp388_comp);

    gpio_init(BMP388_INT_PIN);
    gpio_set_dir(BMP388_INT_PIN, GPIO_IN); //high impedance mode
    gpio_set_irq_enabled_with_callback(BMP388_INT_PIN, GPIO_IRQ_EDGE_RISE, true, bmp388_int_callback);

    float temp;
    float pres;

    while(1){
        // bmp388_get_compensated_pressure_temp(&pres, &temp, &bmp388_comp);
        // PRINTNUM("Pres = %f\n", pres);
        if(data_ready){
            data_ready = 0;
            PRINTNUM("interrupts = %u\n", int_counter);
        }

        sleep_ms(1000);
    }
}

void bmp388_int_callback(uint gpio, uint32_t events)
{
    int_counter++;
    pin = gpio;
    data_ready = 1;
}

void bmp388_load_compensation_params(bmpCompensationParameters* comp)
{
    //Going to load a total of 21 bytes -> buffers must be 23 bytes
    uint8_t tx_buf[23] = {0};
    uint8_t rx_buf[23] = {0};

    bmp388_read_from_register(BMP388_COMP_PARAMS_START, tx_buf, rx_buf, sizeof(tx_buf), BMP388_CS);

    //Temp correction parameters
    comp->t1 = (((uint16_t)rx_buf[3]) << 8 | rx_buf[2])*pow(2.0, 8.0);
    comp->t2 = (((uint16_t)rx_buf[5]) << 8 | rx_buf[4])/pow(2.0, 30.0);
    comp->t3 = ((int8_t)rx_buf[6])/pow(2.0, 48.0);

    //Pressure correction parameters
    comp->p1 = (((int16_t)(((uint16_t)rx_buf[8]) << 8 | rx_buf[7])) - pow(2.0, 14.0))/pow(2.0, 20.0); 
    comp->p2 = (((int16_t)(((uint16_t)rx_buf[10]) << 8 | rx_buf[9])) - pow(2.0, 14.0))/pow(2.0, 29.0);
    comp->p3 = ((int8_t)rx_buf[11])/pow(2.0, 32.0);
    comp->p4 = ((int8_t)rx_buf[12])/pow(2.0, 37);
    comp->p5 = ((uint16_t)(((uint16_t)rx_buf[14]) << 8 | rx_buf[13]))*pow(2.0, 3.0);
    comp->p6 = ((uint16_t)(((uint16_t)rx_buf[16]) << 8 | rx_buf[15]))/pow(2.0, 6.0);
    comp->p7 = ((int8_t)rx_buf[17])/pow(2.0, 8.0);
    comp->p8 = ((int8_t)rx_buf[18])/pow(2.0, 15.0);
    comp->p9 = ((int16_t)(((uint16_t)rx_buf[20]) << 8 | rx_buf[19]))/pow(2.0, 48.0);
    comp->p10 = ((int8_t)rx_buf[21])/pow(2.0, 48.0);
    comp->p11 = ((int8_t)rx_buf[22])/pow(2.0, 65);
}

void bmp388_read_modify_write_register(uint8_t dev_register, uint8_t bits_to_update, uint8_t mask, uint8_t cs_pin)
{
    //The mask signifies which bits to update. mask = 0b00000001 means leave all bits as-is except maybe bit 0
    uint8_t tx_buf[2] = {0};
    uint8_t rx_buf[2] = {0};

    bmp388_read_from_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf),cs_pin);
    uint8_t value_from_register = rx_buf[1];
    value_from_register &= ~(mask); //This sets all bits we want to update to 0
    uint8_t value_to_register = value_from_register | (bits_to_update & mask);
    tx_buf[1] = value_to_register;
    bmp388_write_to_register(dev_register,tx_buf,rx_buf,sizeof(tx_buf), cs_pin);
}

void bmp388_read_from_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register | SET_SPI_READ; //Sets the read bit, bit 7, to 1
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;

    gpio_put(cs_pin,0); //Chip select is active low
    spi_write_read_blocking(spi1, tx_buf, rx_buf, n_bytes);
    gpio_put(cs_pin,1);
}

void bmp388_get_compensated_pressure_temp(float* pres_comp, float* temp_comp, bmpCompensationParameters* comp_params)
{
    uint32_t pres_raw, temp_raw;
    bmp388_get_raw_pressure_temp(&pres_raw, &temp_raw);
    *temp_comp = bmp388_compensate_temperature(temp_raw, comp_params);
    *pres_comp = bmp388_compensate_pressure(pres_raw, comp_params);
}

void bmp388_get_raw_pressure_temp(uint32_t* pres_raw, uint32_t* temp_raw)
{
    uint8_t tx_buf[8] = {0};
    uint8_t rx_buf[8] = {0};

    bmp388_read_from_register(BMP388_DATA_0, tx_buf, rx_buf, sizeof(tx_buf), BMP388_CS);

    uint32_t pres_7_0 = rx_buf[2];
    uint32_t pres_15_8 = rx_buf[3];
    uint32_t pres_23_16 = rx_buf[4];

    uint32_t temp_7_0 = rx_buf[5];
    uint32_t temp_15_8 = rx_buf[6];
    uint32_t temp_23_16 = rx_buf[7];

    *pres_raw = pres_23_16 << 16 | pres_15_8 << 8 | pres_7_0;
    *temp_raw = temp_23_16 << 16 | temp_15_8 << 8 | temp_7_0;
}

float bmp388_compensate_temperature(uint32_t uncomp_temp, bmpCompensationParameters* comp_params)
{
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - comp_params->t1);
    partial_data2 = (float)(partial_data1 * comp_params->t2);
    //Update the compensated temperature in calib structure since this is
    //needed for pressure calculation
    comp_params->t_lin = partial_data2 + (partial_data1 * partial_data1) * comp_params->t3;

    //Returns compensated temperature
    return comp_params->t_lin;
}

float bmp388_compensate_pressure(uint32_t uncomp_press, bmpCompensationParameters* comp_params)
{
    /* Variable to store the compensated pressure */
    float comp_press;
    /* Temporary variables used for compensation */
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;
    /* Calibration data */

    partial_data1 = comp_params->p6 * comp_params->t_lin;
    partial_data2 = comp_params->p7* (comp_params->t_lin * comp_params->t_lin);
    partial_data3 = comp_params->p8* (comp_params->t_lin * comp_params->t_lin * comp_params->t_lin);
    partial_out1 = comp_params->p5+ partial_data1 + partial_data2 + partial_data3;

    partial_data1 = comp_params->p2 * comp_params->t_lin;
    partial_data2 = comp_params->p3 * (comp_params->t_lin * comp_params->t_lin);
    partial_data3 = comp_params->p4 * (comp_params->t_lin * comp_params->t_lin * comp_params->t_lin);
    partial_out2 = (float)uncomp_press * (comp_params->p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = comp_params->p9 + comp_params->p10* comp_params->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * comp_params->p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;


    return comp_press;
}

void bmp388_write_to_register(uint8_t dev_register, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t n_bytes, uint8_t cs_pin)
{
    tx_buf[0] = dev_register & 0x7F; //Explicitly sets the read bit to 0
    gpio_put(cs_pin,0); //Chip select is active low
    //spi_write_read_blocking(spi1, tx_buf, rx_buf, n_bytes);
    spi_write_blocking(spi1, tx_buf, n_bytes);
    gpio_put(cs_pin,1);
}
