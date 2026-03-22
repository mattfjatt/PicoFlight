#ifndef PINS_H
#define PINS_H
//Here, gpio pins are defined
//TODO: Change to enums?

/*
//These are the pins for the MPU6050-based PCB for receiver pins and pwm pins
//PWM-pins for servo/ESC control
#define SERVO_0 18 //Slice 1, motor0
#define SERVO_1 19 //Slice 1, motor1
#define SERVO_2 20 //Slice 2, motor2
#define SERVO_3 21 //Slice 2, motor3, conflicting with the clock pin for ICM45686!

//Receiver-pins
#define GPIO_CH_0 10
#define GPIO_CH_1 11
#define GPIO_CH_2 12
#define GPIO_CH_3 13
*/

//These are the pins for the ICM45686-based PCB for receiver pins and pwm pins

//PWM-pins for servo/ESC control
//TODO: Change the names SERVO_x and GPIO_CH_x to something that more clearly indicates what the pins do
#define SERVO_0 36
#define SERVO_1 37
#define SERVO_2 38
#define SERVO_3 39 

//Receiver-pins
#define GPIO_CH_0 25
#define GPIO_CH_1 26
#define GPIO_CH_2 27
#define GPIO_CH_3 28

//SPI-pins
#define SPI0_MOSI 3
#define SPI0_MISO 4
#define SPI0_SCK  2

#define SPI1_MOSI 11
#define SPI1_MISO 12
#define SPI1_SCK  10

#define ICM20948_CS 5
#define ICM45686_CS 6
#define MMC5983_CS  13
#define BMP388_CS   16


//Sensor interrupt pins
#define ICM45686_INT_PIN 14
#define MMC5983_INT_PIN 15
#define BMP388_INT_PIN 17

//Clock-pin
//gpios connected to the clock system are 21 23 24 25
#define ICM45686_PICO_CLOCK_PIN 21 //May conflict with servo! But is the only clock pin available on the pico 2! Do not run with servo!


//I2C-pins
//These pins are used such that the ICM20498 can use pins 2 3 4 5, but this will not work on the drone 
//as the pins are hardwired in the pcb! Use the default values for the drone of SDA = 4 and SCL = 5
#define I2C0_SDA 8
#define I2C0_SCL 9

#define I2C1_SDA 26
#define I2C1_SCL 27

#endif
