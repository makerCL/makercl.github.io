#ifndef APDS9960_H
#define APDS9960_H

#include "stm32f4xx_hal.h"


#include <cstdint>
#include <cstdio>
#include <cstring>

class APDS9960 {
public:
    APDS9960(I2C_HandleTypeDef* i2cHandle, UART_HandleTypeDef* uartHandle, uint16_t ATIME);

    char ball_color[13];

    uint8_t initialize();
    void readRGBC();
    bool ballDetect();
    bool colorSort();

    void colorSet();

    static const uint8_t APDS9960_I2C_ADDR = 0x72;
    uint16_t RGBC_Buffer[4] = {0, 0, 0, 0}; // RGBC values from sensor
    uint16_t RGBC_Select[4] = {0, 0, 0, 0}; // RGB Values of ball to collect
    uint8_t RGB_Margin;
    uint16_t ATIME;


    uint8_t readReg(uint8_t reg);
    uint16_t readReg16(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t value);

    void printRGBCBuffer();



private:
    I2C_HandleTypeDef* hi2c;
    UART_HandleTypeDef* huart;

};

#endif
