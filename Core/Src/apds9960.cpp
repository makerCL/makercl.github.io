
#include "APDS9960.h"
	/**
	* @brief APDS9960 constructor
	*
	* @param i2cHandle I2C handler
	* @param uartHandle UART handler
	* @param Atime Integration time for photodiode
	*/
APDS9960::APDS9960(I2C_HandleTypeDef* i2cHandle, UART_HandleTypeDef* uartHandle, uint16_t Atime) {

	hi2c = i2cHandle;
	huart = uartHandle;
	RGB_Margin = 20;
	ATIME = Atime;
	strcpy(ball_color, "RED"); // default team color to RED

	initialize();
}

	/**
	* @brief Initialize the APDS9960 sensor
	*
	* @return uint8_t Returns 1 if initialization is successful, 0 otherwise
	*/
uint8_t APDS9960::initialize() {

	uint8_t id = readReg(0x92);
	if (id != 0xab) {
		HAL_UART_Transmit(huart, (uint8_t*)"RGB ID NOT FOUND", strlen("RGB ID NOT FOUND"), HAL_MAX_DELAY);
		return 0;
	} else {
		HAL_UART_Transmit(huart, (uint8_t*)"RGB Sensor initialized\r\n", strlen("RGB Sensor initialized\r\n"), HAL_MAX_DELAY);
	}
    writeReg(0x80, 0b00001011); // Enables, just using ALS/color feature.
    writeReg(0x83, 250); // Increase Wait time
    writeReg(0x90, 0b00001000); // Increase LED Strength
    writeReg(0x81, ATIME); // Set ATIME for photodiode integration time


    colorSet();

    return 1;

}

    /**
     * @brief Read the RGBC (Red, Green, Blue, Clear) values from the APDS9960 sensor
     */
void APDS9960::readRGBC() {

    RGBC_Buffer[0] = readReg16(0x96);
    RGBC_Buffer[1] = readReg16(0x98);
    RGBC_Buffer[2] = readReg16(0x9A);
    RGBC_Buffer[3] = readReg16(0x94);
}

/**
 * @brief Sort colors based on RGBC values
 * 
 * @return bool Returns true if the color is within the target range, false otherwise
 */
bool APDS9960::ballDetect() {
	readRGBC();

	if (RGBC_Buffer[3] > 30) {
		return true;
	} else {
		return false;
	}
}


/**
 * @brief Sort colors based on RGBC values
 * 
 * @return bool Returns true if the color is within the target range, false otherwise
 */
bool APDS9960::colorSort() {
    // Duplicated local copies for readability
	uint16_t targetR = RGBC_Select[0];
    uint16_t targetG = RGBC_Select[1];
    uint16_t targetB = RGBC_Select[2];
    uint16_t targetC = RGBC_Select[3];

    uint16_t r = RGBC_Buffer[0];
    uint16_t g = RGBC_Buffer[1];
    uint16_t b = RGBC_Buffer[2];
    uint16_t c = RGBC_Buffer[3];



    if (r >= targetR - r && r <= targetR + RGB_Margin &&
        g >= targetG - RGB_Margin && g <= targetG + RGB_Margin &&
        b >= targetB - RGB_Margin && b <= targetB + RGB_Margin) {
        return true;
    }
    return false;
}

/**
 * @brief Set the target color
 */
void APDS9960::colorSet() {
    if (strcmp(ball_color, "RED") == 0) {
        RGBC_Select[0] = 47;
        RGBC_Select[1] = 9;
        RGBC_Select[2] = 10;
        RGBC_Select[3] = 60;
    } else if (strcmp(ball_color, "LIGHTYELLOW") == 0) {
        RGBC_Select[0] = 125;
        RGBC_Select[1] = 55;
        RGBC_Select[2] = 27;
        RGBC_Select[3] = 180;
    }
}

/**
 * @brief Read a register from the APDS9960 sensor
 * 
 * @param reg Register to read
 * @return uint8_t The value read from the register
 */
uint8_t APDS9960::readReg(uint8_t reg) {
    uint8_t value = 0;
    HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
    return value;
}

/**
 * @brief Read a 16-bit register from the APDS9960 sensor
 * 
 * @param reg Register to read
 * @return uint16_t The value read from the register
 */
uint16_t APDS9960::readReg16(uint8_t reg) {
    uint8_t value1, value2;
    HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value1, 1, 1000);
    HAL_I2C_Mem_Read(hi2c, APDS9960_I2C_ADDR, reg + 1, I2C_MEMADD_SIZE_8BIT, &value2, 1, 1000);
    uint16_t result = (uint16_t)(value2 << 8 | value1);
    return result;
}

/**
 * @brief Write a value to a register on the APDS9960 sensor
 * 
 * @param reg Register to write to
 * @param value Value to write to the register
 */
void APDS9960::writeReg(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(hi2c, APDS9960_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}

/**
 * @brief Print the RGBC buffer values
 */
void APDS9960::printRGBCBuffer() {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "R: %u, G: %u, B: %u, C: %u\r\n", RGBC_Buffer[0], RGBC_Buffer[1], RGBC_Buffer[2], RGBC_Buffer[3]);
    HAL_UART_Transmit(huart, (uint8_t*)(buffer), strlen(buffer), HAL_MAX_DELAY);
}







