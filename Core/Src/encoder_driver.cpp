/*
 * encoder_driver.cpp
 *
 *  Created on: Jun 5, 2023
 *      Author: John Bennett
 */

#include "encoder_driver.h"
#include "string.h"

	/**
	* @brief Creates an encoder structure
	*
	* @param GPIO_Pin1 Specifies the port bit to be written for the first encoder output.
	* @param GPIOx1 Input of GPIOx where x can be (A..H) to select the GPIO peripheral of the second encoder
	* @param GPIO_Pin2 Specifies the port bit to be written for the first encoder output.
	* @param GPIOx2 Input of GPIOx where x can be (A..H) to select the GPIO peripheral of the second encoder
	* @param TPR conversion for ticks per revolution
	* @return encoder_drv_t Returns encoder driver structure
	*/
encoder_drv_t init_encoder(uint16_t	GPIO_Pin1, GPIO_TypeDef* GPIOx1, uint16_t GPIO_Pin2, GPIO_TypeDef* GPIOx2, int32_t TPR){
	encoder_drv_t encoder_drv = {0,0,0, GPIO_Pin1, GPIOx1, GPIO_Pin2,	GPIOx2, TPR, 0};
	encoder_drv.LAST_ENCODER_COUNT = NewState(&encoder_drv);
	return encoder_drv;
}

	/**
	* @brief Updates Encoder Position increments TOTAL_COUNT up or down depending on the LAST_ENCODER_COUNT.
	*
	* @param encoder_drv encoder driver structure.
	*
	*/
void update_encoder(encoder_drv_t* encoder_drv){
	uint8_t NEW_COUNT = NewState(encoder_drv);
	if (NEW_COUNT - encoder_drv->LAST_ENCODER_COUNT == 1){
		encoder_drv->TOTAL_COUNT++;
	} else if ((encoder_drv->LAST_ENCODER_COUNT - NEW_COUNT == 1) || (NEW_COUNT == 3 && encoder_drv->LAST_ENCODER_COUNT == 0)){
		encoder_drv->TOTAL_COUNT--;
	} else if (NEW_COUNT == 0 && encoder_drv->LAST_ENCODER_COUNT == 3){
		encoder_drv->TOTAL_COUNT++;
	} else {
		//zero(encoder_drv);
	}
	encoder_drv->LAST_ENCODER_COUNT = NEW_COUNT;
}

	/**
	* @brief Updates Encoder state manly updating the pos the number of revolution.
	*
	* @param encoder_drv encoder driver structure.
	*
	*/
void Update_Encoder_State(encoder_drv_t* encoder_drv){
	encoder_drv->pos = encoder_drv->TOTAL_COUNT/encoder_drv->TPR;
}

	/**
	* @brief zeros encoder.
	*
	* @param encoder_drv encoder driver structure.
	*
	*/
void zero(encoder_drv_t* encoder_drv){
	encoder_drv->TOTAL_COUNT = 0;
	encoder_drv->LAST_COUNT = 0;
}

// private functions

	/**
	* @brief represents encoder GPIO pin states as gray code then converts the gray code to a binary value representing encoder states.
	*
	* @param encoder_drv encoder driver structure.
	* @return uint8_t Returns binary representation of pin state
	*/
uint8_t NewState(encoder_drv_t* encoder_drv){
	uint8_t gray = (HAL_GPIO_ReadPin(encoder_drv->GPIOx1, encoder_drv->GPIO_Pin1) << 1) | HAL_GPIO_ReadPin(encoder_drv->GPIOx2, encoder_drv->GPIO_Pin2);
    uint8_t binary = 0;
	while (gray != 0) {
        binary ^= gray;
        gray >>= 1;
    }
	return binary;
}

