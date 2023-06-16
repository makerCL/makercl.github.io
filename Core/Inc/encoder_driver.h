/*
 * encoder_driver.h
 *
 *  Created on: Jun 2, 2023
 *      Author: johna
 */

#ifndef INC_ENCODER_DRIVER_H_
#define INC_ENCODER_DRIVER_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef struct encoder_drv {
	int32_t             TOTAL_COUNT;
	int32_t             LAST_COUNT;
	uint8_t				LAST_ENCODER_COUNT;

	uint16_t			GPIO_Pin1;
	GPIO_TypeDef*		GPIOx1;
	uint16_t			GPIO_Pin2;
	GPIO_TypeDef*		GPIOx2;

	int32_t             TPR;
	double				pos;
} encoder_drv_t;

encoder_drv_t init_encoder(uint16_t	GPIO_Pin1, GPIO_TypeDef* GPIOx1, uint16_t GPIO_Pin2, GPIO_TypeDef* GPIOx2, int32_t TPR);

void update_encoder(encoder_drv_t* encoder_drv);

//void print_encoder(encoder_drv_t* encoder_drv);

void Update_Encoder_State(encoder_drv_t* encoder_drv);

void zero(encoder_drv_t* encoder_drv);

uint8_t NewState(encoder_drv_t* encoder_drv);


#endif /* INC_ENCODER_DRIVER_H_ */
