/*
 * line_driver.h
 *
 *  Created on: May 31, 2023
 *      Author: johna
 */

#ifndef INC_LINE_DRIVER_H_
#define INC_LINE_DRIVER_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef struct line_drv {
	char				state;
	GPIO_TypeDef*		GPIOx;
	uint16_t			GPIO_Pin;
} line_drv_t;

void update_Line(line_drv_t* line_drv);

void print_LineF(line_drv_t* line_drv, UART_HandleTypeDef* huart);

#endif /* INC_LINE_DRIVER_H_ */
