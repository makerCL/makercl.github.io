/*
 * bluetooth_driver.h
 *
 *  Created on: May 23, 2023
 *      Author: johna
 */

#ifndef INC_BLUETOOTH_DRIVER_H_
#define INC_BLUETOOTH_DRIVER_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef struct blue_drv {
	char            status;
	char			cur_state;
	char*			blue_char_ptr;
} blue_drv_t;

void updateStatus(blue_drv_t* blue_drv);

void print_Blue(blue_drv_t* blue_drv, UART_HandleTypeDef* huart);

#endif /* INC_BLUETOOTH_DRIVER_H_ */
