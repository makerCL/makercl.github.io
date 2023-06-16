/*
 * motor_driver.h
 *
 *  Created on: May 23, 2023
 *      Author: johna
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef struct motor_drv {
	int32_t             pulse;
	uint32_t			chan_1;
	uint32_t			chan_2;
	TIM_HandleTypeDef*	htim;
} motor_drv_t;

void enable(motor_drv_t* motor_drv);

void disable(motor_drv_t* motor_drv);

void scaleNewPulse(motor_drv_t* motor_drv, int8_t val_8);

void NewPulse(motor_drv_t* motor_drv, int32_t val_32);

void setPWM(motor_drv_t* motor_drv);



#endif /* INC_MOTOR_DRIVER_H_ */
