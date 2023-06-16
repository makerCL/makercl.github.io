/*
 * motor_drive.c
 *
 *  Created on: May 23, 2023
 *      Author: John Bennett
 */

#include "motor_driver.h"

	/**
	* @brief Enables motor timer for a specified channel.
	*
	* @param motor_drv motor driver structure.
	*/
void enable(motor_drv_t* motor_drv){
	HAL_TIM_PWM_Start(motor_drv->htim,motor_drv->chan_1);
	HAL_TIM_PWM_Start(motor_drv->htim,motor_drv->chan_2);
}
	/**
	* @brief Disables motor timer for a specified channel.
	*
	* @param motor_drv motor driver structure.
	*/
void disable(motor_drv_t* motor_drv){
	HAL_TIM_PWM_Stop(motor_drv->htim,motor_drv->chan_1);
	HAL_TIM_PWM_Stop(motor_drv->htim,motor_drv->chan_2);
}

	/**
	* @brief Calculates the new pulse scaled from an int8_t to be a value between the current pulse and the negative of that pulse. This function works best when using hex.
	*
	* @param motor_drv motor driver structure.
	* @param val_8 an 8 bit number that will be scaled to represent the new pulse.
	*/
void scaleNewPulse(motor_drv_t* motor_drv, int8_t val_8){
	motor_drv->pulse = ((int32_t)val_8)*(2*(int32_t)motor_drv->htim->Init.Period + 1)/256;
}

	/**
	* @brief Sets new pulse.
	*
	* @param motor_drv motor driver structure.
	* @param PWM_32 a 32 bit number to be set as the new pulse in motor_drv.
	*/
void NewPulse(motor_drv_t* motor_drv, int32_t PWM_32){
	if(PWM_32 >= (int32_t)motor_drv->htim->Init.Period){
		motor_drv->pulse =  motor_drv->htim->Init.Period/2;
	} else if (-PWM_32 >= (int32_t)motor_drv->htim->Init.Period){
		motor_drv->pulse =  -motor_drv->htim->Init.Period/2;
	} else {
		motor_drv->pulse = PWM_32/2;
	}

}

	/**
	* @brief Sets up the PWM signal for the motors. Uses inverse PWM.
	*
	* @param motor_drv motor driver structure.
	*
	*/
void setPWM(motor_drv_t* motor_drv){
	uint32_t ch1_pulse = 0;
	uint32_t ch2_pulse = 0;
	if(motor_drv->pulse > (int32_t) motor_drv->htim->Init.Period){
		ch1_pulse = motor_drv->htim->Init.Period;
		ch2_pulse = 0;
	}
	else if(-motor_drv->pulse > (int32_t) motor_drv->htim->Init.Period){
		ch1_pulse = 0;
		ch2_pulse = motor_drv->htim->Init.Period;
	}
	else if(motor_drv->pulse >= 0){
		ch1_pulse =  motor_drv->htim->Init.Period;
		ch2_pulse =  motor_drv->htim->Init.Period - (uint32_t)(motor_drv->pulse);
	}
	else if(motor_drv->pulse < 0){
		ch1_pulse =  motor_drv->htim->Init.Period - (uint32_t)(-motor_drv->pulse);
		ch2_pulse =  motor_drv->htim->Init.Period;
	}


	if (motor_drv->pulse >= 0) {
		ch1_pulse = motor_drv->htim->Init.Period;
		ch2_pulse = motor_drv->htim->Init.Period - motor_drv->pulse;
	} else {
		ch1_pulse = motor_drv->htim->Init.Period + motor_drv->pulse;
		ch2_pulse = motor_drv->htim->Init.Period;
	}

	if (ch1_pulse > motor_drv->htim->Init.Period) {
		ch1_pulse = motor_drv->htim->Init.Period;
		ch2_pulse = 0;
	} else if (ch2_pulse > motor_drv->htim->Init.Period) {
		ch1_pulse = 0;
		ch2_pulse = motor_drv->htim->Init.Period;
	}

	__HAL_TIM_SET_COMPARE(motor_drv->htim, motor_drv->chan_1, ch1_pulse);
	__HAL_TIM_SET_COMPARE(motor_drv->htim, motor_drv->chan_2, ch2_pulse);
}

