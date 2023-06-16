/*
 * navigation.cpp
 *
 *  Created on: Jun 13, 2023
 *      Author: John Bennett
 */



#include "navigation.h"
#include <cmath>

	/**
	* @brief Rotates robot a set number of degrees counter clockwise.
	*
	* @param nav_drv navigation driver structure.
	* @param deg specified degrees to turn.
	*
	*/
void nav_Rot(nav_drv_t* nav_drv, float deg){
	if(nav_drv->flag == 0){
		zero(nav_drv->encoder1);
		zero(nav_drv->encoder2);

		nav_drv->PID1->setpoint = -deg*nav_drv->world->ticksPerDeg;
		nav_drv->PID2->setpoint =  deg*nav_drv->world->ticksPerDeg;
		nav_drv->PID1->last_error = 100;
		nav_drv->PID2->last_error = 100;

		nav_drv->flag = 1;

		nav_drv->world->theta += deg;
	}
}

	/**
	* @brief Moves robot a set number of inches forward.
	*
	* @param nav_drv navigation driver structure.
	* @param inches to move forward.
	*
	*/
void nav_Lin(nav_drv_t* nav_drv, float inches){
	if(nav_drv->flag == 0){
		zero(nav_drv->encoder1);
		zero(nav_drv->encoder2);

		nav_drv->PID1->setpoint =  inches*nav_drv->world->ticksPerinch;
		nav_drv->PID2->setpoint =  inches*nav_drv->world->ticksPerinch;
		nav_drv->PID1->last_error = 100;
		nav_drv->PID2->last_error = 100;

		nav_drv->flag = 1;

		nav_drv->world->x_tot_pos += inches*cos(nav_drv->world->theta * M_PI / 180.0);
		nav_drv->world->y_tot_pos += inches*sin(nav_drv->world->theta * M_PI / 180.0);
	}
}

	/**
	* @brief Updates new PWM Signals to motors specified by PID controller.
	*
	* @param nav_drv navigation driver structure.
	*
	*/
void nav_Update_PID(nav_drv_t* nav_drv){
	NewPulse(nav_drv->motor1,PID_runController(nav_drv->PID1,nav_drv->encoder1->TOTAL_COUNT));
	NewPulse(nav_drv->motor2,PID_runController(nav_drv->PID2,nav_drv->encoder2->TOTAL_COUNT));
}

	/**
	* @brief Checks if PID Control is complete under specified tolerance and updates flag.
	*
	* @param nav_drv navigation driver structure.
	*
	*/
void nav_Update_Flag(nav_drv_t* nav_drv){
	if((nav_drv->flag != 0) &&
	   (nav_drv->PID1->last_error <= nav_drv->PID1->tol && -nav_drv->PID1->last_error <= nav_drv->PID1->tol) &&
	   (nav_drv->PID2->last_error <= nav_drv->PID2->tol && -nav_drv->PID2->last_error <= nav_drv->PID2->tol)){
		nav_drv->flag = 0;
	}
}

	/**
	* @brief Runs PID Controller to calculate output for new PWM control.
	*
	* @param PID_drv PID driver structure.
	* @return int32_t calculated 32 bit PWM Value.
	*
	*/
int32_t PID_runController(PID_drv_t* PID_drv, int32_t curr_posn){
	int32_t error = PID_drv->setpoint - curr_posn;
	PID_drv->error_sum = PID_drv->error_sum + error;

	int32_t PWM = PID_drv->k_p * error + PID_drv->k_i * PID_drv->error_sum + PID_drv->k_d * (error - PID_drv->last_error);
	PID_drv->last_error = error;

	return PWM;
}
