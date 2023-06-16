#ifndef INC_NAVIGATION_H_
#define INC_NAVIGATION_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "motor_driver.h"
#include "encoder_driver.h"

typedef struct PID_drv{
	int32_t last_error; // for derivative control
	int32_t error_sum;
	int32_t setpoint;

	float k_p;
	float k_i;
	float k_d;

	int16_t tol;

} PID_drv_t;


typedef struct world_drv{
	double			x_tot_pos;
	double          y_tot_pos;
	double			theta;

	int16_t			r_max;
	int16_t			r_min;

	double			x_home;
	double         y_home;

	float			ticksPerDeg;
	float           ticksPerinch;

} world_drv_t;

typedef struct nav_drv{
	motor_drv_t* 	motor1;
	motor_drv_t* 	motor2;
	encoder_drv_t* 	encoder1;
	encoder_drv_t* 	encoder2;
	PID_drv_t* 		PID1;
	PID_drv_t* 		PID2;

	world_drv_t*	world;
	uint8_t			flag;

} nav_drv_t;

void nav_Rot(nav_drv_t* nav_drv, float deg);

void nav_Lin(nav_drv_t* nav_drv, float inches);

void nav_Update_PID(nav_drv_t* nav_drv);

void nav_Update_Flag(nav_drv_t* nav_drv);

int32_t PID_runController(PID_drv_t* PID_drv, int32_t curr_posn);




#endif /* INC_NAVIGATION_H_ */

