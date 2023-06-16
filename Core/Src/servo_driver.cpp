#include "servo_driver.h"

/**
 * @brief Servo constructor
 * @author Miles Alderman
 * @param PWMHandle Handler for PWM 
 * @param channel Channel for the timer
 */
Servo::Servo(TIM_HandleTypeDef* PWMHandle, uint32_t channel) : htim(PWMHandle), timer_channel(channel){
	flag = 0;
}

/**
 * @brief Initialize the Servo object and starts PWM
 * 
 */
void Servo::initialize(){
	HAL_TIM_PWM_Start(htim, timer_channel);
	// Set the initial position of the servo to 90 degrees
}

/**
 * @brief Set the position of the Servo object
 * 
 * @param position_deg Desired position in degrees
 * @param delay Delay in milliseconds before the servo starts moving
 */
void Servo::setAngle(uint32_t position_deg, uint16_t delay) {
	  // Ensure the input is within the valid range
	  if (position_deg > max_rot) {
		  position_deg = max_rot;
	  }

	  // Compute pulse width based on desired position
	  float pulse_width_ms = min_pulse + (max_pulse - min_pulse) * position_deg / max_rot;

	  // Compute compare register value
	  uint32_t compare_value = static_cast<uint32_t> ((96000000/ (htim->Init.Prescaler + 1)) * pulse_width_ms / 1000);

	  __HAL_TIM_SET_COMPARE(htim, timer_channel, compare_value);
	  startTimer(delay);
}

/**
 * @brief Update the servo flag
 * 
 * Decrements the flag if it's greater than 0.
 */
void Servo::update_servo_flag(){
	if(flag>0){
		flag--;
	}
}

/**
 * @brief Starts a timer
 * 
 * @param delay Delay in milliseconds for the timer
 */
void Servo::startTimer(uint16_t delay) {
	flag = delay;
}

/*
float Servo::elapsedTime(){
	uint32_t end_time = __HAL_TIM_GetCounter(hSW);
	float elapsed_time = (end_time - start_time)/1000;
	return elapsed_time; //in ms
}*/
