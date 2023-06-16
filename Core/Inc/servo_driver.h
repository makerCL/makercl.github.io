#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "stm32f4xx_hal.h"


class Servo {
public:

  Servo(TIM_HandleTypeDef* PWMHandle, uint32_t channel);
  void initialize();
  void setAngle(uint32_t position_deg, uint16_t delay);
  void update_servo_flag();

  float min_pulse = 0.5; //ms
  float max_pulse = 2.5; //ms
  uint16_t max_rot = 180; //degrees

  uint16_t flag; // used to indicate if servo is in motion

  void startTimer(uint16_t delay);
  float elapsedTime();

private:
  TIM_HandleTypeDef* htim;
  uint32_t timer_channel;


  //uint32_t start_time;
};


#endif
