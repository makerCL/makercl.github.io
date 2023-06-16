/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_PWM_IN2_Pin GPIO_PIN_0
#define M1_PWM_IN2_GPIO_Port GPIOA
#define M1_PWM_IN1_Pin GPIO_PIN_1
#define M1_PWM_IN1_GPIO_Port GPIOA
#define M1_OUTA_Pin GPIO_PIN_2
#define M1_OUTA_GPIO_Port GPIOA
#define M1_OUTA_EXTI_IRQn EXTI2_IRQn
#define M1_OUTB_Pin GPIO_PIN_3
#define M1_OUTB_GPIO_Port GPIOA
#define M1_OUTB_EXTI_IRQn EXTI3_IRQn
#define VBAT_ADC_Pin GPIO_PIN_4
#define VBAT_ADC_GPIO_Port GPIOA
#define M3_PWM_IN2_Pin GPIO_PIN_6
#define M3_PWM_IN2_GPIO_Port GPIOA
#define M3_PWM_IN1_Pin GPIO_PIN_7
#define M3_PWM_IN1_GPIO_Port GPIOA
#define LEFT_LINE_OUT_Pin GPIO_PIN_10
#define LEFT_LINE_OUT_GPIO_Port GPIOB
#define LEFT_LINE_OUT_EXTI_IRQn EXTI15_10_IRQn
#define RIGHT_LINE_OUT_Pin GPIO_PIN_12
#define RIGHT_LINE_OUT_GPIO_Port GPIOB
#define RIGHT_LINE_OUT_EXTI_IRQn EXTI15_10_IRQn
#define M2_OUTB_Pin GPIO_PIN_14
#define M2_OUTB_GPIO_Port GPIOB
#define M2_OUTB_EXTI_IRQn EXTI15_10_IRQn
#define M2_OUTA_Pin GPIO_PIN_15
#define M2_OUTA_GPIO_Port GPIOB
#define M2_OUTA_EXTI_IRQn EXTI15_10_IRQn
#define M2_PWM_IN2_Pin GPIO_PIN_8
#define M2_PWM_IN2_GPIO_Port GPIOA
#define M2_PWM_IN1_Pin GPIO_PIN_9
#define M2_PWM_IN1_GPIO_Port GPIOA
#define BLUE_TX_Pin GPIO_PIN_11
#define BLUE_TX_GPIO_Port GPIOA
#define BLUE_RX_Pin GPIO_PIN_12
#define BLUE_RX_GPIO_Port GPIOA
#define EXT_TX_Pin GPIO_PIN_15
#define EXT_TX_GPIO_Port GPIOA
#define EXT_RX_Pin GPIO_PIN_3
#define EXT_RX_GPIO_Port GPIOB
#define SORT_SERVO_PWM_Pin GPIO_PIN_8
#define SORT_SERVO_PWM_GPIO_Port GPIOB
#define GATE_SERVO_PWM_Pin GPIO_PIN_9
#define GATE_SERVO_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
