/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * @author Miles Alderman
  * @author John Bennett
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "apds9960.h"
#include "servo_driver.h"
#include <stdio.h>
#include "motor_driver.h"
#include "bluetooth_driver.h"
#include "line_driver.h"
#include "encoder_driver.h"
#include "navigation.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t sort_flg = 1;
uint8_t corral_flg = 1;
uint8_t nav_flg = 1;
uint8_t nav_2_flg = 1;

char char_in;
char blue_char;
double PathL;


blue_drv_t blue1 = {'0', '1', &blue_char};

line_drv_t lineR = {'0', GPIOB, RIGHT_LINE_OUT_Pin};
line_drv_t lineL = {'0', GPIOB, LEFT_LINE_OUT_Pin};

motor_drv_t motor1 = {0, TIM_CHANNEL_2, TIM_CHANNEL_1,&htim2};
motor_drv_t motor2 = {0, TIM_CHANNEL_2, TIM_CHANNEL_1,&htim1};
motor_drv_t motor3 = {-1000, TIM_CHANNEL_2, TIM_CHANNEL_1,&htim3};

encoder_drv_t encoder1 = init_encoder(M1_OUTA_Pin, GPIOA, M1_OUTB_Pin, GPIOA, 64*50);
encoder_drv_t encoder2 = init_encoder(M2_OUTA_Pin, GPIOB, M2_OUTB_Pin, GPIOB, 64*50);

// Start Color Sensor
APDS9960 RGB_SORT(&hi2c1, &huart6, 250);

// Servo Object
Servo SERVO_SORT(&htim4, TIM_CHANNEL_4);
Servo SERVO_CORRAL(&htim4, TIM_CHANNEL_3);

// PID
PID_drv_t PID1 = {0, 0, 0, 20, 0, 0, 17};
PID_drv_t PID2 = {0, 0, 0, 20, 0, 0, 17};

// Build World
double y_start = 36;
world_drv_t world = {0,-y_start,90,5, (int16_t)y_start, 0, -y_start,37000/720,6400/11};

//Navigation
nav_drv_t nav = {&motor1,&motor2,&encoder1,&encoder2,&PID1,&PID2,&world,0};




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
void usrprint(const char* message);
void usrprint(uint32_t value);

/**
  * @brief The mastermind task. Mastermind manages other tasks
  */
void MASTERMIND_TASK () {
	if (nav_flg == 5 && nav.flag == 0) {
		corral_flg = 2;
	}
	if (corral_flg == 2 && SERVO_CORRAL.flag == 0){
		nav_flg = 6;
	}
	if (nav_flg == 5 && nav.flag == 0) {
		corral_flg = 1;
	}
}

/**
  * @brief Navigation task controls the desired position of robot
  */
void NAV_TASK(){
	if(blue1.status == '1'){
		nav_Update_PID(&nav);
		if(nav_flg == 1 && nav.flag == 0){
			nav_Lin(&nav,6*3);
			nav_flg++;
		} else if(nav_flg == 2 && nav.flag == 0){
			nav_Rot(&nav,-60);
			nav_flg++;
			PathL = -world.y_tot_pos;
		} else if(nav_flg == 3){
			if(nav_2_flg == 12 && nav.flag == 0){
				nav_Rot(&nav,120);
				nav_flg++;
				nav_2_flg = 1;
			} else if(nav_2_flg%2 == 1){
				if(nav.flag == 0){
					nav_Lin(&nav,PathL);
					nav_2_flg++;
				}
			} else if(nav_2_flg%2 == 0){
				if(nav.flag == 0){
					nav_Rot(&nav,60);
					nav_2_flg++;
				}
			}
		} else if(nav_flg == 4){
			if(nav.flag == 0){
				nav_Lin(&nav,-.5);
				nav_2_flg++;
			}
			if(lineR.state == '1' || lineL.state == '1'){
				nav_flg++;
			}
		} else if(nav_flg == 5){
			if(nav.flag == 0){
				//waiting for Corral 2 lift
			}
		}  else if(nav_flg == 6){
			if(nav.flag == 0){
				nav_Lin(&nav,.5*(nav_2_flg-2));
				nav_flg = 2;
				nav_2_flg = 1;
			}
		}
	}
}

/**
  * @brief Tasks for the Corral module.
  * @param SERVO_obj Servo object associated with the corral.
  */
void CORRAL_TASK(Servo& SERVO_obj) {
	if(blue1.status == '1'){
		// 45deg => 0deg
		if (corral_flg == 1 && SERVO_obj.flag == 0) {
			SERVO_obj.setAngle(0,2000);
			corral_flg = 0;
		// 0deg => 45deg
		} else if (corral_flg == 2 && SERVO_obj.flag == 0) {
			SERVO_obj.setAngle(45,2000);
			corral_flg = 0;
		}
	}

}

/**
  * @brief Tasks for the Motor module.
  * @param motor1 First motor object.
  * @param motor2 Second motor object.
  * @param motor3 Third motor object.
  */
void MOTOR_TASK(motor_drv_t* motor1, motor_drv_t* motor2, motor_drv_t* motor3) {
	  setPWM(motor1);
	  setPWM(motor2);
	  setPWM(motor3);
}

/**
  * @brief Sorts colelcted balls
  * @param RGB_obj RGB sensor object.
  * @param SERVO_obj Servo object associated with the sort module.
  */
void SORT_TASK(APDS9960& RGB_obj, Servo& SERVO_obj) {
	if(blue1.status == '1'){
		if (sort_flg != 0) {
			RGB_obj.readRGBC();
		}

		if (sort_flg == 1) {
			//State 1: Sensing
			if (RGB_obj.ballDetect()) {
				sort_flg = 2;
				//usrprint("Ball Detected");
			}
			//RGB_obj.printRGBCBuffer();

		} else if (sort_flg == 2){
			//State 2: Processing

			// Determine if ball should be kept
			if (RGB_obj.colorSort()) { //if ball is our color
				SERVO_obj.setAngle(0,5000); // coral
				//usrprint("Target Ball Acquired! Storing...");
			} else {
				SERVO_obj.setAngle(180,5000); // reject
				//usrprint("Incorrect ball... rejecting");
			}

			sort_flg = 3;

		} else if (sort_flg == 3) {
			//State 3: Sort Movement
			if (SERVO_obj.flag == 0){
				//usrprint("Position Reset");
				SERVO_obj.setAngle(90,5000); // 90 degrees
				sort_flg = 4;
			}

		} else if (sort_flg == 4) {
			//State 4: resetting
			if (SERVO_obj.flag == 0){
				sort_flg = 1;
				//usrprint("Ready for new ball!");
			}
		}
	}

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

 /**
  * @brief The application entry point.
  * @retval int Return code
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  //////// INTITIALIZATION //////////////////////////////
  SERVO_SORT.initialize();
  SERVO_SORT.setAngle(90,2000);

  SERVO_CORRAL.max_rot = 270;
  SERVO_CORRAL.initialize();
  SERVO_CORRAL.setAngle(45,2000); //start in upright position TODO: move to mastermind at some point


  // Start UART
  HAL_UART_Receive_IT(&huart1,(uint8_t*) &char_in, 1);
  HAL_UART_Receive_IT(&huart6,(uint8_t*) &blue_char, 1);
  // Start timer
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim9);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MASTERMIND_TASK();
	  CORRAL_TASK(SERVO_CORRAL);
	  SORT_TASK(RGB_SORT, SERVO_SORT);
	  NAV_TASK();
	  MOTOR_TASK(&motor1,&motor2,&motor3);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4800-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4800-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4800-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 96-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 2-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 48000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 2000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 48000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : M1_OUTA_Pin M1_OUTB_Pin */
  GPIO_InitStruct.Pin = M1_OUTA_Pin|M1_OUTB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_LINE_OUT_Pin RIGHT_LINE_OUT_Pin M2_OUTB_Pin M2_OUTA_Pin */
  GPIO_InitStruct.Pin = LEFT_LINE_OUT_Pin|RIGHT_LINE_OUT_Pin|M2_OUTB_Pin|M2_OUTA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Transmit a string message via UART interface.
 * 
 * This function appends a line terminator ("\r\n") to the string 
 * and sends it via the UART interface.
 *
 * @param message Null-terminated string to transmit.
 */
void usrprint(const char* message)
{
    // Create a buffer for the complete message with \r\n
    char* completeMessage = new char[strlen(message) + 3];
    strcpy(completeMessage, message);
    strcat(completeMessage, "\r\n");
    HAL_UART_Transmit(&huart6, reinterpret_cast<uint8_t*>(const_cast<char*>(completeMessage)), strlen(completeMessage), HAL_MAX_DELAY);

    delete[] completeMessage; // Release the dynamically allocated memory
}

/**
 * @brief Transmit a uint32_t value via UART interface.
 *
 * This function converts the uint32_t value to string and sends 
 * it via the UART interface.
 *
 * @param value The uint32_t value to transmit.
 */
void usrprint(uint32_t value)
{
    char stringValue[30];
    sprintf(stringValue, "%lu", value); // Convert uint32_t to string
    // Print the value
    usrprint(stringValue);
}

/**
 * @brief Transmit a formatted string via UART interface.
 *
 * This function formats a string using a provided format and a uint32_t value. 
 * The formatted string is then sent via the UART interface.
 *
 * @param message A format string for sprintf.
 * @param value The uint32_t value to be used with the format string.
 */
void usrprint(const char* message,uint32_t value)
{
    char stringValue[30];
    sprintf(stringValue, message, value); // Convert uint32_t to string
    // Print the value
    usrprint(stringValue);
}

/**
 * @brief Rx Transfer completed callback.
 *
 * This function is used to recreate a uart transfer interrupt for both uarts: huart1 and huart6
 * Check which version of the UART triggered this callback
 *
 * @param huart UART handle.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	// Check which version of the UART triggered this callback
	if(huart == &huart1){
		HAL_UART_Transmit(&huart1,(uint8_t*) &char_in, 1,1000);
		HAL_UART_Receive_IT(huart,(uint8_t*) &char_in, 1);

	}
	if(huart == &huart6){
		HAL_UART_Receive_IT(huart,(uint8_t*) &blue_char, 1);
	}
}

/**
 * @brief Period elapsed callback in non-blocking mode. Callback: timer has rolled over
 *
 * This function is used to keep track of when timers overflow.
 * We have two timers that cause interrupt triggers a second and a millisecond timer
 * This timer is used for printing not as quickly updating operation states and flags. As well as keeping track of the state of the deadman switch.
 *
 *
 * @param htim TIM handle.
 */
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  // Check which version of the timer triggered this callback
  if (htim == &htim9 ){
	  SERVO_CORRAL.update_servo_flag();
	  SERVO_SORT.update_servo_flag();
	  Update_Encoder_State(&encoder1);
	  Update_Encoder_State(&encoder2);
	  nav_Update_Flag(&nav);
  }
  else if (htim == &htim10 ){
  	  updateStatus(&blue1);
  	  // May need to move if stantement as function into master mind task
  	  if(blue1.status == '0' && blue1.cur_state == '1'){
  		  disable(&motor1);
  		  disable(&motor2);
  		  disable(&motor3);
  		  blue1.cur_state = '0';
  	  } else if(blue1.status == '1' && blue1.cur_state == '0'){
  		  enable(&motor1);
  		  enable(&motor2);
  		  enable(&motor3);
  		  blue1.cur_state = '1';
  	  }
	  usrprint("PID1.error: %lu",PID2.last_error);
	  usrprint("PID1.error: %lu",-PID2.last_error);
	  usrprint("nav_flg: %lu",nav_flg);
	  //usrprint("encoder1.pos: %lu",encoder1.TOTAL_COUNT);

    }
}

/**
 * @brief EXTI line detection callback.
 *
 * Used for updating line followers and updating encoders on every rising and falling edge.
 *
 * @param GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 */

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if (GPIO_Pin == RIGHT_LINE_OUT_Pin){
		update_Line(&lineR);
		usrprint("RightL.State: %lu",lineR.state);
	}
	if (GPIO_Pin == LEFT_LINE_OUT_Pin){
		update_Line(&lineL);
		usrprint("LeftL.State: %lu",lineL.state);
	}
	if (GPIO_Pin == M1_OUTA_Pin || GPIO_Pin == M1_OUTB_Pin){
		update_encoder(&encoder1);
	}
	if (GPIO_Pin == M2_OUTA_Pin || GPIO_Pin == M2_OUTB_Pin){
		update_encoder(&encoder2);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
