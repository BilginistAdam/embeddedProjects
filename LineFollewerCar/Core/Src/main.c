/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "L298N.h"
#include "TCRT5000.h"
#include "HCSR04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STOP_DISTANCE	10U
#define MOTOR_SPEED_LINEAR_MAX	630U
#define MOTOR_SPEED_ROUNDED_MAX	680U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//HCSR04 Handler
HCSR_Handlder_t hhcsr;

//TCRT5000 Handler
TCRT_Handler_t htcrt_right;
TCRT_Handler_t htcrt_left;

//L298N Handler
L298N_Handler_t hl298n_right;
L298N_Handler_t hl298n_left;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//Init Func.
static void HCSR04_Init(void);
static void HTCRT_Init(void);
static void HL298N_Init(void);

//Motor Func
void motorForward(void);
void motorBackward(void);
void motorLeft(void);
void motorRight(void);
void motorStop(void);
void motorHandler(int8_t cmd);
//TCRT
int8_t tcrt_handler(void);

//HCSR04 Distance Control
int8_t hcsr_cntrlDistance(void);

int8_t cmd = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HCSR04_Init();
  HTCRT_Init();
  HL298N_Init();

  HAL_Delay(1500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Send Read Command
	  HCSR_Read(&hhcsr);

	  //Calculate distance
	  HCSR_Handler(&hhcsr);
	  cmd = tcrt_handler();
	  //control Distance
	  if(hcsr_cntrlDistance()){
		 motorHandler(cmd);
	  }else{
		  //Motor Stop
		  motorStop();
	  }

	  //Wait echo signal
	  HAL_Delay(100);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(pwrLED_GPIO_Port, pwrLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L298N_1_Pin|L298N_2_Pin|L298N_3_Pin|L298N_4_Pin
                          |HCSR_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : pwrLED_Pin */
  GPIO_InitStruct.Pin = pwrLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pwrLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L298N_1_Pin L298N_2_Pin L298N_3_Pin L298N_4_Pin
                           HCSR_Trig_Pin */
  GPIO_InitStruct.Pin = L298N_1_Pin|L298N_2_Pin|L298N_3_Pin|L298N_4_Pin
                          |HCSR_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TCRT_0_Pin TCRT_1_Pin */
  GPIO_InitStruct.Pin = TCRT_0_Pin|TCRT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
static void HCSR04_Init(void){
	hhcsr.config.htim = &htim1;
	hhcsr.config.channel = TIM_CHANNEL_1;
	hhcsr.config.trigPort = HCSR_Trig_GPIO_Port;
	hhcsr.config.trigPin = HCSR_Trig_Pin;

	if(HAL_OK != HCSR_Init(&hhcsr)){
		Error_Handler();
	}
}

static void HTCRT_Init(void){
	//tcrt right
	htcrt_right.Config.trigPort = TCRT_0_GPIO_Port;
	htcrt_right.Config.trigPin = TCRT_0_Pin;

	if(HAL_OK != TCRT_Init(&htcrt_right)){
		Error_Handler();
	}

	//tcrt left
	htcrt_left.Config.trigPort = TCRT_1_GPIO_Port;
	htcrt_left.Config.trigPin = TCRT_1_Pin;

	if(HAL_OK != TCRT_Init(&htcrt_left)){
		Error_Handler();
	}
}

static void HL298N_Init(void){
	//L298N Right
	hl298n_right.Config.Timer = &htim3;
	hl298n_right.Config.Channel = TIM_CHANNEL_1;
	hl298n_right.Config.port[0] = L298N_1_GPIO_Port;
	hl298n_right.Config.port[1] = L298N_2_GPIO_Port;
	hl298n_right.Config.pin[0] = L298N_1_Pin;
	hl298n_right.Config.pin[1] = L298N_2_Pin;

	if(HAL_OK != L298N_MotorInit(&hl298n_right)){
		Error_Handler();
	}

	//L298N Left
	hl298n_left.Config.Timer = &htim3;
	hl298n_left.Config.Channel = TIM_CHANNEL_2;
	hl298n_left.Config.port[0] = L298N_3_GPIO_Port;
	hl298n_left.Config.port[1] = L298N_4_GPIO_Port;
	hl298n_left.Config.pin[0] = L298N_3_Pin;
	hl298n_left.Config.pin[1] = L298N_4_Pin;

	if(HAL_OK != L298N_MotorInit(&hl298n_left)){
		Error_Handler();
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	HCSR_IC_Handler(&hhcsr, htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//TCRT Right
	TCRT_IT_Handler(&htcrt_right, GPIO_Pin);
	//TCRT Left
	TCRT_IT_Handler(&htcrt_left, GPIO_Pin);
}

//Motor Func
void motorForward(void){
	//set direction to forward
	L298N_setMotorDirection(&hl298n_right, L298N_MOTOR_DIRECTION_FORWARD);
	L298N_setMotorDirection(&hl298n_left, L298N_MOTOR_DIRECTION_FORWARD);

	//set motor Speed
	L298N_setMotorSpeed(&hl298n_right, MOTOR_SPEED_LINEAR_MAX);
	L298N_setMotorSpeed(&hl298n_left, MOTOR_SPEED_LINEAR_MAX);
}

void motorBackward(void){
	//set direction to backward
	L298N_setMotorDirection(&hl298n_right, L298N_MOTOR_DIRECTION_BACKWARD);
	L298N_setMotorDirection(&hl298n_left, L298N_MOTOR_DIRECTION_BACKWARD);

	//set motor Speed
	L298N_setMotorSpeed(&hl298n_right, MOTOR_SPEED_LINEAR_MAX);
	L298N_setMotorSpeed(&hl298n_left, MOTOR_SPEED_LINEAR_MAX);
}

void motorLeft(void){
	//set direction to forward
	L298N_setMotorDirection(&hl298n_right, L298N_MOTOR_DIRECTION_STOP);
	L298N_setMotorDirection(&hl298n_left, L298N_MOTOR_DIRECTION_FORWARD);

	//set motor Speed
	L298N_setMotorSpeed(&hl298n_right, L298N_MOTOR_SPEED_STOP);
	L298N_setMotorSpeed(&hl298n_left, MOTOR_SPEED_ROUNDED_MAX);
}

void motorRight(void){
	//set direction to forward
	L298N_setMotorDirection(&hl298n_right, L298N_MOTOR_DIRECTION_FORWARD);
	L298N_setMotorDirection(&hl298n_left, L298N_MOTOR_DIRECTION_STOP);

	//set motor Speed
	L298N_setMotorSpeed(&hl298n_right, MOTOR_SPEED_ROUNDED_MAX);
	L298N_setMotorSpeed(&hl298n_left, L298N_MOTOR_SPEED_STOP);
}

void motorStop(void){
	//set direction to forward
	L298N_setMotorDirection(&hl298n_right, L298N_MOTOR_DIRECTION_STOP);
	L298N_setMotorDirection(&hl298n_left, L298N_MOTOR_DIRECTION_STOP);

	//set motor Speed
	L298N_setMotorSpeed(&hl298n_right, L298N_MOTOR_SPEED_STOP);
	L298N_setMotorSpeed(&hl298n_left, L298N_MOTOR_SPEED_STOP);
}

void motorHandler(int8_t cmd){
	if(cmd == 0){
		motorStop();
	}else if(cmd == 1){
		motorForward();
	}else if(cmd == 2){
		motorLeft();
	}else if(cmd == 3){
		motorRight();
	}else if(cmd == 4){
		motorBackward();
	}else{
		motorStop();
	}
}

int8_t tcrt_handler(void){
	//0 -> STOP
	//1 -> Forward
	//2 -> Left
	//3 -> Right
	//4 -> Bacward
	TCRT_FlagStatus_t leftStatus = TCRT_getStatus(&htcrt_left);
	TCRT_FlagStatus_t rightStatus = TCRT_getStatus(&htcrt_right);
	if(!(leftStatus || rightStatus)){
		return 1;
	}else if(leftStatus && rightStatus){
		return 0;
	}else if(!rightStatus && leftStatus){
		return 2;
	}else if(rightStatus && !leftStatus){
		return 3;

	}else{
		return 0;
	}
}

int8_t hcsr_cntrlDistance(void){
	//If distance is lower than STOP_DISTANACE (10cm), Motor stop
	if(HCSR_getDistance(&hhcsr) < STOP_DISTANCE){
		return RESET;
	}
	return SET;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
