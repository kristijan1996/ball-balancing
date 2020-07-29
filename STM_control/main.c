/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// val is horizontal pixel value of the center of the ball, received from PC.
// Since frame is 640 pixels wide, center is somewhere around 320th pixel.
uint16_t val = 322;
// Flag used to indicate that 10ms has elapsed from timer6 ISR
uint8_t FLAG_10ms = 0;
// 1/10ms is sampling freq of motor control
float dt = 0.01;

/* Function used to set value in CCR1 register of timer3. This value determines
 * the time duration of logical 1 on 50Hz PWM pulse generated by timer3.
 * Clock is prescaled in such a way that for timer to count 20ms it needs to
 * count to 2000. This mapping is natural: 2000-20ms, and allowes duty cycle
 * in range 0.5ms - 2.5ms, which motor receives in order to keep shaft in range
 * 0deg - 180deg, to be interpreted as range of digital values 50 - 250, which
 * are written in CCR1 register.
 *
 * Arguments
 * ---------
 * htim - timer instance
 * val - value in range 50-250 to be written in CCR1 register
 *
 * Returns
 * -------
 * void
 * */
void setPulse(TIM_HandleTypeDef* htim, uint32_t val){
	htim->Instance->CCR1 = val;
}

/* Aux function which sends value (maximum is 999) via UART,
 * digit by digit separating it and putting it into buffer.
 * Not used currently, left for whatever reason.
 *
 * Arguments
 * ----------
 * val - value to be sent in range [0,999]
 *
 * Returns
 * -------
 * void
 * */
void sendValue(uint16_t val){
	uint8_t buff[5];
	buff[0] = val/100 + '0';
	buff[1] = (val/10)%10 + '0';
	buff[2] = val%10 + '0';
	buff[3] = '\r';
	buff[4] = '\n';

	HAL_UART_Transmit(&huart2, buff, 5, 1000);
}

/* Function which reads and interprets incoming value over UART.
 * PC sends horizontal pixel coordinate of the center of the ball.
 * For this value to be interpreted uniformly independent of number
 * of digits it consists of, 'A's needed to be prepended to values
 * which don't have three digits, because leading 0 is interpreted
 * as end of string.
 *
 * Arguments
 * ---------
 * val - pointer to global variable in which center of the ball is stored
 *
 * Returns
 * -------
 * void
 *  */
void receiveValue(uint16_t* val){
	uint8_t buff[5];
	uint16_t value = 0;

	if (HAL_UART_Receive(&huart2, buff, 5, 1000) == HAL_OK){
		for (int i = 2; i >= 0; i--){
			if (buff[i] != 'A'){
				value += (buff[i] - '0')*pow(10, 2-i);
			}
			else
				break;
		}

		*val = value;
	}
}

/* Overloaded function to be executed after timer3 finishes its counting
 * and reaches the falling edge of PWM it generates. This happens when
 * counter reaches value in CCR1 register. Concretely, it turns off LED. */
extern void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim){
	if (htim == &htim3){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	}
}

/* Overloaded function to be executed at he end of timer's period.
 * Depending on timer, either LED is turned on, or flag is being set. */
extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if (htim == &htim3){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}
	else if (htim == &htim6){
		FLAG_10ms = 1;
	}
}



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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Initial values used in PID regulation
  float PID_p = 0, PID_i = 0, PID_d = 0, PID = 0;
  float dist_prev_error = 0, step = 0;
  int16_t dist_error = 0, derror = 0;
  // Pulse value which controls shaft position, in range [50, 250] 170 being horizontal
  uint16_t pulse = 170;
  // Maximum deviation from 170 position is restricted to 170 +- maxStep
  uint16_t maxStep = 25;

  //float Kp = 0.07, Ki = 0.5, Kd = 0.03; // DO NOT TOUCH, These work great
  float Kp = 0.05, Ki = 0.4, Kd = 0.03;

  while (1)
  {
	  // Constantly receive values from PC
	  receiveValue(&val);

	  // but only use them every 10ms
	  if (FLAG_10ms){
		  // Calculate error value as difference from the central pixel
		  dist_error = val - 322;
		  // delta_error, needed for differential part of PID
		  derror = dist_error - dist_prev_error;

		  // Calculate P, I and D values
		  PID_p = Kp * dist_error;
		  PID_d = Kd * (derror/dt);
		  PID_i = PID_i + dist_error*dt;
		  // Do not allow integrator to wind up
		  if (PID_i > maxStep)
			  PID_i = maxStep;
		  else if (PID_i < -maxStep)
			  PID_i = -maxStep;

		  // Combine three parts
		  PID = PID_p + Ki * PID_i + PID_d;

		  // This directly translates to motor control variable step,
		  // which is limited in [170 - maxStep, 170 + maxStep] range
		  step = PID;
		  if (step > maxStep)
			  step = maxStep;
		  if (step < -maxStep)
			  step = -maxStep;

		  // Control motor either up or down from the horizontal position
		  pulse = 170 + step;
		  setPulse(&htim3, pulse);

		  // Prepare values for new iteration
		  dist_prev_error = dist_error;
		  FLAG_10ms = 0;
	  }

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 480;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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
  sConfigOC.Pulse = 170;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 480;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/