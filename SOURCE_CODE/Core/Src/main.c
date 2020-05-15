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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t StateFlags;
uint8_t TimerFlags;
uint8_t SlowStartEvents;
uint8_t TMR_WarmUp_Timer;
uint8_t TMR_SlowStart_Timer;
uint8_t TMR_StartUp_Timer;
uint8_t TMR_Duty_Timer;
uint8_t TMR_Stall_Timer;
uint8_t TMR_StallCheck_Timer;
uint8_t TMR_Serial_Timer;
uint16_t TMR_Comm_Time;
uint16_t Comm_After_ZC;
uint16_t Expected_ZC;
uint16_t ZC;
int ZCError;
uint8_t PwmDuty;
uint8_t CommState;
uint8_t IsrState;
uint16_t Bit16;
uint32_t Bit32;
uint16_t Temp_Comm_Time;
int Temp;
char buffer[50];
const int CCR_Values[256]={
   0,  4,  8,  12, 16, 20, 24, 27, 31, 35, 39, 43, 47, 51, 55, 59,
   63, 67, 71, 75, 78, 82, 86, 90, 94, 98, 102,106,110,114,118,122,
   125,129,133,137,141,145,149,153,157,161,165,169,173,176,180,184,
   188,192,196,200,204,208,212,216,220,224,227,231,235,239,243,247,
   251,255,259,263,267,271,275,278,282,286,290,294,298,302,306,310,
   314,318,322,325,329,333,337,341,345,349,353,357,361,365,369,373,
   376,380,384,388,392,396,400,404,408,412,416,420,424,427,431,435,
   439,443,447,451,455,459,463,467,471,475,478,482,486,490,494,498,
   502,506,510,514,518,522,525,529,533,537,541,545,549,553,557,561,
   565,569,573,576,580,584,588,592,596,600,604,608,612,616,620,624,
   627,631,635,639,643,647,651,655,659,663,667,671,675,678,682,686,
   690,694,698,702,706,710,714,718,722,725,729,733,737,741,745,749,
   753,757,761,765,769,773,776,780,784,788,792,796,800,804,808,812,
   816,820,824,827,831,835,839,843,847,851,855,859,863,867,871,875,
   878,882,886,890,894,898,902,906,910,914,918,922,925,929,933,937,
   941,945,949,953,957,961,965,969,973,976,980,984,988,992,996,1000
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  StateFlags |= STOP_FLAG;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(StateFlags & STOP_FLAG) InitSystem();
	  TimeBaseManager();
	  WarmUpControl();
	  ControlSlowStart();
	  ControlStartUp();
	  StallControl();
	  SpeedManager();
	  SerialMonitoring();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
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
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1023;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PH_A_Pin|PH_B_Pin|PH_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PH_A_Pin PH_B_Pin PH_C_Pin */
  GPIO_InitStruct.Pin = PH_A_Pin|PH_B_Pin|PH_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void InitSystem(void)
{
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	TMR_WarmUp_Timer = WARMUP_TIME;
	TMR_SlowStart_Timer = SLOWSTART_TIME;
	TMR_Duty_Timer = DUTY_TIME;
	TMR_Stall_Timer = STALL_TIME;
	TMR_StallCheck_Timer = STALL_CHECK_TIME;
	TMR_Serial_Timer = SERIAL_TIME;

	StateFlags = 0x00U;
	TimerFlags = 0x00U;
	SlowStartEvents = SLOW_STEPS;

	Comm_After_ZC = 0;
	Expected_ZC = 0;
	ZC = 0;
	ZCError = 0;
}

void InitDriver(void)
{
	PwmDuty = FindTableIndex(START_PWM_DUTY);
	GetPulseVal(PwmDuty);

	TMR_Comm_Time = TIMER_START_COUNT;
	Expected_ZC = (TMR_Comm_Time>>1) | 0x8000;

	CommState = 1;
	Commutate();

	sprintf(buffer, "BLDC started\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 10);

	StateFlags |= (INIT_COMPLETE_FLAG | STARTUP_IN_PROGRESS);
}

uint8_t FindTableIndex(uint16_t Find_duty)
{
	uint8_t Index;
	for (Index = 0; CCR_Values[Index] < Find_duty; Index++);
	return Index;
}

void TimeBaseManager(void)
{
	HAL_Delay(9);
	TimerFlags |= (TIMER_WARMUP_FLAG | TIMER_SLOWSTART_FLAG);
	TimerFlags |= (TIMER_STARTUP_FLAG | TIMER_DUTY_FLAG);
	TimerFlags |= TIMER_SERIAL_FLAG;
}

void WarmUpControl(void)
{
	if (StateFlags & WARMUP_COMPLETE_FLAG) return;

	if ((~(TimerFlags)) & TIMER_WARMUP_FLAG) return;
	TimerFlags &= ~((uint8_t)TIMER_WARMUP_FLAG);

	if (TMR_WarmUp_Timer)
	{
		TMR_WarmUp_Timer--;
	}
	else
	{
		if (StateFlags & RUN_FLAG)
		{
			StateFlags |= WARMUP_COMPLETE_FLAG;
			InitDriver();
		}
	}
}

void ControlSlowStart(void)
{
	if ((~(StateFlags)) & INIT_COMPLETE_FLAG) return;

	if (StateFlags & SLOWSTART_COMPLETE_FLAG) return;

	if ((~(TimerFlags)) & TIMER_SLOWSTART_FLAG) return;
	TimerFlags &= ~((uint8_t)TIMER_SLOWSTART_FLAG);

	if (--TMR_SlowStart_Timer == 0)
	{
		if (--SlowStartEvents == 0)
		{
			StateFlags |= SLOWSTART_COMPLETE_FLAG;
			TMR_StartUp_Timer = STARTUP_TIME;
			HAL_TIM_Base_Stop(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2, TMR_Comm_Time);
			HAL_TIM_Base_Start(&htim2);
			IsrState = COMMUTATE;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		else
		{
			TMR_SlowStart_Timer = SLOWSTART_TIME;
			Commutate();
		}
	}
}

void ControlStartUp(void)
{
	if ((~(StateFlags)) & SLOWSTART_COMPLETE_FLAG) return;

	if ((~(StateFlags)) & STARTUP_IN_PROGRESS) return;

	if ((~(TimerFlags)) & TIMER_STARTUP_FLAG) return;
	TimerFlags &= ~((uint8_t)TIMER_STARTUP_FLAG);

	if (--TMR_StartUp_Timer == 0)
	{
		StateFlags &= ~((uint8_t)STARTUP_IN_PROGRESS);

		if ((~(StateFlags)) & STARTUP_COMPLETE_FLAG)
		{
			StateFlags |= STOP_FLAG;
		}
	}
}

void StallControl(void)
{
	if ((~(StateFlags)) & INIT_COMPLETE_FLAG) return;

	if ((~(StateFlags)) & STARTUP_COMPLETE_FLAG) return;

	if ((~(TimerFlags)) & TIMER_STALL_FLAG) return;
	TimerFlags &= ~((uint8_t)TIMER_STALL_FLAG);

	if(--TMR_Stall_Timer == 0)
	{
		StateFlags |= STOP_FLAG;
	}

	if (--TMR_StallCheck_Timer == 0)
	{
		TMR_StallCheck_Timer = STALL_CHECK_TIME;

		if(TMR_Comm_Time > MIN_COMMUTATION_COUNT)
		{
			StateFlags |= STOP_FLAG;
		}
	}
}

void SpeedManager(void)
{
	uint16_t AdcVal;

	if((~(TimerFlags)) & TIMER_DUTY_FLAG) return;
	TimerFlags &= ~((uint8_t)TIMER_DUTY_FLAG);

	if(--TMR_Duty_Timer) return;

	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
	{
		AdcVal = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	if (AdcVal < OFF_ADC_VAL)
	{
		if (StateFlags & RUN_FLAG) StateFlags |= STOP_FLAG;
		StateFlags &= ~((uint8_t)RUN_FLAG);
		return;
	}

	if (AdcVal > ON_ADC_VAL) StateFlags |= RUN_FLAG;

	if ((~(StateFlags)) & STARTUP_COMPLETE_FLAG) return;

	if (AdcVal > PwmDuty) PwmDuty++;

	if (AdcVal < PwmDuty) PwmDuty--;

	GetPulseVal(PwmDuty);
	return;
}

void GetPulseVal(uint8_t PulseVal)
{
	htim1.Instance->CCR1 = CCR_Values[PulseVal];
	htim1.Instance->CCR2 = CCR_Values[PulseVal];
	htim1.Instance->CCR3 = CCR_Values[PulseVal];
}

void Commutate(void)//Low Side Modulation
{
	switch (CommState)
	{
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

		StateFlags &= ~((uint8_t)RISING_BEMF_FLAG);

		Bit32 = GPIOB_BASE;
		Bit16 = GPIO_PIN_10;
		break;
	case 2:
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

		StateFlags |= RISING_BEMF_FLAG;

		Bit32 = GPIOA_BASE;
		Bit16 = GPIO_PIN_1;
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

		StateFlags &= ~((uint8_t)RISING_BEMF_FLAG);

		Bit32 = GPIOA_BASE;
		Bit16 = GPIO_PIN_0;
		break;
	case 4:
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

		StateFlags |= RISING_BEMF_FLAG;

		Bit32 = GPIOB_BASE;
		Bit16 = GPIO_PIN_10;
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

		StateFlags &= ~((uint8_t)RISING_BEMF_FLAG);

		Bit32 = GPIOA_BASE;
		Bit16 = GPIO_PIN_1;
		break;
	case 6:
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

		StateFlags |= RISING_BEMF_FLAG;

		Bit32 = GPIOA_BASE;
		Bit16 = GPIO_PIN_0;

		CommState = 0;
		break;
	default:
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		CommState = 1;
		break;
	}
	CommState++;
}

void NextZC(void)
{
	switch (CommState)
	{
	case 1:
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
		break;
	case 2:
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
		break;
	case 3:
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
		break;
	case 4:
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
		break;
	case 5:
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
		break;
	case 6:
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
		break;
	default:
		HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
		HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
		HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);
		break;
	}
}

void SerialMonitoring(void)
{
	if ((~(StateFlags)) & INIT_COMPLETE_FLAG) return;

	if ((~(StateFlags)) & STARTUP_COMPLETE_FLAG) return;

	if ((~(TimerFlags)) & TIMER_SERIAL_FLAG) return;
	TimerFlags &= ~((uint8_t)TIMER_SERIAL_FLAG);

	if(--TMR_Serial_Timer == 0)
	{
		TMR_Serial_Timer = SERIAL_TIME;

		Temp = CCR_Values[PwmDuty] * 100 / PWM_PERIOD;
		sprintf(buffer, "PWM Duty Cycle = %d %%\n", Temp);
		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 10);

		Temp = ((uint16_t)-TMR_Comm_Time) * NUM_PHASES * NUM_POLES;
		Temp = US_PER_SEC * SEC_PER_MIN / Temp;
		sprintf(buffer, "Speed = %d RPM\n", Temp);
		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 10);
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
