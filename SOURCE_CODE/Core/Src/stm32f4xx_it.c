/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern uint8_t StateFlags;
extern uint8_t IsrState;
extern uint8_t TMR_Stall_Timer;
extern uint16_t TMR_Comm_Time;
extern uint16_t Comm_After_ZC;
extern uint16_t Expected_ZC;
extern uint16_t ZC;
extern int ZCError;
extern uint16_t Bit16;
extern uint32_t Bit32;
extern uint16_t Temp_Comm_Time;
extern int Temp;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */
	switch (IsrState)
	{
	case BEMF_DETECTION:
		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC3);
		if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) || __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) || __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3))
		{
			ZC = __HAL_TIM_GET_COUNTER(&htim2);
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
			if (StateFlags & STARTUP_COMPLETE_FLAG)
			{
				HAL_TIM_Base_Stop(&htim2);
				__HAL_TIM_SET_COUNTER(&htim2, Comm_After_ZC);
				HAL_TIM_Base_Start(&htim2);
			}

			IsrState = COMMUTATE;
			break;
		}

		if (StateFlags & STARTUP_COMPLETE_FLAG)
		{
			TMR_Comm_Time += (Expected_ZC>>2);
		}

	case COMMUTATE:
		if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE))
		{
			htim2.Instance->SR &= ~TIM_IT_UPDATE;// Clear Interrupt
			Commutate();

			if ((~(StateFlags)) & RISING_BEMF_FLAG)
			{
				Temp_Comm_Time = TMR_Comm_Time + BLANK_PERIOD;

				while(__HAL_TIM_GET_COUNTER(&htim2) < BLANK_PERIOD);

				HAL_TIM_Base_Stop(&htim2);
				__HAL_TIM_SET_COUNTER(&htim2, Temp_Comm_Time+1);
				HAL_TIM_Base_Start(&htim2);

				while (HAL_GPIO_ReadPin((GPIO_TypeDef *)Bit32, Bit16) != GPIO_PIN_RESET)
				{
					if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) break;
				}

				if (!(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)))
				{
					htim2.Instance->SR &= ~(TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3);// Clear Interrupt
					NextZC();
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
					IsrState = BEMF_DETECTION;
				}
			}
			else
			{
				Expected_ZC = (TMR_Comm_Time>>1) | 0x8000;
				ZCError = ZC - Expected_ZC;
				Temp = ZCError;

				if (Temp & 0x8000) Temp = ~Temp + 1;// Absolute Value

				if (Temp < ((uint16_t)-Expected_ZC>>1))
				{
					StateFlags |= STARTUP_COMPLETE_FLAG;
					TMR_Stall_Timer = STALL_TIME;
				}

				TMR_Comm_Time -= (ZCError>>ERROR_FACTOR);
				Temp_Comm_Time = TMR_Comm_Time;
				HAL_TIM_Base_Stop(&htim2);
				__HAL_TIM_SET_COUNTER(&htim2, Temp_Comm_Time+1);
				HAL_TIM_Base_Start(&htim2);
				IsrState = COMMUTATE;
				Comm_After_ZC = Expected_ZC;
			}
		}
		break;
	default:
		StateFlags |= STOP_FLAG;
		break;
	}
  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
