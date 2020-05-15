/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include "stdio.h"
#include "string.h"
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
void InitSystem(void);
void InitDriver(void);
uint8_t FindTableIndex(uint16_t Find_duty);
void TimeBaseManager(void);
void WarmUpControl(void);
void ControlSlowStart(void);
void ControlStartUp(void);
void StallControl(void);
void Commutate(void);
void NextZC(void);
void SpeedManager(void);
void GetPulseVal(uint8_t PulseVal);
void SerialMonitoring(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ZPH_A_Pin GPIO_PIN_0
#define ZPH_A_GPIO_Port GPIOA
#define ZPH_B_Pin GPIO_PIN_1
#define ZPH_B_GPIO_Port GPIOA
#define PA7_ANALOG_IN_Pin GPIO_PIN_4
#define PA7_ANALOG_IN_GPIO_Port GPIOA
#define PH_A_Pin GPIO_PIN_0
#define PH_A_GPIO_Port GPIOB
#define PH_B_Pin GPIO_PIN_1
#define PH_B_GPIO_Port GPIOB
#define PH_C_Pin GPIO_PIN_2
#define PH_C_GPIO_Port GPIOB
#define ZPH_C_Pin GPIO_PIN_10
#define ZPH_C_GPIO_Port GPIOB
#define PL_A_Pin GPIO_PIN_8
#define PL_A_GPIO_Port GPIOA
#define PL_B_Pin GPIO_PIN_9
#define PL_B_GPIO_Port GPIOA
#define PL_C_Pin GPIO_PIN_10
#define PL_C_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
/*
--> 'StateFlags' variable
____________________________________________________________________________
bit |        7        |        6        |        5        |        4        |
____|_________________|_________________|_________________|_________________|
    |  Rising_BEMF_FL |     Stop_FL     |      Run_FL     | StartUp_CMPL_FL |
____|_________________|_________________|_________________|_________________|
____________________________________________________________________________
bit |        3        |        2        |        1        |        0        |
____|_________________|_________________|_________________|_________________|
    |StartUpInProgress|SlowStart_CMPL_FL| WarmUp_CMPLT_FL |   Init_CMPL_FL  |
____|_________________|_________________|_________________|_________________|
*/
#define INIT_COMPLETE_FLAG 				0x01U
#define WARMUP_COMPLETE_FLAG 			0x02U
#define SLOWSTART_COMPLETE_FLAG	 		0x04U
#define STARTUP_IN_PROGRESS 			0x08U
#define STARTUP_COMPLETE_FLAG 			0x10U
#define RUN_FLAG	 					0x20U
#define STOP_FLAG 						0x40U
#define RISING_BEMF_FLAG 				0x80U
/*
--> 'TimerFlags' variable
____________________________________________________________________________
bit |        7        |        6        |        5        |        4        |
____|_________________|_________________|_________________|_________________|
    |                 |                 |  TMR_Serial_FL  |   TMR_Duty_FL   |
____|_________________|_________________|_________________|_________________|
____________________________________________________________________________
bit |        3        |        2        |        1        |        0        |
____|_________________|_________________|_________________|_________________|
    |   TMR_Stall_FL  |  TMR_Startup_FL | TMR_SlowStart_FL|  TMR_WarmUp_FL  |
____|_________________|_________________|_________________|_________________|
*/
#define TIMER_WARMUP_FLAG 				0x01U
#define TIMER_SLOWSTART_FLAG 			0x02U
#define TIMER_STARTUP_FLAG 				0x04U
#define TIMER_STALL_FLAG 				0x08U
#define TIMER_DUTY_FLAG 				0x10U
#define TIMER_SERIAL_FLAG				0x20U

#define WARMUP_TIME						40U
#define SLOWSTART_TIME					20U
#define STARTUP_TIME					200U
#define DUTY_TIME						1U
#define STALL_TIME						100U
#define STALL_CHECK_TIME				50U
#define SERIAL_TIME						100U
#define SLOW_STEPS						1U

#define PWM_PERIOD						1023U
#define START_PWM_DUTY					PWM_PERIOD * 13U / 100U

#define TIMER_AUTO_RELOAD				65535U
#define START_COMMUTATION_TIME			1200U
#define TIMER_START_COUNT				TIMER_AUTO_RELOAD - START_COMMUTATION_TIME + 1U

#define BEMF_DETECTION					101U
#define COMMUTATE 						102U

#define MIN_COMMUTATION_TIME			90U
#define MIN_COMMUTATION_COUNT			65535U - MIN_COMMUTATION_TIME
#define BLANK_PERIOD					10U
#define ERROR_FACTOR					4U

#define ADC_MAX_RESOLUTION				255U
#define OFF_ADC_VAL						ADC_MAX_RESOLUTION * 15U / 100U
#define ON_ADC_VAL						ADC_MAX_RESOLUTION * 25U / 100U

#define US_PER_SEC						1000000U
#define SEC_PER_MIN						60U
#define NUM_POLES						14U
#define NUM_PHASES						3U
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
