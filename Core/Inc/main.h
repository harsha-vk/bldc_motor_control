/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define USR_BTN_Pin GPIO_PIN_13
#define USR_BTN_GPIO_Port GPIOC
#define USR_BTN_EXTI_IRQn EXTI15_10_IRQn
#define M1_CURR_FDBK_C_Pin GPIO_PIN_0
#define M1_CURR_FDBK_C_GPIO_Port GPIOC
#define M1_CURR_FDBK_B_Pin GPIO_PIN_1
#define M1_CURR_FDBK_B_GPIO_Port GPIOC
#define M1_BEMF_A_Pin GPIO_PIN_3
#define M1_BEMF_A_GPIO_Port GPIOC
#define M1_CURR_FDBK_A_Pin GPIO_PIN_0
#define M1_CURR_FDBK_A_GPIO_Port GPIOA
#define M1_V_BUS_Pin GPIO_PIN_1
#define M1_V_BUS_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define USR_POT_Pin GPIO_PIN_4
#define USR_POT_GPIO_Port GPIOA
#define M1_PWM_A_L_Pin GPIO_PIN_7
#define M1_PWM_A_L_GPIO_Port GPIOA
#define M1_PWM_B_L_Pin GPIO_PIN_0
#define M1_PWM_B_L_GPIO_Port GPIOB
#define M1_PWM_C_L_Pin GPIO_PIN_1
#define M1_PWM_C_L_GPIO_Port GPIOB
#define USR_LED_Pin GPIO_PIN_2
#define USR_LED_GPIO_Port GPIOB
#define M1_HALL_C_Pin GPIO_PIN_10
#define M1_HALL_C_GPIO_Port GPIOB
#define M1_BEMF_B_Pin GPIO_PIN_11
#define M1_BEMF_B_GPIO_Port GPIOB
#define M1_BEMF_C_Pin GPIO_PIN_13
#define M1_BEMF_C_GPIO_Port GPIOB
#define M1_BEMF_GPIO_Pin GPIO_PIN_9
#define M1_BEMF_GPIO_GPIO_Port GPIOC
#define M1_PWM_A_H_Pin GPIO_PIN_8
#define M1_PWM_A_H_GPIO_Port GPIOA
#define M1_PWM_B_H_Pin GPIO_PIN_9
#define M1_PWM_B_H_GPIO_Port GPIOA
#define M1_PWM_C_H_Pin GPIO_PIN_10
#define M1_PWM_C_H_GPIO_Port GPIOA
#define M1_HALL_A_Pin GPIO_PIN_15
#define M1_HALL_A_GPIO_Port GPIOA
#define M1_HALL_B_Pin GPIO_PIN_3
#define M1_HALL_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
