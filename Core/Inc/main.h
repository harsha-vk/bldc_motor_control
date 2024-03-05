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
#define UsrBtn_Pin GPIO_PIN_13
#define UsrBtn_GPIO_Port GPIOC
#define UsrBtn_EXTI_IRQn EXTI15_10_IRQn
#define CurrC_Pin GPIO_PIN_0
#define CurrC_GPIO_Port GPIOC
#define CurrB_Pin GPIO_PIN_1
#define CurrB_GPIO_Port GPIOC
#define BemfA_Pin GPIO_PIN_3
#define BemfA_GPIO_Port GPIOC
#define CurrA_Pin GPIO_PIN_0
#define CurrA_GPIO_Port GPIOA
#define Vbus_Pin GPIO_PIN_1
#define Vbus_GPIO_Port GPIOA
#define UsrPot_Pin GPIO_PIN_4
#define UsrPot_GPIO_Port GPIOA
#define PwmAL_Pin GPIO_PIN_7
#define PwmAL_GPIO_Port GPIOA
#define PwmBL_Pin GPIO_PIN_0
#define PwmBL_GPIO_Port GPIOB
#define PwmCL_Pin GPIO_PIN_1
#define PwmCL_GPIO_Port GPIOB
#define UsrLED_Pin GPIO_PIN_2
#define UsrLED_GPIO_Port GPIOB
#define HallC_Pin GPIO_PIN_10
#define HallC_GPIO_Port GPIOB
#define HallC_EXTI_IRQn EXTI15_10_IRQn
#define BemfB_Pin GPIO_PIN_11
#define BemfB_GPIO_Port GPIOB
#define BemfC_Pin GPIO_PIN_13
#define BemfC_GPIO_Port GPIOB
#define Bemf0_Pin GPIO_PIN_9
#define Bemf0_GPIO_Port GPIOC
#define PwmAH_Pin GPIO_PIN_8
#define PwmAH_GPIO_Port GPIOA
#define PwmBH_Pin GPIO_PIN_9
#define PwmBH_GPIO_Port GPIOA
#define PwmCH_Pin GPIO_PIN_10
#define PwmCH_GPIO_Port GPIOA
#define HallA_Pin GPIO_PIN_15
#define HallA_GPIO_Port GPIOA
#define HallA_EXTI_IRQn EXTI15_10_IRQn
#define HallB_Pin GPIO_PIN_3
#define HallB_GPIO_Port GPIOB
#define HallB_EXTI_IRQn EXTI3_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
