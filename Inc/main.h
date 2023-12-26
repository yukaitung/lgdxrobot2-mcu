/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define M4_E_CH1_Pin GPIO_PIN_0
#define M4_E_CH1_GPIO_Port GPIOA
#define M4_E_CH2_Pin GPIO_PIN_1
#define M4_E_CH2_GPIO_Port GPIOA
#define M4_PWM_Pin GPIO_PIN_2
#define M4_PWM_GPIO_Port GPIOA
#define M3_PWM_Pin GPIO_PIN_3
#define M3_PWM_GPIO_Port GPIOA
#define D_STBY_Pin GPIO_PIN_4
#define D_STBY_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_5
#define M2_PWM_GPIO_Port GPIOA
#define M2_E_CH1_Pin GPIO_PIN_6
#define M2_E_CH1_GPIO_Port GPIOA
#define M2_E_CH2_Pin GPIO_PIN_7
#define M2_E_CH2_GPIO_Port GPIOA
#define L_GREEN_Pin GPIO_PIN_1
#define L_GREEN_GPIO_Port GPIOB
#define L_RED_Pin GPIO_PIN_2
#define L_RED_GPIO_Port GPIOB
#define BT1_SW_Pin GPIO_PIN_10
#define BT1_SW_GPIO_Port GPIOB
#define M1_IN2_Pin GPIO_PIN_12
#define M1_IN2_GPIO_Port GPIOB
#define M1_IN1_Pin GPIO_PIN_13
#define M1_IN1_GPIO_Port GPIOB
#define M2_IN1_Pin GPIO_PIN_14
#define M2_IN1_GPIO_Port GPIOB
#define M2_IN2_Pin GPIO_PIN_15
#define M2_IN2_GPIO_Port GPIOB
#define M1_E_CH1_Pin GPIO_PIN_8
#define M1_E_CH1_GPIO_Port GPIOA
#define M1_E_CH2_Pin GPIO_PIN_9
#define M1_E_CH2_GPIO_Port GPIOA
#define M3_IN2_Pin GPIO_PIN_10
#define M3_IN2_GPIO_Port GPIOA
#define M3_IN1_Pin GPIO_PIN_15
#define M3_IN1_GPIO_Port GPIOA
#define M1_PWM_Pin GPIO_PIN_3
#define M1_PWM_GPIO_Port GPIOB
#define M4_IN1_Pin GPIO_PIN_4
#define M4_IN1_GPIO_Port GPIOB
#define M4_IN2_Pin GPIO_PIN_5
#define M4_IN2_GPIO_Port GPIOB
#define M3_E_CH1_Pin GPIO_PIN_6
#define M3_E_CH1_GPIO_Port GPIOB
#define M3_E_CH2_Pin GPIO_PIN_7
#define M3_E_CH2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
