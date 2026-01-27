/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define DR2BIN2_Pin GPIO_PIN_13
#define DR2BIN2_GPIO_Port GPIOC
#define DR1AIN2_Pin GPIO_PIN_14
#define DR1AIN2_GPIO_Port GPIOC
#define DR1AIN1_Pin GPIO_PIN_15
#define DR1AIN1_GPIO_Port GPIOC
#define MJ3CH2_Pin GPIO_PIN_0
#define MJ3CH2_GPIO_Port GPIOA
#define MJ3CH1_Pin GPIO_PIN_1
#define MJ3CH1_GPIO_Port GPIOA
#define DR2PWMB_Pin GPIO_PIN_2
#define DR2PWMB_GPIO_Port GPIOA
#define DR1PWMB_Pin GPIO_PIN_3
#define DR1PWMB_GPIO_Port GPIOA
#define ESTOP_Pin GPIO_PIN_4
#define ESTOP_GPIO_Port GPIOA
#define DR1PWMA_Pin GPIO_PIN_5
#define DR1PWMA_GPIO_Port GPIOA
#define MJ1CH1_Pin GPIO_PIN_6
#define MJ1CH1_GPIO_Port GPIOA
#define MJ1CH2_Pin GPIO_PIN_7
#define MJ1CH2_GPIO_Port GPIOA
#define BT2_Pin GPIO_PIN_0
#define BT2_GPIO_Port GPIOB
#define BT1_Pin GPIO_PIN_1
#define BT1_GPIO_Port GPIOB
#define DRxSTBY_Pin GPIO_PIN_2
#define DRxSTBY_GPIO_Port GPIOB
#define DR1BIN2_Pin GPIO_PIN_12
#define DR1BIN2_GPIO_Port GPIOB
#define DR1BIN1_Pin GPIO_PIN_13
#define DR1BIN1_GPIO_Port GPIOB
#define MJ4CH2_Pin GPIO_PIN_8
#define MJ4CH2_GPIO_Port GPIOA
#define MJ4CH1_Pin GPIO_PIN_9
#define MJ4CH1_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_10
#define SPI2_CS_GPIO_Port GPIOA
#define DR2BIN1_Pin GPIO_PIN_15
#define DR2BIN1_GPIO_Port GPIOA
#define DR2PWMA_Pin GPIO_PIN_3
#define DR2PWMA_GPIO_Port GPIOB
#define DR2AIN1_Pin GPIO_PIN_4
#define DR2AIN1_GPIO_Port GPIOB
#define DR2AIN2_Pin GPIO_PIN_5
#define DR2AIN2_GPIO_Port GPIOB
#define MJ2CH2_Pin GPIO_PIN_6
#define MJ2CH2_GPIO_Port GPIOB
#define MJ2CH1_Pin GPIO_PIN_7
#define MJ2CH1_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_8
#define D1_GPIO_Port GPIOB
#define RS1_Pin GPIO_PIN_9
#define RS1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
