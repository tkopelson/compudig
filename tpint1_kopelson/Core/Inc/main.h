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
#include "stm32l4xx_hal.h"

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
#define POTE_1_Pin GPIO_PIN_0
#define POTE_1_GPIO_Port GPIOA
#define POTE_2_Pin GPIO_PIN_1
#define POTE_2_GPIO_Port GPIOA
#define LED_ESTADO_1_Pin GPIO_PIN_2
#define LED_ESTADO_1_GPIO_Port GPIOA
#define LED_EXTRACTOR_1_Pin GPIO_PIN_3
#define LED_EXTRACTOR_1_GPIO_Port GPIOA
#define LED_CALEFACCION_1_Pin GPIO_PIN_4
#define LED_CALEFACCION_1_GPIO_Port GPIOA
#define LED_EXTRACTOR_2_Pin GPIO_PIN_5
#define LED_EXTRACTOR_2_GPIO_Port GPIOA
#define LED_CALEFACCION_2_Pin GPIO_PIN_6
#define LED_CALEFACCION_2_GPIO_Port GPIOA
#define LED_ESTADO_0_Pin GPIO_PIN_7
#define LED_ESTADO_0_GPIO_Port GPIOA
#define BOTON_Pin GPIO_PIN_12
#define BOTON_GPIO_Port GPIOA
#define LED_ESTADO2_1_Pin GPIO_PIN_6
#define LED_ESTADO2_1_GPIO_Port GPIOB
#define LED_ESTADO2_0_Pin GPIO_PIN_7
#define LED_ESTADO2_0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
