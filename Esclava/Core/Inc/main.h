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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define FASE1_SENSOR_Pin GPIO_PIN_0
#define FASE1_SENSOR_GPIO_Port GPIOA
#define FASE2_SENSOR_Pin GPIO_PIN_1
#define FASE2_SENSOR_GPIO_Port GPIOA
#define FASE3_SENSOR_Pin GPIO_PIN_2
#define FASE3_SENSOR_GPIO_Port GPIOA
#define FASE4_SENSOR_Pin GPIO_PIN_3
#define FASE4_SENSOR_GPIO_Port GPIOA
#define FASE5_SENSOR_Pin GPIO_PIN_4
#define FASE5_SENSOR_GPIO_Port GPIOA
#define FASE6_SENSOR_Pin GPIO_PIN_5
#define FASE6_SENSOR_GPIO_Port GPIOA
#define FASE6_ROJO_Pin GPIO_PIN_7
#define FASE6_ROJO_GPIO_Port GPIOA
#define LED_STATUS_Pin GPIO_PIN_0
#define LED_STATUS_GPIO_Port GPIOB
#define LED_FALLA_Pin GPIO_PIN_1
#define LED_FALLA_GPIO_Port GPIOB
#define FASE3_VERDE_Pin GPIO_PIN_2
#define FASE3_VERDE_GPIO_Port GPIOB
#define FASE5_ROJO_Pin GPIO_PIN_10
#define FASE5_ROJO_GPIO_Port GPIOB
#define FASE3_AMA_Pin GPIO_PIN_12
#define FASE3_AMA_GPIO_Port GPIOB
#define FASE6_VERDE_Pin GPIO_PIN_13
#define FASE6_VERDE_GPIO_Port GPIOB
#define FASE6_AMA_Pin GPIO_PIN_14
#define FASE6_AMA_GPIO_Port GPIOB
#define FASE1_VERDE_Pin GPIO_PIN_15
#define FASE1_VERDE_GPIO_Port GPIOB
#define FASE1_AMA_Pin GPIO_PIN_8
#define FASE1_AMA_GPIO_Port GPIOA
#define FASE1_ROJO_Pin GPIO_PIN_9
#define FASE1_ROJO_GPIO_Port GPIOA
#define FASE2_VERDE_Pin GPIO_PIN_10
#define FASE2_VERDE_GPIO_Port GPIOA
#define FASE2_AMA_Pin GPIO_PIN_11
#define FASE2_AMA_GPIO_Port GPIOA
#define FASE2_ROJO_Pin GPIO_PIN_12
#define FASE2_ROJO_GPIO_Port GPIOA
#define FASE3_ROJO_Pin GPIO_PIN_15
#define FASE3_ROJO_GPIO_Port GPIOA
#define FASE4_VERDE_Pin GPIO_PIN_3
#define FASE4_VERDE_GPIO_Port GPIOB
#define FASE4_AMA_Pin GPIO_PIN_4
#define FASE4_AMA_GPIO_Port GPIOB
#define FASE4_ROJO_Pin GPIO_PIN_5
#define FASE4_ROJO_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define FASE5_VERDE_Pin GPIO_PIN_8
#define FASE5_VERDE_GPIO_Port GPIOB
#define FASE5_AMA_Pin GPIO_PIN_9
#define FASE5_AMA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
