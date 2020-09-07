/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FASE6_ROJO_Pin|FASE1_AMA_Pin|FASE1_ROJO_Pin|FASE2_VERDE_Pin 
                          |FASE2_AMA_Pin|FASE2_ROJO_Pin|FASE3_ROJO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_STATUS_Pin|LED_FALLA_Pin|FASE3_VERDE_Pin|FASE5_ROJO_Pin 
                          |FASE3_AMA_Pin|FASE6_VERDE_Pin|FASE6_AMA_Pin|FASE1_VERDE_Pin 
                          |FASE4_VERDE_Pin|FASE4_AMA_Pin|FASE4_ROJO_Pin|FASE5_VERDE_Pin 
                          |FASE5_AMA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin 
                           PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = FASE6_ROJO_Pin|FASE1_AMA_Pin|FASE1_ROJO_Pin|FASE2_VERDE_Pin 
                          |FASE2_AMA_Pin|FASE2_ROJO_Pin|FASE3_ROJO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin 
                           PBPin PBPin PBPin PBPin 
                           PBPin PBPin PBPin PBPin 
                           PBPin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin|LED_FALLA_Pin|FASE3_VERDE_Pin|FASE5_ROJO_Pin 
                          |FASE3_AMA_Pin|FASE6_VERDE_Pin|FASE6_AMA_Pin|FASE1_VERDE_Pin 
                          |FASE4_VERDE_Pin|FASE4_AMA_Pin|FASE4_ROJO_Pin|FASE5_VERDE_Pin 
                          |FASE5_AMA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

}

/* USER CODE BEGIN 2 */

/**
* @brief Función para apagar completamente una fase.
*
* Cuando es llamada, apaga la fase con el número pasado como argumento de
* inmediato.

* @param: numero
* @retval: ninguno. */
void Apaga_Fase(uint8_t numero)		/* Función para apagar completamente una */
{									/* fase.								 */
	switch(numero)
	{
	case 1:
	{
		HAL_GPIO_WritePin(GPIOB,FASE1_VERDE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,FASE1_AMA_Pin | FASE1_ROJO_Pin, GPIO_PIN_SET);
	}
	break;
	case 2:
	{
		HAL_GPIO_WritePin(GPIOA,FASE2_VERDE_Pin | FASE2_AMA_Pin | FASE2_ROJO_Pin, GPIO_PIN_SET);
	}
	break;
	case 3:
	{
		HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE3_AMA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,FASE3_ROJO_Pin, GPIO_PIN_SET);
	}
	break;
	case 4:
	{
		HAL_GPIO_WritePin(GPIOB,FASE4_VERDE_Pin | FASE4_AMA_Pin | FASE4_ROJO_Pin, GPIO_PIN_SET);
	}
	break;
	case 5:
	{
		HAL_GPIO_WritePin(GPIOB,FASE5_VERDE_Pin | FASE5_AMA_Pin | FASE5_ROJO_Pin, GPIO_PIN_SET);
	}
	break;
	case 6:
	{
		HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin | FASE6_AMA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,FASE1_AMA_Pin | FASE6_ROJO_Pin, GPIO_PIN_SET);
	}
	break;
	}
}
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
