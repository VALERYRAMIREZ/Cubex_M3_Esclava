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
#include "adc.h"
#include "crc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
uint8_t mensaje[2];

 uint32_t cError;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Di_Estado(void);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_CRC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_MspInit(&hi2c1);

    HAL_GPIO_WritePin(GPIOA, FASE1_AMA_Pin, GPIO_PIN_SET);
      HAL_Delay(33000);
      HAL_GPIO_WritePin(GPIOA, FASE1_AMA_Pin, GPIO_PIN_RESET);

    	if(HAL_I2C_Slave_Receive_IT(&hi2c1, mensaje, 1) != HAL_OK)
    	{
    		Di_Estado();
    		HAL_GPIO_WritePin(GPIOA, FASE6_ROJO_Pin, GPIO_PIN_SET);
    	}

    	//HAL_GPIO_WritePin(GPIOA, FASE1_ROJO_Pin, GPIO_PIN_SET);
    	HAL_Delay(5000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void Di_Estado(void)
{
   	switch(HAL_I2C_GetError(&hi2c1))
    	{
    		case HAL_I2C_ERROR_NONE:
    		{
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE4_VERDE_Pin | FASE4_AMA_Pin | FASE4_ROJO_Pin, GPIO_PIN_RESET);
    		} break;
    		case HAL_I2C_ERROR_BERR:
    		{
    			HAL_GPIO_WritePin(GPIOB,FASE4_VERDE_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE4_AMA_Pin | FASE4_ROJO_Pin, GPIO_PIN_RESET);
    		} break;
    		case HAL_I2C_ERROR_ARLO:
    		{
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE4_VERDE_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE4_AMA_Pin | FASE4_ROJO_Pin, GPIO_PIN_RESET);
    		} break;
    		case HAL_I2C_ERROR_AF:
    		{
    			HAL_GPIO_WritePin(GPIOB,FASE4_AMA_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE4_VERDE_Pin | FASE4_ROJO_Pin, GPIO_PIN_RESET);
    		} break;
    		case HAL_I2C_ERROR_OVR:
    		{
    			HAL_GPIO_WritePin(GPIOB, FASE3_VERDE_Pin | FASE4_AMA_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE4_VERDE_Pin | FASE4_ROJO_Pin, GPIO_PIN_RESET);
    		} break;
    		case HAL_I2C_ERROR_DMA:
    		{
    			HAL_GPIO_WritePin(GPIOB, FASE4_VERDE_Pin | FASE4_AMA_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE4_ROJO_Pin, GPIO_PIN_RESET);
    		} break;
    		case HAL_I2C_ERROR_TIMEOUT:
    		{
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE4_VERDE_Pin | FASE4_AMA_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE4_ROJO_Pin, GPIO_PIN_RESET);
    		} break;
    		case HAL_I2C_ERROR_SIZE:
    		{
    			HAL_GPIO_WritePin(GPIOB,FASE4_ROJO_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE4_VERDE_Pin | FASE4_AMA_Pin, GPIO_PIN_RESET);
    		} break;
    		case HAL_I2C_ERROR_DMA_PARAM:
    		{
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE4_ROJO_Pin, GPIO_PIN_SET);
    			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE4_AMA_Pin, GPIO_PIN_RESET);
    		} break;
    	}

    	switch(HAL_I2C_GetState(&hi2c1))	/* Para ver el estado del periférico I2C */
      {								/* en cuando sale de las condiciones.    */
      	  case HAL_I2C_STATE_RESET:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin | FASE6_AMA_Pin | FASE1_VERDE_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_READY:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE6_AMA_Pin | FASE1_VERDE_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_BUSY:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE6_VERDE_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE6_AMA_Pin | FASE1_VERDE_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_BUSY_TX:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE6_AMA_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE6_VERDE_Pin | FASE1_VERDE_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_BUSY_RX:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE6_AMA_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin | FASE1_VERDE_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_LISTEN:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin | FASE6_AMA_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE1_VERDE_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_BUSY_TX_LISTEN:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE6_VERDE_Pin | FASE6_AMA_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE1_VERDE_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_BUSY_RX_LISTEN:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE1_VERDE_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE6_VERDE_Pin | FASE6_AMA_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_ABORT:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE1_VERDE_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin | FASE6_AMA_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_TIMEOUT:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE6_VERDE_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE6_AMA_Pin | FASE1_AMA_Pin, GPIO_PIN_RESET);
      	  } break;
      	  case HAL_I2C_STATE_ERROR:
      	  {
      		  HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE6_VERDE_Pin | FASE6_AMA_Pin,GPIO_PIN_SET);
      		  HAL_GPIO_WritePin(GPIOB,FASE1_VERDE_Pin, GPIO_PIN_RESET);
      	  } break;
      }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/* USER CODE BEGIN 4 */

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
