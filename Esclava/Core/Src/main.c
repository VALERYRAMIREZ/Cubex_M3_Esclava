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
#include "i2c.h"
#include "rtc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t mensaje[2];
RTC_TimeTypeDef horaLeida;
RTC_DateTypeDef fechaLeida;
RTC_AlarmTypeDef intAlarma;
RTC_AlarmTypeDef alarmaLeida;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//volatile unsigned long _CFSR;
//volatile unsigned long _HFSR;
//volatile unsigned long _DFSR;
//volatile unsigned long _AFSR;
//volatile unsigned long _MMAR;
//volatile unsigned long _BFAR;
//volatile unsigned long _IPSR;
//volatile unsigned long _BASEPRI;
//volatile unsigned long _PRIMASK;
//volatile unsigned long _FAULTMASK;
//volatile unsigned long _ISPR0;

volatile uint8_t fasesTiempo [18] = {0}; /* Vector para almacenar los tiempos */
									/* de encendido de las lámparas de fase. */
volatile uint8_t c1Fase = 0;
volatile uint8_t f1Actual = 1;

volatile uint8_t c2Fase = 0;
volatile uint8_t f2Actual = 1;

volatile uint8_t c3Fase = 0;
volatile uint8_t f3Actual = 1;

volatile uint8_t c4Fase = 0;
volatile uint8_t f4Actual = 1;

volatile uint8_t c5Fase = 0;
volatile uint8_t f5Actual = 1;

volatile uint8_t c6Fase = 0;
volatile uint8_t f6Actual = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void reg_Esp(void);
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc);
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
//	reg_Esp();

	fasesTiempo[0] = 5;				/* Tiempos de encendido de la fase 1, se */
	fasesTiempo[1] = 2;				/* debe eliminar esta asignación y tomar */
	fasesTiempo[2] = 4;				/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[3] = 5;				/* Tiempos de encendido de la fase 2, se */
	fasesTiempo[4] = 2;				/* debe eliminar esta asignación y tomar */
	fasesTiempo[5] = 4;				/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[6] = 5;				/* Tiempos de encendido de la fase 3, se */
	fasesTiempo[7] = 2;				/* debe eliminar esta asignación y tomar */
	fasesTiempo[8] = 4;				/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[9] = 5;				/* Tiempos de encendido de la fase 4, se */
	fasesTiempo[10] = 2;			/* debe eliminar esta asignación y tomar */
	fasesTiempo[11] = 4;			/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[12] = 5;			/* Tiempos de encendido de la fase 5, se */
	fasesTiempo[13] = 2;			/* debe eliminar esta asignación y tomar */
	fasesTiempo[14] = 4;			/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[15] = 5;			/* Tiempos de encendido de la fase 6, se */
	fasesTiempo[16] = 2;			/* debe eliminar esta asignación y tomar */
	fasesTiempo[17] = 4;			/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */


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
  /* USER CODE BEGIN 2 */
  //HAL_I2C_MspInit(&hi2c1);
  //HAL_RTC_MspInit(&hrtc);

	HAL_GPIO_WritePin(GPIOA, FASE6_ROJO_Pin | FASE1_AMA_Pin | FASE1_ROJO_Pin |
			FASE2_VERDE_Pin | FASE2_AMA_Pin | FASE2_ROJO_Pin | FASE3_ROJO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LED_STATUS_Pin | LED_FALLA_Pin | FASE3_VERDE_Pin |
			FASE4_VERDE_Pin | FASE4_AMA_Pin |FASE4_ROJO_Pin | FASE5_VERDE_Pin |
			FASE5_AMA_Pin | FASE5_ROJO_Pin | FASE3_AMA_Pin | FASE6_VERDE_Pin |
			FASE6_AMA_Pin | FASE1_VERDE_Pin, GPIO_PIN_SET);

  __HAL_RTC_ALARM_ENABLE_IT(&hrtc,RTC_IT_SEC);
  CLEAR_BIT(RTC->CRL,RTC_CRL_CNF);
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

/* USER CODE BEGIN 4 */
//void reg_Esp(void)
//{
//	_CFSR = (*((volatile unsigned long *)(0xE000ED28)));
//	_HFSR = (*((volatile unsigned long *)(0xE000ED2C)));
//	_DFSR = (*((volatile unsigned long *)(0xE000ED30)));
//	_AFSR = (*((volatile unsigned long *)(0xE000ED3C)));
//	_MMAR = (*((volatile unsigned long *)(0xE000ED34)));
//	_BFAR = (*((volatile unsigned long *)(0xE000ED38)));
//	_IPSR = __get_IPSR();
//	_BASEPRI = __get_BASEPRI();
//	_PRIMASK = __get_PRIMASK();
//	_FAULTMASK = __get_FAULTMASK();
//	_ISPR0 = __get_IPSR();
//}
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)/* Cada vez que hay  */
{									/* una interrupción por evento de segundo*/
	HAL_RTC_WaitForSynchro(hrtc);	/* lee la hora y la fecha.               */
	HAL_RTC_GetTime(hrtc,&horaLeida, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(hrtc,&fechaLeida, RTC_FORMAT_BCD);

/********************************* FASE 1*************************************/
	switch(f1Actual)
	{
	case 1:
	{
		HAL_GPIO_WritePin(GPIOB,FASE1_VERDE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,FASE1_AMA_Pin | FASE1_ROJO_Pin,GPIO_PIN_SET);
		if(c1Fase == fasesTiempo[0])
		{
			HAL_GPIO_WritePin(GPIOB,FASE1_VERDE_Pin,GPIO_PIN_SET);
			f1Actual = 2;
			c1Fase = 0;
			break;
		}
	}
	break;
	case 2:
	{
		HAL_GPIO_WritePin(GPIOA,FASE1_AMA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE1_VERDE_Pin | FASE1_ROJO_Pin,GPIO_PIN_SET);
		if(c1Fase == fasesTiempo[1])
		{
			HAL_GPIO_WritePin(GPIOA,FASE1_AMA_Pin,GPIO_PIN_SET);
			f1Actual = 3;
			c1Fase = 0;
			break;
		}
	}
	break;
	case 3:
	{
		HAL_GPIO_WritePin(GPIOA,FASE1_ROJO_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE1_VERDE_Pin | FASE1_AMA_Pin,GPIO_PIN_SET);
		if(c1Fase == fasesTiempo[2])
		{
			HAL_GPIO_WritePin(GPIOA,FASE1_ROJO_Pin,GPIO_PIN_SET);
			f1Actual = 1;
			c1Fase = 0;
			break;
		}
	}
	break;
	default:
	{
		Error_Handler();
	}
	break;
	}
	c1Fase++;

/********************************* FASE 2*************************************/
	switch(f2Actual)
	{
	case 1:
	{
		HAL_GPIO_WritePin(GPIOA,FASE2_VERDE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,FASE2_AMA_Pin | FASE2_ROJO_Pin,GPIO_PIN_SET);
		if(c2Fase == fasesTiempo[3])
		{
			HAL_GPIO_WritePin(GPIOA,FASE2_VERDE_Pin,GPIO_PIN_SET);
			f2Actual = 2;
			c2Fase = 0;
			break;
		}
	}
	break;
	case 2:
	{
		HAL_GPIO_WritePin(GPIOA,FASE2_AMA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,FASE2_VERDE_Pin | FASE2_ROJO_Pin,GPIO_PIN_SET);
		if(c2Fase == fasesTiempo[4])
		{
			HAL_GPIO_WritePin(GPIOA,FASE2_AMA_Pin,GPIO_PIN_SET);
			f2Actual = 3;
			c2Fase = 0;
			break;
		}
	}
	break;
	case 3:
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,FASE2_VERDE_Pin | FASE2_AMA_Pin,GPIO_PIN_SET);
		if(c2Fase == fasesTiempo[5])
		{
			HAL_GPIO_WritePin(GPIOA,FASE2_ROJO_Pin,GPIO_PIN_SET);
			f2Actual = 1;
			c2Fase = 0;
			break;
		}
	}
	break;
	default:
	{
		Error_Handler();
	}
	break;
	}
	c2Fase++;

/********************************* FASE 3*************************************/
	switch(f3Actual)
	{
	case 1:
	{
		HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin | FASE3_ROJO_Pin,GPIO_PIN_SET);
		if(c3Fase == fasesTiempo[6])
		{
			HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin,GPIO_PIN_SET);
			f3Actual = 2;
			c3Fase = 0;
			break;
		}
	}
	break;
	case 2:
	{
		HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE3_ROJO_Pin,GPIO_PIN_SET);
		if(c3Fase == fasesTiempo[7])
		{
			HAL_GPIO_WritePin(GPIOB,FASE3_AMA_Pin,GPIO_PIN_SET);
			f3Actual = 3;
			c3Fase = 0;
			break;
		}
	}
	break;
	case 3:
	{
		HAL_GPIO_WritePin(GPIOA,FASE3_ROJO_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE3_VERDE_Pin | FASE3_AMA_Pin,GPIO_PIN_SET);
		if(c3Fase == fasesTiempo[8])
		{
			HAL_GPIO_WritePin(GPIOA,FASE3_ROJO_Pin,GPIO_PIN_SET);
			f3Actual = 1;
			c3Fase = 0;
			break;
		}
	}
	break;
	default:
	{
		Error_Handler();
	}
	break;
	}
	c3Fase++;

/********************************* FASE 4*************************************/
	switch(f4Actual)
	{
	case 1:
	{
		HAL_GPIO_WritePin(GPIOB,FASE4_VERDE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE4_AMA_Pin | FASE4_ROJO_Pin,GPIO_PIN_SET);
		if(c4Fase == fasesTiempo[9])
		{
			HAL_GPIO_WritePin(GPIOB,FASE4_VERDE_Pin,GPIO_PIN_SET);
			f4Actual = 2;
			c4Fase = 0;
			break;
		}
	}
	break;
	case 2:
	{
		HAL_GPIO_WritePin(GPIOB,FASE4_AMA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE4_VERDE_Pin | FASE4_ROJO_Pin,GPIO_PIN_SET);
		if(c4Fase == fasesTiempo[10])
		{
			HAL_GPIO_WritePin(GPIOB,FASE4_AMA_Pin,GPIO_PIN_SET);
			f4Actual = 3;
			c4Fase = 0;
			break;
		}
	}
	break;
	case 3:
	{
		HAL_GPIO_WritePin(GPIOB,FASE4_ROJO_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE4_VERDE_Pin | FASE4_AMA_Pin,GPIO_PIN_SET);
		if(c4Fase == fasesTiempo[11])
		{
			HAL_GPIO_WritePin(GPIOB,FASE4_ROJO_Pin,GPIO_PIN_SET);
			f4Actual = 1;
			c4Fase = 0;
			break;
		}
	}
	break;
	default:
	{
		Error_Handler();
	}
	break;
	}
	c4Fase++;

/********************************* FASE 5*************************************/
	switch(f5Actual)
	{
	case 1:
	{
		HAL_GPIO_WritePin(GPIOB,FASE5_VERDE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE5_AMA_Pin | FASE5_ROJO_Pin,GPIO_PIN_SET);
		if(c5Fase == fasesTiempo[12])
		{
			HAL_GPIO_WritePin(GPIOB,FASE5_VERDE_Pin,GPIO_PIN_SET);
			f5Actual = 2;
			c5Fase = 0;
			break;
		}
	}
	break;
	case 2:
	{
		HAL_GPIO_WritePin(GPIOB,FASE5_AMA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE5_VERDE_Pin | FASE5_ROJO_Pin,GPIO_PIN_SET);
		if(c5Fase == fasesTiempo[13])
		{
			HAL_GPIO_WritePin(GPIOB,FASE5_AMA_Pin,GPIO_PIN_SET);
			f5Actual = 3;
			c5Fase = 0;
			break;
		}
	}
	break;
	case 3:
	{
		HAL_GPIO_WritePin(GPIOB,FASE5_ROJO_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE5_VERDE_Pin | FASE5_AMA_Pin,GPIO_PIN_SET);
		if(c5Fase == fasesTiempo[14])
		{
			HAL_GPIO_WritePin(GPIOB,FASE5_ROJO_Pin,GPIO_PIN_SET);
			f5Actual = 1;
			c5Fase = 0;
			break;
		}
	}
	break;
	default:
	{
		Error_Handler();
	}
	break;
	}
	c5Fase++;

/********************************* FASE 6*************************************/
	switch(f6Actual)
	{
	case 1:
	{
		HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE6_AMA_Pin | FASE6_ROJO_Pin,GPIO_PIN_SET);
		if(c6Fase == fasesTiempo[15])
		{
			HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin,GPIO_PIN_SET);
			f6Actual = 2;
			c6Fase = 0;
			break;
		}
	}
	break;
	case 2:
	{
		HAL_GPIO_WritePin(GPIOB,FASE6_AMA_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin | FASE6_ROJO_Pin,GPIO_PIN_SET);
		if(c6Fase == fasesTiempo[16])
		{
			HAL_GPIO_WritePin(GPIOB,FASE6_AMA_Pin,GPIO_PIN_SET);
			f6Actual = 3;
			c6Fase = 0;
			break;
		}
	}
	break;
	case 3:
	{
		HAL_GPIO_WritePin(GPIOA,FASE6_ROJO_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,FASE6_VERDE_Pin | FASE6_AMA_Pin,GPIO_PIN_SET);
		if(c6Fase == fasesTiempo[17])
		{
			HAL_GPIO_WritePin(GPIOA,FASE6_ROJO_Pin,GPIO_PIN_SET);
			f6Actual = 1;
			c6Fase = 0;
			break;
		}
	}
	break;
	default:
	{
		Error_Handler();
	}
	break;
	}
	c6Fase++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(GPIOB,LED_FALLA_Pin,GPIO_PIN_RESET);
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
