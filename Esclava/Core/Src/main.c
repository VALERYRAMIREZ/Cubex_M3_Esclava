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
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fases.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t mensaje[2];
RTC_TimeTypeDef horaLeida;
RTC_DateTypeDef fechaLeida;
RTC_AlarmTypeDef intAlarma;
RTC_AlarmTypeDef alarmaLeida;
Fases fases;
extern uint8_t bFases[6];

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
extern uint8_t fasesTiempo[18];
uint32_t sensorLeido[BUFFER_ADC];
uint8_t canalADC = 0;
uint32_t codigoError = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void reg_Esp(void);
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
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

	fasesTiempo[0] = 6;				/* Tiempos de encendido de la fase 1, se */
	fasesTiempo[1] = 3;				/* debe eliminar esta asignación y tomar */
	fasesTiempo[2] = 4;				/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[3] = 6;				/* Tiempos de encendido de la fase 2, se */
	fasesTiempo[4] = 3;				/* debe eliminar esta asignación y tomar */
	fasesTiempo[5] = 4;				/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[6] = 6;				/* Tiempos de encendido de la fase 3, se */
	fasesTiempo[7] = 3;				/* debe eliminar esta asignación y tomar */
	fasesTiempo[8] = 4;				/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[9] = 6;				/* Tiempos de encendido de la fase 4, se */
	fasesTiempo[10] = 3;			/* debe eliminar esta asignación y tomar */
	fasesTiempo[11] = 4;			/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[12] = 6;			/* Tiempos de encendido de la fase 5, se */
	fasesTiempo[13] = 3;			/* debe eliminar esta asignación y tomar */
	fasesTiempo[14] = 4;			/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fasesTiempo[15] = 6;			/* Tiempos de encendido de la fase 6, se */
	fasesTiempo[16] = 3;			/* debe eliminar esta asignación y tomar */
	fasesTiempo[17] = 4;			/* los valores del comando recibido una  */
									/* vez se reciban por I2C.               */

	fases.tFase1.Hours = 0x02;		/* Instante de tiempo en el que comenzará*/
	fases.tFase1.Minutes = 0x20;	/* a correr el programa específico para  */
	fases.tFase1.Seconds = 0x05;	/* la fase 1.							 */

	fases.tFase2.Hours = 0x02;		/* Instante de tiempo en el que comenzará*/
	fases.tFase2.Minutes = 0x20;	/* a correr el programa específico para  */
	fases.tFase2.Seconds = 0x06;	/* la fase 2.							 */

	fases.tFase3.Hours = 0x02;		/* Instante de tiempo en el que comenzará*/
	fases.tFase3.Minutes = 0x20;	/* a correr el programa específico para  */
	fases.tFase3.Seconds = 0x07;	/* la fase 3.							 */

	fases.tFase4.Hours = 0x02;		/* Instante de tiempo en el que comenzará*/
	fases.tFase4.Minutes = 0x20;	/* a correr el programa específico para  */
	fases.tFase4.Seconds = 0x08;	/* la fase 4.							 */

	fases.tFase5.Hours = 0x02;		/* Instante de tiempo en el que comenzará*/
	fases.tFase5.Minutes = 0x20;	/* a correr el programa específico para  */
	fases.tFase5.Seconds = 0x09;	/* la fase 5.							 */

	fases.tFase6.Hours = 0x02;		/* Instante de tiempo en el que comenzará*/
	fases.tFase6.Minutes = 0x20;	/* a correr el programa específico para  */
	fases.tFase6.Seconds = 0x10;	/* la fase 6.							 */


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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA,FASE6_ROJO_Pin | FASE1_AMA_Pin | FASE1_ROJO_Pin |
		  FASE2_VERDE_Pin | FASE2_AMA_Pin | FASE2_ROJO_Pin | FASE3_ROJO_Pin,
		  GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,LED_STATUS_Pin | LED_FALLA_Pin | FASE3_VERDE_Pin |
		  FASE4_VERDE_Pin | FASE4_AMA_Pin | FASE4_ROJO_Pin | FASE5_VERDE_Pin |
		  FASE5_AMA_Pin | FASE5_ROJO_Pin | FASE3_AMA_Pin | FASE6_VERDE_Pin |
		  FASE6_AMA_Pin | FASE1_VERDE_Pin,GPIO_PIN_SET);
  //HAL_I2C_MspInit(&hi2c1);			/* Inicializando el modo I2C.            */
  HAL_RTC_MspInit(&hrtc);			/* Inicializando el RTC.                 */
  //HAL_ADC_MspInit(&hadc1);			/* Inicializando el ADC1.                */
//    if(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)/*Intenta calibrar el ADC */
//    {									/* y en caso de haber un error de        */
//  	  Error_Handler();				/* calibración, llama a la función de    */
//    }									/* manejo de errores.                    */
  __HAL_RTC_ALARM_ENABLE_IT(&hrtc,RTC_IT_SEC);/* Se habilita la interrupción */
  CLEAR_BIT(RTC->CRL,RTC_CRL_CNF);	/* del RTC cada segundo y luego se       */
  	  	  	  	  	  	  	  	  	/* realiza la salida forzada del modo de */
  	  	  	  	  	  	  	  	    /* configuración.                        */

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
	//HAL_RTC_WaitForSynchro(hrtc);	/* lee la hora y la fecha.               */
	HAL_RTC_GetTime(hrtc,&horaLeida, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(hrtc,&fechaLeida, RTC_FORMAT_BCD);

	//HAL_ADC_Stop_DMA(&hadc1);		/* Se asegura que el ADC no esté midiendo*/
									/* los sensores antes de hacer un cambio */
									/* de estados de fase.					 */
	Fases_Auto(fasesTiempo, &horaLeida, &fases);/* Manejo de fases en
									 * automático, usará el tiempo enviado por
									 * mensaje para iniciar el manejo de las
									 * fases.       						 */
  	if(HAL_ADC_Start_DMA(&hadc1,sensorLeido,2*BUFFER_ADC) != HAL_OK)
  	{
  	    codigoError = 1;			/* Se inicia la conversión de los sensores*/
  		Error_Handler();	  		/* por DMA para poder almacenar los       */
  	}						 		/* valores convertidos. La cantidad de    */
	canalADC = 0;   				/* veces que se van a enviar datos es     */
									/* igual al doble del tamaño del buffer   */
									/* debido a que como el ADC no soporta la
									 * lectura/escritura de 32 bits, entonces
									 * el DMA duplica el dato enviado en la
									 * parte alta del registro de destino y lo
									 * cuenta.                                */
}
/**
* @brief Rutina de transferencia completa por del ACD por DMA
*
* La primera vez que entra en esta rutin lo hace con el sensor de la fase 1 el
* cual está inicializado en en el archivo adc.c, las veces siguientes que entra
* en la rutina activa la lectura del ADC y transferencia por DMA para el sensor
* siguiente de fase hasta completar los 6 sensores, cada vez que termina el
* llenado del buffer correspondiente a un sensor se calcula si la fase tiene
* una falla por circuito abierto o corto circuito, en caso afirmativo se va a
* al Error_Handler(), de lo contrario sigue con el funcionamiento normal.

* @param hadc: adc handle
* @retval None */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	uint8_t tam = sizeof(sensorLeido)/sizeof(sensorLeido[0]);/* Calcula la   */
									/* cantidad de datos a procesar por      */
									/* canal. 							     */
	canalADC++;						/* Para configurar el canal ADC a medir. */
	Corrige_Med16(sensorLeido, tam);
	if(canalADC < 7)				/* Verifica si la señal está entre los   */
	{								/* valores AC permitidos si no se está   */
									/* midiendo el sensor de temperatura.    */
		if(bFases[canalADC - 1] == 1)/* Solo si hay fases encendidas se		 */
		{							/* verifica si la corriente de la fase 	 */
									/* está entre los valores permitidos.    */
			if(Verif_Ten(sensorLeido, tam, 1) != HAL_OK)/* Si está midiendo  */
			{						/* los sensores de fase, esta es la		 */
				codigoError = 2;	/* condición de error.				 	 */
				Error_Handler();
			}
		}
		else if(bFases[canalADC - 1] == 0)/* Solo si hay fases apagadas se   */
		{							/* verifica que la corriente de fase sea */
									/* igual a cero.						 */
			if(Verif_Ten(sensorLeido, tam, 0) != HAL_OK)/* Si está midiendo  */
					{				/* los sensores de fase, esta es la		 */
						codigoError = 2;/* condición de error.				 */
						Error_Handler();
					}
		}
	}
	else if(canalADC == 7)			/* Si se está midiendo el sensor de      */
	{								/* temperatura, esta es la condición.    */
		if(Verif_Temp(sensorLeido,tam))
		{
			codigoError = 12;
			Error_Handler();
		}
	}

	switch(canalADC)				/* Se cambia al sensor de fasea medir, el*/
	{								/* sensor de la fase 1 se establece en la*/
									/* configuración inicial del ADC.        */
	case 1:							/* Se establece el sensor de la fase 2.  */
	{
		sConfig.Channel = ADC_CHANNEL_1;
	}
	break;
	case 2:							/* Se establece el sensor de la fase 3.  */
	{
		sConfig.Channel = ADC_CHANNEL_2;
	}
	break;
	case 3:							/* Se establece el sensor de la fase 4.  */
	{
		sConfig.Channel = ADC_CHANNEL_3;
	}
	break;
	case 4:							/* Se establece el sensor de la fase 5.  */
	{
		sConfig.Channel = ADC_CHANNEL_4;
	}
	break;
	case 5:							/* Se establece el sensor de la fase 6.  */
	{
		sConfig.Channel = ADC_CHANNEL_5;
	}
	break;
	case 6:							/* Se establece el sensor de temperatura.*/
	{
		sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	}
	break;
	case 7:							/* Una vez que ha procesado la medida de */
	{								/* temperatura, se establece el sensor de*/
		sConfig.Channel = ADC_CHANNEL_0;/* la fase 1 nuevamente.			 */
	}
	break;
	default:						/* En caso de que el contador se pase,   */
	{								/* simplemente se sale de la función.	 */
		return;
	}
	break;
	}

	  /** Common config
	  */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	  hadc1.Init.ContinuousConvMode = ENABLE;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.NbrOfConversion = 1;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	sConfig.Rank = ADC_REGULAR_RANK_1;/* Se configura el proximo canal a     */
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;/* muestrear.           */
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		codigoError = 3;
		Error_Handler();
	}
	HAL_ADC_MspInit(hadc);
	if(canalADC <=6)				/* Se inicia la conversión por DMA       */
	{								/* solo hasta que se lee el sensor de    */
									/* temperatura.							 */
		if(HAL_ADC_Start_DMA(&hadc1,sensorLeido,2*BUFFER_ADC) != HAL_OK)
		{
			codigoError = 4;
			Error_Handler();
		}
	}
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
