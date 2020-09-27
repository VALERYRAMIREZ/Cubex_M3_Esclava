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
#include <stdbool.h>
#include "fases.h"					/* Archivo de cabecera para el manejo de las
									 * fases.								  */
#include "mensaje.h"				/* Archivo de cabecera para el servicio de
									 * mensajes vía I2C.					  */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
RTC_TimeTypeDef horaLeida;			/* Estructura de tiempo para almacenar la
									 * hora del RTC después de leída.		  */
RTC_DateTypeDef fechaLeida;			/* Estructura de fecha para almacenar la
									 * fecha leída del RTC.					  */
tFases fTiempo;						/* Estructura anidada de tiempos para
									 * almacenar los tiempos de inicio de los
									 * programas para cada fase.			  */
dFases fFecha;						/* Estructura anidada de fechas para
									 * almacenar las fechas de inicio de los
									 * programas para cada fase. NOTA: Creado
									 * pero no usado hasta este momento.	  */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define 	MODO_FASE	manualTiempo[18]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef deBug_hardFault
	volatile unsigned long _CFSR;
	volatile unsigned long _HFSR;
	volatile unsigned long _DFSR;
	volatile unsigned long _AFSR;
	volatile unsigned long _MMAR;
	volatile unsigned long _BFAR;
	volatile unsigned long _IPSR;
	volatile unsigned long _BASEPRI;
	volatile unsigned long _PRIMASK;
	volatile unsigned long _FAULTMASK;
	volatile unsigned long _ISPR0;
#endif
extern uint8_t fasesTiempo[18];		/* Vector para la duración de tiempos de
									 * encendido de cada fase en modo
									 * automático.							 */
extern uint8_t bFases[6];
#ifndef pruebaFunc
	char tramaEntrada[120] = {0};	/* Vector para almacenar la trama recibida
									 * por I2C.								 */
#endif
uint32_t sensorLeido[BUFFER_ADC];	/* Arreglo para almacenar los datos leídos
									 * mediante el ADC.						 */
uint8_t canalADC = 0;				/* Variable para contabilizar el canal ADC
									 * actual.								 */
uint32_t codigoError = 0;			/* Variable para contabilizar el código de
									 * error generado.						 */
uint8_t errEnviar[] = "Error        ";/* Arreglo que almacena el código de
									 * error sucedido a enviar y la hora del
									 * evento.					 			 */
uint16_t dirMaestra = 0x66;			/* Dirección que debe tener la maestra
									 * para la recepción como esclava.		 */
_Bool dRec = 0;						/* Variable para seleccionar la función de
									 * recepción de datos vía I2C a utilizar,
									 * si es "false" entonces la función de
									 * recepción de comando recibe el comando
									 * sin argumentos, si es "true" la función
									 * de recepción de comando recibe el
									 * comando con argumentos. Se hizo así para
									 * hacer que el tramaño de la trama a
									 * recibir sea variable y de esa manera no
									 * hacer que el sistema reciba siempre una
									 * trama larga.							 */

#ifdef Proteus
	char mensajeP[] = "Hola Mundo";	/* Mensaje a imprimir en la consola de
									 * proteus, no activar ya que creo que no
									 * funciona si se compila desde el IDE al
									 * menos en las pruebas que realicé.   	 */
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef deBug_hardFault
	void reg_Esp(void);				/* Función para la lectura de los registros
									 * del sistema.							  */
#endif
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c1);
#ifdef Proteus
	void Imprime_Proteus(char *mes);/* Función para imprimir en la consola de
									 * proteus.								 */
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef pruebaFunc
char tramaEntrada[] = "iRTC 23 59 58 31 09 20 02";/* 23:59:58, 31/09/20
									 * día 2 de la semana.					  */
/* Vector para almancenar el mensaje que recibiría la tarjet cuando se utiliza
 * el modo prueba.															  */
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

/* 							Para realizar PRUEBAS:
 *
 * En main.h se encuentran las directivas para preprocesamiento de cierto
 * código según lo que se quiera hacer, las directivas son las siguientes:
 *
 * #define HAB_RTC: 		se utiliza para habilitar o deshabilitar la
 * 							funcionalidad del RTC en tiempo de compilación,
 * 							útil cuando se quiera probar la recepción de los
 * 							comandos sin activar el RTC.
 *
 * #define deBug_hardFault:	se utiliza para habilitar la lectura de los
 * 							registros del sistema en main.c y en la rutina
 * 							de interupción por hardfault. Llamando a la función
 * 							reg_Esp() puedes leer los registros del sistema
 * 							en cualquier parte del programa.
 *
 * #define respComando:		se utiliza para habilitar la respuesta de la
 * 							esclava al finalizar de ejecutar alguna orden.
 * 							Mucho de esto aún no ha sido implementado debido al
 * 							problema constante con el funcionamiento del I2C.
 *
 * #define Proteus:			se utiliza para habilitar la impresión en la consola
 * 							de proteus, esto no me ha funcionado en ningún
 * 							momento, puede ser porque no está compilado por el
 * 							compilador de proteus o porque la memoria del
 * 							dispositivo no llega hasta la dirección de memoria
 * 							requerida por proteus para que funcione la impresión.
 *
 * 		El macro MODO_FASE funciona de manera tal queselecciona entre el modo
 * automático "false" y el modo manual "true". En el modo manual se le debe
 * pasar el estado en que se quiere cada luminaria al vector manualTiempo[]
 * para que cambie el estado de cada luminaria, el vector tiene 19 elementos,
 * los primeros 18 son para uno para cada luminaria y el último elemento tiene
 * asignado el macro MODO_FASE (ver línea 46 en este) archivo. Solo debe
 * llenarse el vector con "1" o "0" donde "1" indica luminaria encendida y "0"
 * indica luminaria apagada.
 *
 * 		En el modo automático se utiliza el vector fasesTiempo[] de 18
 * elementos para almacenar los tiempos que durará encendida cada luminaria de
 * cada fase cuando esté activa (ver a partir de la línea 122 de este archivo).
 * Con esto se puede probar el manejo de fases del sistema por completo sin
 * necesidad de usar el I2C.
 *
 * 		En el caso de una corriente fuera de rango en las fases aunque el
 * Error_Handler() automáticamente debe apagar la fase, la tarjeta maestra
 * obligatoriamente debe enviarle un comando de apagado permanente a la
 * esclava mientras de resuelve el error. El apagado de las fases desde la
 * tarjeta maestra puede hacerse de dos maneras dependiendo del modo de
 * control de fases activo:
 *
 * Método 1: en el caso de las fases en modo automático debe enviársele
 * el comando de hora de activación de fase con la hora 24:60:60, eso
 * automáticamente apaga la fase y como el RTC nunca va a dar esa hora, la
 * fase no volverá a encender hasta que se le indique una hora válida. Se
 * debe tener en cuenta que si se le asigna a la fase una hora de inicio que
 * ya pasó en el día, la fase encenderá a la hora indicada del día siguiente.
 *
 * Método 2: en el caso de las fases en modo manual, basta con colocar en "1"
 * los elementos correspondientes a la fase que se desee apagar en el vector
 * manualTiempo[] y se debe recordar que no se debe actualizar esa fase
 * mientras se resuelve el error.
 *
 *		La función Apaga_Fase(...) lo que hace es poner a "1" todas las salidas
 * de una fase, luego cuando se da una hora de activación las vuelve a encender
 * en la secuencia que se le programe.
 *
 * NOTA 1: 	Puedes simular cualquier trama mensaje recibido asignando la trama
 * 		   	al vector tramaEntrada[] más arriba.
 *
 * NOTA 2: 	sin utilizar el I2C todas las funciones de manejo de fases
 * 			funcionan y están probadas pero cuando se tiene activado el I2C y
 * 			el RTC al mismo tiempo incluso sin usar las interrupciones del I2C
 * 			(usando el I2C en modo bloqueante) el sistema tiende a generar un
 * 			hard fault normalmente al actualizar el RTC. Si se utiliza el I2C
 * 			en modo bloqueante con el RTC deshabilitado entonces todos los
 * 			comandos son recibidos correctamente, si se utiliza el RTC con el
 * 			I2C deshabilitado entonces el RTC funciona perfectamente.
 */

	uint32_t tEspera = HAL_MAX_DELAY;/* Variable para establecer el tiempo que
									 * permanecerá en espera la función de
									 * recepción I2C en el main.			 */

	MODO_FASE = false;				/* Selecciona el modo de trabajo de las
									 * fases. Esto es para simulación.		 */

	for(uint8_t val = 0; val < 18; val++)/* Se le asigna el estado a cada	 */
	{								/* luminaria de fase, en este momento	 */
		manualTiempo[val] = 0;		/* con todas las luminarias encendidas,	 */
	}								/* cambiar a gusto.						 */
#ifdef deBug_hardFault
	reg_Esp();						/* Función para leer los registros de
									 * estado del sistema cuando inicia y luego
									 * se vuelve a utilizar en la rutina de
									 * interrupción por hard fault en caso de
									 * que sea necesario. Quitarle todos los
									 * comentarios a las funciones y sus
									 * definiciones en caso de que sea
									 * necesario.							 */
#endif

/* Para simular los tiempos de duración de encendido de cualquier luminaria de
 * fase modo prueba modifica los elementos de fasesTiempo[], la duración mínima
 * de cualquier luminaria debe ser de 2 segundos.							 */

#ifdef pruebaFunc
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

	/* Para simular las horas de inicio de un programa en cada fase cambia los
	 * valores asignados en cada miembro de estructura a continuación. Si se
	 * hace desde aquí, debe ser en formato BCD. Si se reciben por I2C los
	 * valores deben estar en formato decimal.								 */

	fFecha.dSistema.Date = 1;
	fFecha.dSistema.Month = 1;
	fFecha.dSistema.Year = 20;
	fFecha.dSistema.WeekDay = 1;

	fTiempo.hSistema.Hours = 0x02;
	fTiempo.hSistema.Minutes = 0x20;
	fTiempo.hSistema.Seconds = 0x00;

	fTiempo.tFase1.Hours = 2;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase1.Minutes = 0x20;	/* a correr el programa específico para  */
	fTiempo.tFase1.Seconds = 0x05;	/* la fase 1.							 */

	fTiempo.tFase2.Hours = 0x02;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase2.Minutes = 0x20;	/* a correr el programa específico para  */
	fTiempo.tFase2.Seconds = 0x06;	/* la fase 2.							 */

	fTiempo.tFase3.Hours = 0x02;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase3.Minutes = 0x20;	/* a correr el programa específico para  */
	fTiempo.tFase3.Seconds = 0x07;	/* la fase 3.							 */

	fTiempo.tFase4.Hours = 0x02;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase4.Minutes = 0x20;	/* a correr el programa específico para  */
	fTiempo.tFase4.Seconds = 0x08;	/* la fase 4.							 */

	fTiempo.tFase5.Hours = 0x02;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase5.Minutes = 0x20;	/* a correr el programa específico para  */
	fTiempo.tFase5.Seconds = 0x09;	/* la fase 5.							 */

	fTiempo.tFase6.Hours = 0x02;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase6.Minutes = 0x20;	/* a correr el programa específico para  */
	fTiempo.tFase6.Seconds = 0x10;	/* la fase 6.							 */
#endif

/* Lo siguiente indica que la hora de inicio de todas las fases es la 24:60:60,
 * esto es la forma de indicarle a la fase que va a inicia apagada y es para
 * cuando se utiliza el control de la tarjata esclava mediante comandos vía
 * I2C.																		 */

#ifndef pruebaFunc
	fTiempo.tFase1.Hours = 0x24;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase1.Minutes = 0x60;	/* a correr el programa específico para  */
	fTiempo.tFase1.Seconds = 0x60;	/* la fase 1.							 */

	fTiempo.tFase2.Hours = 0x24;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase2.Minutes = 0x60;	/* a correr el programa específico para  */
	fTiempo.tFase2.Seconds = 0x60;	/* la fase 2.							 */

	fTiempo.tFase3.Hours = 0x24;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase3.Minutes = 0x60;	/* a correr el programa específico para  */
	fTiempo.tFase3.Seconds = 0x60;	/* la fase 3.							 */

	fTiempo.tFase4.Hours = 0x24;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase4.Minutes = 0x60;	/* a correr el programa específico para  */
	fTiempo.tFase4.Seconds = 0x60;	/* la fase 4.							 */

	fTiempo.tFase5.Hours = 0x24;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase5.Minutes = 0x60;	/* a correr el programa específico para  */
	fTiempo.tFase5.Seconds = 0x60;	/* la fase 5.							 */

	fTiempo.tFase6.Hours = 0x24;	/* Instante de tiempo en el que comenzará*/
	fTiempo.tFase6.Minutes = 0x60;	/* a correr el programa específico para  */
	fTiempo.tFase6.Seconds = 0x60;	/* la fase 6.							 */
#endif


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
#ifdef pruebaFunc
//  MX_RTC_Init();
  USU_RTC_Init(&fTiempo.hSistema, &fFecha.dSistema);
#endif
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA,FASE6_ROJO_Pin | FASE1_AMA_Pin | FASE1_ROJO_Pin |
		  FASE2_VERDE_Pin | FASE2_AMA_Pin | FASE2_ROJO_Pin | FASE3_ROJO_Pin,
		  GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,LED_STATUS_Pin | LED_FALLA_Pin | FASE3_VERDE_Pin |
		  FASE4_VERDE_Pin | FASE4_AMA_Pin | FASE4_ROJO_Pin | FASE5_VERDE_Pin |
		  FASE5_AMA_Pin | FASE5_ROJO_Pin | FASE3_AMA_Pin | FASE6_VERDE_Pin |
		  FASE6_AMA_Pin | FASE1_VERDE_Pin,GPIO_PIN_SET);

  HAL_I2C_MspInit(&hi2c1);			/* Inicializando el modo I2C.            */
//    if(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)/*Intenta calibrar el ADC */
//    {									/* y en caso de haber un error de        */
//  	  Error_Handler();				/* calibración, llama a la función de    */
//    }									/* manejo de errores.                    */
#ifdef pruebaFunc
  __HAL_RTC_ALARM_ENABLE_IT(&hrtc,RTC_IT_SEC);/* Se habilita la interrupción */
  CLEAR_BIT(RTC->CRL,RTC_CRL_CNF);	/* del RTC cada segundo y luego se       */
  	  	  	  	  	  	  	  	  	/* realiza la salida forzada del modo de */
  	  	  	  	  	  	  	  	    /* configuración.                        */
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifndef pruebaFunc
	  if(HAL_I2C_Slave_Receive(&hi2c1, (uint8_t *) tramaEntrada,
			  tTrama, tEspera)		/* Función de recepción vía I2C en modo	 */
			  != HAL_OK)			/* bloqueante, varía el tiempo de espera
			  	  	  	  	  	  	 * para timout en función del tipo de
			  	  	  	  	  	  	 * mensaje a recibir, si espera recibir el
			  	  	  	  	  	  	 * formato de comando incial sin parámetros
			  	  	  	  	  	  	 * entonces el tiempo de espera es el
			  	  	  	  	  	  	 * máximo, si espera recibir el formato de
			  	  	  	  	  	  	 * comando final con parámetros entonces el
			  	  	  	  	  	  	 * tiempo de espera es de 5 segundos antes
			  	  	  	  	  	  	 * de salirse de la función.			 */
	  {
		  tEspera = HAL_MAX_DELAY;	/* En caso de que se haya vencido el 	 */
		  codigoError = 20;			/* tiempo de espera o haya existido un   */
		  Error_Handler();			/* error de recepción vuelve a asignar el*/
		  HAL_I2C_MspDeInit(&hi2c1);/* tiempo de espera máximo, luego		 */
		  MX_I2C1_Init();			/* establece el código de error			 */
		  	  	  	  	  	  	  	/* correspondiente, llama al			 */
		  	  	  	  	  	  	  	/* Error_Handler() y reinicia el módulo	 */
		  	  	  	  	  	  	  	/* I2C.									 */
	  }
	  else							/* La forma de funcionamiento de esta    */
	  {								/* etapa es como sigue: la primera vez   */
		  if(dRec == false)			/* recibe el comando sin argumentos, así */
		  {							/* usa la función Selec_Comando() para   */
			  tTrama = Selec_Comando((char *) tramaEntrada);/* actualizar    */
			  dRec = true;			/* tTrama con la cantidad de caracteres a*/
			  tEspera = 5000;
		  }							/* recibir en el próximo envío para ese  */
		  else if(dRec == true)		/* comando, se sale de la función y va	 */
		  {							/* nuevamente a la función de recepción  */
			  Selec_Opera((char *) tramaEntrada);/* por I2C, enntonces recibe*/
			  dRec = false;			/* los parámetros a asignar y vuelve al  */
			  tEspera = HAL_MAX_DELAY;
		  }							/* estado inicial.						 */
	  }
#endif
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
#ifdef deBug_hardFault
	void reg_Esp(void)
	{
		_CFSR = (*((volatile unsigned long *)(0xE000ED28)));
		_HFSR = (*((volatile unsigned long *)(0xE000ED2C)));
		_DFSR = (*((volatile unsigned long *)(0xE000ED30)));
		_AFSR = (*((volatile unsigned long *)(0xE000ED3C)));
		_MMAR = (*((volatile unsigned long *)(0xE000ED34)));
		_BFAR = (*((volatile unsigned long *)(0xE000ED38)));
		_IPSR = __get_IPSR();
		_BASEPRI = __get_BASEPRI();
		_PRIMASK = __get_PRIMASK();
		_FAULTMASK = __get_FAULTMASK();
		_ISPR0 = __get_IPSR();
	}
#endif

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)/* Cada vez que hay  */
{									/* una interrupción por evento de		 */
									/* segundo.								 */
	//HAL_RTC_WaitForSynchro(hrtc);	/* lee la hora y la fecha.               */
	HAL_RTC_GetTime(hrtc,&horaLeida, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(hrtc,&fechaLeida, RTC_FORMAT_BCD);
	if(MODO_FASE == false)					/* Si MODO_FASE es igual a cero, se		 */
	{								/* manejan las fases en modo automático. */
		Fases_Auto(fasesTiempo, &horaLeida, &fTiempo);/* Manejo de fases en
								     * automático, usará el tiempo enviado por
									 * mensaje para iniciar el manejo de las
									 * fases.       						 */
	}
	else if(MODO_FASE == true)		/* Si MODO_FASE es igual a cero, se		 */
	{								/* manejan las fases en modo manual. 	 */
		Fases_Sec();				/* Función para el manejo de las fases en*/
	}								/* modo manual.							 */

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
			codigoError = 3;
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

/* A partir de aquí se reconfigura y reinicia el ADC.						*/

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
		codigoError = 4;
		Error_Handler();
	}
	HAL_ADC_MspInit(hadc);
	if(canalADC <=6)				/* Se inicia la conversión por DMA       */
	{								/* solo hasta que se lee el sensor de    */
									/* temperatura.							 */
		if(HAL_ADC_Start_DMA(&hadc1, sensorLeido, 2*BUFFER_ADC) != HAL_OK)
		{
			codigoError = 1;
			Error_Handler();
		}
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c1)/* No hace nada	 */
{									/* porque no se está utilizando la		 */
	__asm("nop");					/* interrupción del I2C.				 */
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//	enum nFases {FASE_1 = 1, FASE_2, FASE_3, FASE_4, FASE_5, FASE_6};/* Enumerador
//									 * para el caso fAFX (asignación de
//									 * parámetros una fase a la vez).		 */
//	HAL_GPIO_WritePin(GPIOB,LED_FALLA_Pin,GPIO_PIN_RESET);/* Se enciende el
//									 * indicador de falla.					 */
//	errEnviar[6] = codigoError;		/* Se termina de llenar llenar la trama a*/
//	errEnviar[8] = horaLeida.Hours;	/* enviar con el número del código de	 */
//	errEnviar[10] = horaLeida.Minutes;/* error y la hora a la que ocurrió el */
//	errEnviar[12] = horaLeida.Seconds;/* error.								 */
//	switch(codigoError)				/* Decide la acción a tomar en función   */
//	{								/* del error generado.					 */
//	case 1:
//	{
//		/* Manejo de error no implementado. Este caso correspondería a un error
//		 * interno del microcontrolador debido a que no pudo inicializar el
//		 * ADC en modo DMA y debería ser porque el ADC no está configurado.	 */
//	}
//	case 2:							/* El caso 1 corresponde a error por	 */
//	{								/* corriente fuera de rango, por sobre	 */
//		Apaga_Fase(canalADC);		/* corriente o por corriente baja.
//									 * En este caso se apaga la fase y se
//									 * envía el código de error generado. Para
//									 * volver a encender la fase que generó el
//									 * error se le debe enviar un comando de
//									 * asignación de hora de inicio y tiempos
//									 * de duración a la fase correspondiente.*/
//	}
//	break;
//	case 3:							/* En caso de error por temperatura fuera*/
//	{								/* de rango, se apagan todas las fases y */
//		Apaga_Fase(FASE_1);			/* luego se deshabilita el RTC, el ADC	 */
//		Apaga_Fase(FASE_2);			/* queda habilitado pero como no se está */
//		Apaga_Fase(FASE_3);			/* entrando en el callback del RTC,		 */
//		Apaga_Fase(FASE_4);			/* no se está midiendo nada con el ADC.	 */
//		Apaga_Fase(FASE_5);			/* De esa manera habría que habilitar el */
//		Apaga_Fase(FASE_6);			/* RTC mediante comando.				 */
//		HAL_RTC_MspDeInit(&hrtc);
//	}
//	break;
//	case 4:
//	{
//		/* Manejo de error no implementado. Este caso correspondería a un error
//		 * interno del microcontrolador debido a que no pudo inicializar el
//		 * ADC en modo DMA y debería ser porque el ADC no está configurado.	 */
//	}
//	case 5:
//	{
//		/* Manejo de error no implementado. Este caso corresponde a una fase
//		 * no implementada, se envía el código de error pero el sistema sigue
//		 * funcionando normalmente ya que no se asignó ningún parámetro
//		 * erroneo.															 */
//	}
//	break;
//	case 6:
//	{
//		/* Manejo de error no implementado. Este caso corresponde a un comando
//		 * no implementado, se envía el código de error pero el sistema sigue
//		 * funcionando normalmente ya que no se asignó ningún parámetro
//		 * erroneo.															 */
//	}
//	break;
//	}
//	HAL_I2C_Master_Transmit(&hi2c1, dirMaestra, errEnviar, sizeof(errEnviar),
//			10000);
//  /* USER CODE END Error_Handler_Debug */
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
