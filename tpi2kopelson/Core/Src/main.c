/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VALIDACION 0
#define COMUNICACION 1
#define FIN 2
#define NADA 3
#define VALIDACION2 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//declaracion d funcioens
void configuracionPines();
void configuracionTIM1();
void enviarStringPorUART(const char* texto);
void esperarRecepcion(void);
void selectorDesicion (void);
void prenderLedComunicacion(void);
void limpiar(void);
void finalComunicacion(void);
void actualizar(void);

//variables globales

char buffer[64];
uint8_t ch;
int i = 0;

const char* etapas[] = { "validacion", "comunicacion" ,"fin", "nada" };
int etapa_actual = NADA;

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//para modularizar el codigo mejor, hice funciones de config inicial de pines y timer asi no queda tanto quilombo
  configuracionPines();
  configuracionTIM1();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  esperarRecepcion();
	  selectorDesicion();

	  switch (etapa_actual)
	      {
	          case VALIDACION:
	              enviarStringPorUART("OK_DEV01_2025_v1.0\n");
	              limpiar();
	              break;

	          case VALIDACION2:
	              enviarStringPorUART("OK\n");
	              prenderLedComunicacion();
	              etapa_actual = COMUNICACION;
	              limpiar();
	              break;

	          case COMUNICACION:
	              actualizar();
	              limpiar();
	              break;

	          case FIN:
	              finalComunicacion();
	              limpiar();
	              break;

	          case NADA:
	              limpiar();
	              break;
	      }

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(actuador1_GPIO_Port, actuador1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, actuador2_Pin|led_estado_Pin|actuador4_Pin|actuador3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : actuador1_Pin */
  GPIO_InitStruct.Pin = actuador1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(actuador1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : actuador2_Pin led_estado_Pin actuador4_Pin actuador3_Pin */
  GPIO_InitStruct.Pin = actuador2_Pin|led_estado_Pin|actuador4_Pin|actuador3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void configuracionPines(void) {

    // 1. Habilitar relojes para GPIOA y GPIOB
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // --- Salidas digitales con pull-up ---
    // PA12 (Actuador1) -> output, pull-up
    GPIOA->MODER &= ~(3 << (12 * 2));
    GPIOA->MODER |=  (1 << (12 * 2));    // Output
    GPIOA->OTYPER &= ~(1 << 12);         // Push-pull
    GPIOA->PUPDR &= ~(3 << (12 * 2));
    GPIOA->PUPDR |=  (1 << (12 * 2));    // Pull-up

    // PB4, PB6, PB7 (Actuadores) + PB5 (LED Estado) -> output, pull-up
    GPIOB->MODER &= ~((3 << (4 * 2)) | (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->MODER |=  ((1 << (4 * 2)) | (1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2))); // Output
    GPIOB->OTYPER &= ~((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7)); // Push-pull
    GPIOB->PUPDR &= ~((3 << (4 * 2)) | (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->PUPDR |=  ((1 << (4 * 2)) | (1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2))); // Pull-up

    // --- PWM: PA8 (TIM1_CH1), PA9 (TIM1_CH2) ---
    GPIOA->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOA->MODER |=  ((2 << (8 * 2)) | (2 << (9 * 2))); // Alternate Function
    GPIOA->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOA->AFR[1] |=  ((1 << ((8 - 8) * 4)) | (1 << ((9 - 8) * 4))); // AF1 = TIM1
    GPIOA->OTYPER &= ~((1 << 8) | (1 << 9)); // Push-pull
    GPIOA->PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOA->PUPDR |=  ((1 << (8 * 2)) | (1 << (9 * 2))); // Pull-up

    // --- Entrada ADC: PA0 ---
    GPIOA->MODER |= (3 << (0 * 2));  // Analog mode
    GPIOA->PUPDR &= ~(3 << (0 * 2)); // Sin pull-up ni pull-down

    // Salir del modo deep-power-down y habilitar el regulador
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |=  ADC_CR_ADVREGEN;
    HAL_Delay(50); // Esperar a que se estabilice el regulador

    // Configurar resolución a 12 bits, alineación a derecha, modo single-shot
    ADC1->CFGR &= ~(ADC_CFGR_EXTEN | ADC_CFGR_RES | ADC_CFGR_ALIGN | ADC_CFGR_CONT);

    // Configurar tiempo de muestreo para canal 5 (PA0) — cuanto mayor, más preciso
    ADC1->SMPR1 &= ~(7 << ADC_SMPR1_SMP5_Pos);
    ADC1->SMPR1 |=  (3 << ADC_SMPR1_SMP5_Pos); // Por ej: 24.5 ciclos

    // Configurar secuencia de conversión: solo canal 5
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR1 &= ~ADC_SQR1_SQ1;
    ADC1->SQR1 |=  (5 << ADC_SQR1_SQ1_Pos);

    // Habilitar el ADC
    ADC1->CR |= ADC_CR_ADEN;



}

void configuracionTIM1(void) {

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    //elegi 1khz en el tpi2 con estos valores. f_pwm = 4Mhz / 40*100 da 1khz

    TIM1->PSC = 39;
    TIM1->ARR = 99;
    TIM1->CCR1 = 0;      // duty canal1
    TIM1->CCR2 =  0;      // duty canal2

    TIM1->CCMR1 &= ~((7 << 4) | (7 << 12));
    TIM1->CCMR1 |= ((6 << 4) | (6 << 12)); // PWM mode 1 para CH1 y CH2
    TIM1->CCMR1 |= (1 << 3) | (1 << 11);   // Preload enable

    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // Enable CH1 y CH2
    TIM1->BDTR |= TIM_BDTR_MOE;   // Main output enable
    TIM1->CR1 |= TIM_CR1_CEN;     // Enable counter
}


void esperarRecepcion(void) {

	//supongamos q la idea es recibir 'INV1'

	while (1) {
	    // Recibo de a 1 byte
	    HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
	    buffer[i++] = ch;

	    // Si llegó el fin de línea
	    if (ch == '\n') {
	        buffer[i - 1] = '\0'; // Reemplazo el '\n' con terminador de string
	        break; // Ya tengo el mensaje completo
	    }
	}

}

void selectorDesicion(void)
{
    uint32_t inicio = HAL_GetTick();

    while (1)
    {
        if ((strcmp(buffer, "INV1") == 0) && (etapa_actual == NADA))
        {
            etapa_actual = VALIDACION;
            break;
        }
        else if (strcmp(buffer, "RecibidoINV1") == 0 && etapa_actual == VALIDACION)
        {
            etapa_actual = VALIDACION2;
            break;
        }
        else if (strcmp(buffer, "FIN1") == 0)
        {
            etapa_actual = FIN;
            break;
        }
        else if (etapa_actual == COMUNICACION)
        {
            if (strcmp(buffer, "SALIDA1") == 0 ||
                strcmp(buffer, "SALIDA2") == 0 ||
                strcmp(buffer, "SALIDA3") == 0 ||
                strcmp(buffer, "SALIDA4") == 0 ||
                strncmp(buffer, "PWM1_", 5) == 0 ||
                strncmp(buffer, "PWM2_", 5) == 0 ||
				strncmp(buffer, "ADC_", 4) == 0
            	)
            {
                break;
            }
            else
            {
                limpiar();
                esperarRecepcion();
                continue;
            }
        }

        if (HAL_GetTick() - inicio > 60000)
        {
            etapa_actual = NADA;
            break;
        }

        limpiar();
        esperarRecepcion();
    }
}






void prenderLedComunicacion(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

void limpiar(void) {
    memset(buffer, 0, sizeof(buffer));
    i = 0;
}
void enviarStringPorUART(const char* texto) {
    HAL_UART_Transmit(&huart2, (uint8_t*)texto, strlen(texto), HAL_MAX_DELAY);
}

void finalComunicacion(void){

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	TIM1->CCR1 = 0;      // duty canal1
	TIM1->CCR2 =  0;      // duty canal2

	etapa_actual = NADA;
}

void actualizar(void)
{
    if (strncmp(buffer, "SALIDA", 6) == 0 && strlen(buffer) == 7)
    {
        switch (buffer[6])
        {
            case '1':
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
                enviarStringPorUART("OK_SALIDA_1\n");
                break;
            case '2':
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
                enviarStringPorUART("OK_SALIDA_2\n");
                break;
            case '3':
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
                enviarStringPorUART("OK_SALIDA_3\n");
                break;
            case '4':
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
                enviarStringPorUART("OK_SALIDA_4\n");
                break;
        }
    }
    else if (strncmp(buffer, "PWM1_", 5) == 0)
    {
        int duty = atoi(&buffer[5]);
        TIM1->CCR1 = duty;

        char respuesta[32];
        sprintf(respuesta, "OK_PWM1_%03d\n", duty);  // 3 dígitos con ceros a la izquierda
        enviarStringPorUART(respuesta);
    }
    else if (strncmp(buffer, "PWM2_", 5) == 0)
    {
        int duty = atoi(&buffer[5]);
        TIM1->CCR2 = duty;

        char respuesta[32];
        sprintf(respuesta, "OK_PWM2_%03d\n", duty);
        enviarStringPorUART(respuesta);
    }
    else if (strncmp(buffer, "ADC_1", 5) == 0)
    {
        // Iniciar conversión manualmente
        ADC1->CR |= ADC_CR_ADSTART;
        while (!(ADC1->ISR & ADC_ISR_EOC)) {} // Esperar fin de conversión
        uint16_t valor = ADC1->DR;

        char respuesta[32];
        sprintf(respuesta, "VALOR_ADC_%04u\n", valor);  // 0 a 4095
        enviarStringPorUART(respuesta);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
