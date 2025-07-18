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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int boton, flag = 0;
int16_t datomap1,datomap2,datopote1,datopote2 = 0;
void LeerPote();

void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	 if (TIM6->SR & TIM_SR_UIF) {
	    TIM6->SR &= ~TIM_SR_UIF; // Limpio flag

	 if (flag == 1) {
		 LeerPote();   // solo se activa la lectura del conversor si el boton está encendido. sino no lee.
		               // justamente como lo plantié en el diagrama en el examen teórico.
	 }
}

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}
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
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

/* Habilito relojes de buses */

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM6EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
  RCC->CCIPR |= RCC_CCIPR_ADCSEL;

  //puse estos 4 xq no hay chance de hacer andar los otros 2 pines de led estado q puse anteriormente... y no detecté la falla a tiempo.
  //ACTUALIZACIÓN POST MUESTRA funcionando perfectamente con los nuevos pines asignados
  //ERA QUE INTERNAMENTE ESTABAN CONECTADOS LOS PINES. Entonces al querer tocar de uno movia cosas del otro.

  /* NUEVO LED_ESTADO_0 -- PIN d1 [PA_9] -- SALIDA */

    GPIOA->MODER |= GPIO_MODER_MODE9_0;           //modo de salida, pushpull y que arranque apagado.
    GPIOA->MODER &= ~GPIO_MODER_MODE9_1;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT9;
    GPIOA->BSRR |= GPIO_BSRR_BR9;

    /* NUEVO LED_ESTADO_1 -- PIN d0 [PA_10] -- SALIDA */

    GPIOA->MODER |= GPIO_MODER_MODE10_0;           //modo de salida, pushpull y que arranque apagado.
    GPIOA->MODER &= ~GPIO_MODER_MODE10_1;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT10;
    GPIOA->BSRR |= GPIO_BSRR_BR10;

    /* NUEVO LED_CALEF2 -- PIN d0 [PA_11] -- SALIDA */

        GPIOA->MODER |= GPIO_MODER_MODE11_0;           //modo de salida, pushpull y que arranque apagado.
        GPIOA->MODER &= ~GPIO_MODER_MODE11_1;
        GPIOA->OTYPER &= ~GPIO_OTYPER_OT11;
        GPIOA->BSRR |= GPIO_BSRR_BR11;

    /*  NUEVO PA8 -- PIN d9 [PA_8] -- SALIDA PWM AF */
        GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;               // modo alternate funcion, pushpull
        GPIOA->MODER |= GPIO_MODER_MODE8_1;
        GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
        GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos);
        GPIOA->OTYPER &= ~GPIO_OTYPER_OT8;
        GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_Msk;


/* BOTON (ON/OFF) -- PIN D2 [PA_12] -- ENTRADA */

  GPIOA->MODER &= ~GPIO_MODER_MODE12;   // configuro entrada y pullup con resistencia externa
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD12_0;
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD12_1;

/* LED_ESTADO_0 -- PIN A6 [PA_7] -- SALIDA */

  GPIOA->MODER |= GPIO_MODER_MODE7_0;          //modo de salida, pushpull y que arranque apagado.
  GPIOA->MODER &= ~GPIO_MODER_MODE7_1;
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT7;
  GPIOA->BSRR |= GPIO_BSRR_BR7;

/* LED_ESTADO_1 -- PIN A7 [PA_2] -- SALIDA */

  GPIOA->MODER |= GPIO_MODER_MODE2_0;    //modo de salida, pushpull y que arranque apagado.
  GPIOA->MODER &= ~GPIO_MODER_MODE2_1;
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;
  GPIOA->BSRR |= GPIO_BSRR_BR2;

/* LED_ESTADO2_0 -- PIN D4 [PB_7] -- SALIDA */

  GPIOB->MODER |= GPIO_MODER_MODE7_0;    //modo de salida, pushpull y que arranque apagado.
  GPIOB->MODER &= ~GPIO_MODER_MODE7_1;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT7;
  GPIOB->BSRR |= GPIO_BSRR_BR7;

/* LED_ESTADO2_1 -- PIN D5 [PB_6] -- SALIDA */

  GPIOB->MODER |= GPIO_MODER_MODE6_0;  //modo de salida, pushpull y que arranque apagado.
  GPIOB->MODER &= ~GPIO_MODER_MODE6_1;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT6;
  GPIOB->BSRR |= GPIO_BSRR_BR6;

/* LED_CALEFACCION_1 -- PIN A3 [PA_4] -- SALIDA */

  GPIOA->MODER |= GPIO_MODER_MODE4_0;  //modo de salida, pushpull y que arranque apagado.
  GPIOA->MODER &= ~GPIO_MODER_MODE4_1;
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;
  GPIOA->BSRR |= GPIO_BSRR_BR4;

/* LED_CALEFACCION_2 -- PIN A4 [PA_6] -- SALIDA */

  GPIOA->MODER |= GPIO_MODER_MODE6_0;  //modo de salida, pushpull y que arranque apagado.
  GPIOA->MODER &= ~GPIO_MODER_MODE6_1;
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT6;
  GPIOA->BSRR |= GPIO_BSRR_BR6;

/* LED_EXTRACTOR_1 -- PIN A2 [PA_3] -- SALIDA PWM AF */

  GPIOA->MODER |= GPIO_MODER_MODE3_1;           // Modo alternate function y pushpull
  GPIOA->MODER &= ~GPIO_MODER_MODE3_0;
  GPIOA->AFR[0] |=  (1 << GPIO_AFRL_AFSEL3_Pos);
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;

/* LED_EXTRACTOR_2 -- PIN A5 [PA_5] -- SALIDA PWM AF */

  GPIOA->MODER |= GPIO_MODER_MODE5_1;              // Modo alternate function y pushpull
  GPIOA->MODER &= ~GPIO_MODER_MODE5_0;
  GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL5_Pos);
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

/* POTE_1 -- PIN A0 [PA_0] -- ADC ENTRADA */

  GPIOA->MODER |= GPIO_MODER_MODE0;             // PA0 en modo analógico
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;            // Sin pull-up/down

/* POTE_2 -- PIN A1 [PA_1] -- ADC ENTRADA */

  GPIOA->MODER |= GPIO_MODER_MODE1;             // PA1 en modo analógico
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1;            // Sin pull-up/down


  /* Inicializo los timers aca */


  // TIM2 (1khz ya q en el examen calculé eso, aprovecho los valores dados ja!)
  // los del tim2 canal 1 estan comentados porque use otro pin para el pwm. que es el tim1. asi q no me sirve,
  //esto es por lo q explique arriba, pero podría haber funcionado tranquilamente.
  TIM2->PSC = 39;
  TIM2->ARR = 99;
//  TIM2->CCR1 = 0;
  TIM2->CCR4 = 0; // 50% duty por defecto
//  TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1
//  TIM2->CCMR1 |= TIM_CCMR1_OC1PE;                     // Preload enable
  TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // PWM mode 1
  TIM2->CCMR2 |= TIM_CCMR2_OC4PE;    // Preload enable
  TIM2->CCER |= TIM_CCER_CC4E;       // Enable output
//  TIM2->CCER  |= TIM_CCER_CC1E;      // Enable output
  TIM2->CR1 |= TIM_CR1_CEN;          // Enable timer


// TIM6 (cada medio seg) para lectura del adc
  NVIC->ISER[1] |= (1 << 22);
  TIM6->PSC = 999;
  TIM6->ARR = 1999;
  TIM6->DIER |= TIM_DIER_UIE;
  TIM6->CR1 |= TIM_CR1_CEN;

//TIM1 prueba para el pin nuevo q agregué para reemplazar el anterior. es tim1 xq a ese pin le corresponde tim1_ch1
  TIM1->PSC = 39;
  TIM1->ARR = 99;
  TIM1->CCR1 = 0;  // 50% duty cycle
  TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;  // Limpio los bits del modo
  TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;  // Preload enable
  TIM1->CCER |= TIM_CCER_CC1E;     // Habilita la salida del canal 1
  TIM1->CR1  |= TIM_CR1_CEN;       // Habilita el contador
  TIM1->BDTR |= TIM_BDTR_MOE;      // Habilita salidas del timer (necesario en timers avanzados como TIM1)

  //Configuro ADC
     ADC1->CR &=~ADC_CR_DEEPPWD;//Para que salga del estado deep sleep
     ADC1->CR |=ADC_CR_ADVREGEN;//REG INT DE VOLTAJE
     HAL_Delay(50);//delay para que se estabilice el reg de voltaje antes de convertir
     ADC1->CFGR &= ~ADC_CFGR_EXTEN;
     ADC1->CFGR &= ~ADC_CFGR_RES; //res de 12 bits
     ADC1->CFGR &= ~ADC_CFGR_ALIGN; //ALINEACION X DER
     ADC1->CFGR &= ~ADC_CFGR_CONT; //SETEO SINGLE CONVERSION

  // config entradas
     ADC1->SMPR1 |= (3 << ADC_SMPR1_SMP5_Pos); // Por ejemplo 24.5 ciclos
     ADC1->SMPR1 |= (3 << ADC_SMPR1_SMP6_Pos); // Igual tiempo para ambos
     ADC1->SQR1 &= ~ADC_SQR1_L;
     ADC1->SQR1 |= ADC_SQR1_L_0;
     ADC1->SQR1 &= ~ADC_SQR1_SQ1;
     ADC1->SQR1 |= (5 << ADC_SQR1_SQ1_Pos);    // canal 5
     ADC1->SQR1 &= ~ADC_SQR1_SQ2;
     ADC1->SQR1 |= (6 << ADC_SQR1_SQ2_Pos);    // canal 6

     ADC1->CR |=ADC_CR_ADEN;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  boton = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12);

	  if (boton == GPIO_PIN_RESET) // Botón presionado (0 lógico ya que tengo logica pullup)
	          {
		       flag = 1; // activa la conversión

		       // esto es para la del ambiente 1
		       if (datomap1 <= 5) {
		                      TIM2->CCR4 = 0; //pwm off
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //led calefaccion
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //led estado 1
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // led estado 0
		                      }

		                      if ((datomap1 > 5) & (datomap1 <=20) ) {
		                      TIM2->CCR4 = 0; //pwm off
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //led calefaccion
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //led estado 1
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //led estado 0

		                      }

		                      if ((datomap1 > 20) & (datomap1 <=30) ) {
		                      TIM2->CCR4 = 40; //pwm 40%
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //led calefaccion
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //led estado 1
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //led estado 0
		                      }

		                      if ((datomap1 > 30) & (datomap1 <=35) ) {
		                      TIM2->CCR4 = 60; //pwm 60%
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //led calefaccion
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //led estado 1
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //led estado 0
		                      }

		                      if (datomap1 > 35) {
		                      TIM2->CCR4 = 100; //pwm 100%
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //led calefaccion
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //led estado 1
		                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //led estado 0
		                      }

               // esto es para la del ambiente 2
               // aqui tmb deje comentado lo de los pines que use antes, para que quede en evidencia de que intenté pero bueno, ya era una cuestion interna al stm32
               // la logica era correcta y estaba todoo bien seteado.

               if (datomap2 <= 5) {
//                                 TIM2->CCR1 = 0; // pwm off
//                                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // led calefaccion2
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // led estado2 1
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // led estado2 0
            	   TIM1->CCR1 = 0; // pwm off
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //nuevo led calefaccion
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //NUEVO led estado2
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //NUEVO led estado2
                              }

               if ((datomap2 > 5) & (datomap2 <= 20)) {
//                                  TIM2->CCR1 = 0; // pwm off
//                                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // led calefaccion2
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // led estado2 1
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // led estado2 0
            	   TIM1->CCR1 = 0; // pwm off
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //nuevo led calefaccion
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);//NUEVO led estado2
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//NUEVO led estado2
                              }

               if ((datomap2 > 20) & (datomap2 <= 30)) {
//                                  TIM2->CCR1 = 40; // pwm 40%
//                                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // led calefaccion2
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // led estado2 1
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // led estado2 0
            	   TIM1->CCR1 = 40; // pwm 40%
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //nuevo led calefaccion
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);//NUEVO led estado2
            	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//NUEVO led estado2
                              }

              if ((datomap2 > 30) & (datomap2 <= 35)) {
//                                  TIM2->CCR1 = 60; // pwm 60%
//                                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // led calefaccion2
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // led estado2 1
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // led estado2 0
            	  TIM1->CCR1 = 60; // pwm 60%
            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //nuevo led calefaccion
            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);//NUEVO led estado2
            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//NUEVO led estado2
                              }

              if (datomap2 > 35) {
//                                  TIM2->CCR1 = 100; // pwm 100%
//                                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // led calefaccion2
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // led estado2 1
//                                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // led estado2 0
            	  TIM1->CCR1 = 100; // pwm 100%
            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //nuevo led calefaccion
            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);//NUEVO led estado2
            	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//NUEVO led estado2

                              }
	          }

	   else // Botón no presionado (1 lógico) BASICAMENTE APAGO TODO.
		   {
	            flag = 0;
	            TIM2->CCR4 = 0; //pwm 0%
	            TIM1->CCR1 = 0; //pwm 0%
//	            TIM2->CCR1 = 0; //pwm 0%
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //led calefaccion
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //led estado 1
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //led estado 0
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //led calefaccion2
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //led estado2 1
	            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //led estado2 0
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);//NUEVO led estado2
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//NUEVO led estado2
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);//nuevo led calefaccion
	          }

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LED_ESTADO_1_Pin|LED_CALEFACCION_1_Pin|LED_CALEFACCION_2_Pin|LED_ESTADO_0_Pin
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_ESTADO2_1_Pin|LED_ESTADO2_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_ESTADO_1_Pin LED_CALEFACCION_1_Pin LED_CALEFACCION_2_Pin LED_ESTADO_0_Pin
                           PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = LED_ESTADO_1_Pin|LED_CALEFACCION_1_Pin|LED_CALEFACCION_2_Pin|LED_ESTADO_0_Pin
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOTON_Pin */
  GPIO_InitStruct.Pin = BOTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ESTADO2_1_Pin LED_ESTADO2_0_Pin */
  GPIO_InitStruct.Pin = LED_ESTADO2_1_Pin|LED_ESTADO2_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LeerPote(void) {
// basicamente la funcion esta hace la lectura de ambos potes, primero le un pote espera a que convierta
// y luego guarda el valor, limpio, selecciono el otro canal del adc y convierto en el otro y guardo el valor.

	ADC1->SQR1 &= ~ ADC_SQR1_SQ1; //LIMPIO
    ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);  // Canal 5 en la primera conversión
    ADC1->CR |= ADC_CR_ADSTART;            // Iniciar conversión

    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {}    // Esperar fin de conversión

    datopote1 = ADC1->DR;
    datomap1 = ( (datopote1*120) / 4095 ) - 20 ; // mapeo para que vaya de  -20°C a 100°C asi es más visual para que el if sea mas legible.

    ADC1->SQR1 &= ~ ADC_SQR1_SQ1; //LIMPIO
    ADC1->SQR1 = (6 << ADC_SQR1_SQ1_Pos);  // Canal 6 en la primera conversión
    ADC1->CR |= ADC_CR_ADSTART;            // Iniciar conversión

    while (!(ADC1->ISR & ADC_ISR_EOC));    // Esperar fin de conversión

    datopote2 = ADC1->DR;
    datomap2 = ( (datopote2*120) / 4095 ) - 20 ; // mapeo para que vaya de  -20°C a 100°C asi es más visual para que el if sea mas legible.

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
