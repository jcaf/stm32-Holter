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
  ******************************************************************************
  SPI:
  A4 - CS
  A5 - SCK
  A6 - MISO
  A7 - MOSI
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adc_val = 0;
char msg[32];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void log_uart(const char *txt, FRESULT fr)
{
    char buf[64];
    int n = sprintf(buf, "%s: %d\r\n", txt, fr);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, HAL_MAX_DELAY);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- Parámetros de adquisición ---- */
#define LED_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_14
#define ECG_FS_HZ            500U
#define HALF_SAMPLES         1024U          // media del buffer
#define BUF_SAMPLES          (HALF_SAMPLES*2)
static uint16_t adcBuf[BUF_SAMPLES];        // buffer circular DMA (ping-pong)

static volatile uint8_t half_ready = 0;
static volatile uint8_t full_ready = 0;

/* ---- SD/FatFS ---- */
FATFS SDFatFs;
FIL   file;
UINT  bw;

/* ---- Prototipos ---- */
static void ECG_Start(void);
static void ECG_Stop(void);
static FRESULT Open_Next_File(FIL *fp);
static void write_half_bigendian(FIL *fp, const uint16_t *src, UINT nSamples);


/* ============ Adquisición ============ */
static void ECG_Start(void)
{
	/* Arrancar la salida OC2 (generar OC2REF) */
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	  /* Iniciar timer que dispara el ADC a 500 Hz (TIM2->TRGO configurado en Cube) */
	    HAL_TIM_Base_Start(&htim2);

  /* DMA circular llenando adcBuf */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, BUF_SAMPLES);


}

static void ECG_Stop(void)
{
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop(&htim2);
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}

/* Callbacks DMA: marcan qué mitad del buffer está lista para escribir */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) half_ready = 1;

  //HAL_UART_Transmit(&huart1, (uint8_t *)"Half\r\n", 6, HAL_MAX_DELAY);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) full_ready = 1;

  //HAL_UART_Transmit(&huart1, (uint8_t *)"Full\r\n", 6, HAL_MAX_DELAY);
}

/* ============ SD helpers ============ */

/* Abre el siguiente FILE_###.ECG disponible */
static FRESULT Open_Next_File(FIL *fp)
{
  char name[24];

  for (int i = 1; i < 1000; ++i)
  {
    snprintf(name, sizeof(name), "FILE_%03d.ECG", i);

    FILINFO finfo;

    if (f_stat(name, &finfo) == FR_OK)
      continue;        // existe, prueba siguiente

    return f_open(fp, name, FA_CREATE_ALWAYS | FA_WRITE);
  }
  return FR_DISK_ERR;
}

/* Escribe N muestras en big-endian (alto luego bajo) en múltiplos de 512B */
static void write_half_bigendian(FIL *fp, const uint16_t *src, UINT nSamples)
{
  /* Convertimos a un buffer de bytes big-endian:
     1024 muestras -> 2048 bytes -> 4 bloques de 512B (ideal para SD) */
  static uint8_t beBuf[HALF_SAMPLES * 2];
  UINT nBytes = nSamples * 2;
  for (UINT i = 0, j = 0; i < nSamples; ++i) {
    uint16_t v = src[i];              // little-endian en RAM (STM32)
    beBuf[j++] = (uint8_t)(v >> 8);   // byte alto primero
    beBuf[j++] = (uint8_t)(v & 0xFF); // byte bajo después
  }
  UINT wrote = 0;
  FRESULT fr = f_write(fp, beBuf, nBytes, &wrote);
  if (fr != FR_OK || wrote != nBytes) {
    // error de escritura -> maneja según tu criterio (cerrar, reintentar, etc.)
    while (1);
  }
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* Ejemplo enviar texto por USART1 */
//  uint8_t msg[] = "When I fall in love!\r\n";
//  while (1)
//   {
//       HAL_ADC_Start(&hadc1);
//       if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
//       {
//           uint16_t val = HAL_ADC_GetValue(&hadc1);
//           char buf[32];
//           int n = sprintf(buf, "%u\n", val);  // enviar como texto, un valor por línea
//           HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, HAL_MAX_DELAY);
//       }
//       HAL_Delay(10); // muestreo aprox. ~500 Hz (ajusta si necesitas más rápido)
//   }
  /* Montar SD */
  //if (f_mount(&SDFatFs, (TCHAR const*)SDPath, 1) != FR_OK)
  //if (f_mount(&SDFatFs, (TCHAR const*)USERPath, 1) != FR_OK)
  /*if (f_mount(&SDFatFs, "", 0)!= FR_OK)
  {
      // error SD
      while (1);
    }

    //Abrir siguiente archivo FILE_###.ECG
    if (Open_Next_File(&file) != FR_OK) {
      while (1);
    }
    */
  	  log_uart("Inicialiazando....", 0 );

  	  /* Montar SD */
    FRESULT fr;
    fr = f_mount(&SDFatFs, "", 0);
    log_uart("f_mount", fr);
    if (fr != FR_OK) {
        while (1);  // aquí sabrás el código exacto
    }

    //
//    fr = Open_Next_File(&file);
//    log_uart("Open_Next_File", fr);
//    if (fr != FR_OK) {
//        while (1);
//    }
    //f_open(fp, name, FA_CREATE_ALWAYS | FA_WRITE);


      f_open(&file, "FILE_001.ECG", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
															  //f_lseek(&file, file.fsize);
															  //f_puts("hoy termino!\n", &file);
															  //f_close(&file);
//      char bb[]="hola1";
//      UINT wrote = 0;
//      fr = f_write(&file, bb, 5, &wrote);
//      f_close(&file);

    /* Iniciar adquisición determinista: TIM2 -> ADC -> DMA (circular) */
    ECG_Start();

    /* Objetivo: 30 s @ 500 Hz => 15000 muestras */
    uint32_t targetSamples  = 5U * ECG_FS_HZ;
    uint32_t writtenSamples = 0;

    while (writtenSamples < targetSamples)
    {
      if (half_ready)
      {
    	  log_uart("half_ready", fr);

        half_ready = 0;
        /* Escribir PRIMERA mitad del buffer: 1024 muestras => 2048 bytes (big-endian) */
        write_half_bigendian(&file, &adcBuf[0], HALF_SAMPLES);
        writtenSamples += HALF_SAMPLES;
      }
      if (full_ready) {
        full_ready = 0;
        /* Escribir SEGUNDA mitad del buffer */
        write_half_bigendian(&file, &adcBuf[HALF_SAMPLES], HALF_SAMPLES);
        writtenSamples += HALF_SAMPLES;
      }
    }

    ECG_Stop();
    f_close(&file);
    //f_mount(NULL, (TCHAR const*)SDPath, 1);   // desmontar
    //f_mount(NULL, (TCHAR const*)USERPath, 1);   // desmontar


    /* Señal de fin OK (parpadeo LED si quieres) */
    while (1) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(300);
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */





  /* Ejemplo recibir un byte */
  //uint8_t rx_data;
  //HAL_UART_Receive(&huart1, &rx_data, 1, HAL_MAX_DELAY);

//  while (1)
//  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_UART_Transmit(&huart1, msg, sizeof(msg)-1, HAL_MAX_DELAY);
//	  HAL_Delay(1000);   // retardo de 1000 ms = 1 segundo
//  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1439;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
