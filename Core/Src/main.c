/* USER CODE BEGIN Header */
/**
 * Version funcional 100$ ok
 * Crea 2 archivos ECG y TST
 * No aplica filtros digitales
 * Avisa con flashes en el led de salida el progreso de la adquisicion
 * Cuando termina de grabar, no se graba una linea adicional indicando la fecha-hora de cierre
 *
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

#define ARM_MATH_CM3   // para tu STM32F1 (Cortex-M3)
#include "arm_math.h"
#include "filter_coeffs_q15.h"
#define USE_FILTER   0//1   // pon 0 para grabar crudo
#if USE_FILTER

	static arm_biquad_casd_df1_inst_q15 S;
#endif

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

I2C_HandleTypeDef hi2c1;

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
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void log_uart(const char *txt, FRESULT fr)
{
    char buf[64];
    int n = sprintf(buf, "%s: %d\r\n", txt, fr);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, HAL_MAX_DELAY);
}
uint8_t decToBcd(uint8_t val) {
    return ( (val/10*16) + (val%10) );
}
uint8_t bcdToDec(uint8_t val) {
    return ( (val/16*10) + (val%16) );
}


/* --- add near top --- */
#define DS3231_ADDR_7BIT 0x68
#define DS3231_ADDR      (DS3231_ADDR_7BIT << 1)

void DS3231_SetTimeDate(uint8_t hour, uint8_t min, uint8_t sec,
                        uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
    uint8_t buffer[7];
    buffer[0] = decToBcd(sec);
    buffer[1] = decToBcd(min);
    buffer[2] = decToBcd(hour);   // formato 24h
    buffer[3] = decToBcd(day);    // 1=lunes … 7=domingo
    buffer[4] = decToBcd(date);
    buffer[5] = decToBcd(month);
    buffer[6] = decToBcd(year);   // últimos 2 dígitos

    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDR, 0x00, 1, buffer, 7, HAL_MAX_DELAY);
}
typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;    // día de la semana
    uint8_t date;   // día del mes
    uint8_t month;
    uint8_t year;
} RTC_DateTime;

/* ----------------- helpers ------------------ */
static FRESULT write_timestamp_line(FIL *ftime, uint32_t block_idx, const RTC_DateTime *dt)
{
    char line[64];
    int n = snprintf(line, sizeof(line), "%06lu,%04u-%02u-%02u %02u:%02u:%02u\r\n",
                     (unsigned long)block_idx,
                     2000 + dt->year, dt->month, dt->date,
                     dt->hour, dt->min, dt->sec);
    UINT bw = 0;
    return f_write(ftime, line, n, &bw);
}

/* Use consistent HAL I2C functions: */
void DS3231_GetTimeDate(RTC_DateTime *dt)
{
    uint8_t buffer[7];
    HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDR, 0x00, 1, buffer, 7, HAL_MAX_DELAY);
    dt->sec   = bcdToDec(buffer[0]);
    dt->min   = bcdToDec(buffer[1]);
    dt->hour  = bcdToDec(buffer[2] & 0x3F);
    dt->day   = bcdToDec(buffer[3]);
    dt->date  = bcdToDec(buffer[4]);
    dt->month = bcdToDec(buffer[5]);
    dt->year  = bcdToDec(buffer[6]);
}


/* ---- Función que crea ambos archivos (ECG + TST) con el mismo índice ---- */
static FRESULT Open_Paired_Files(FIL *f_ecg, FIL *f_tst)
{
    char name_ecg[24], name_tst[24];
    FRESULT fr;
    for (int i = 1; i < 1000; ++i)
    {
        snprintf(name_ecg, sizeof(name_ecg), "FILE_%03d.ECG", i);
        FILINFO finfo;
        if (f_stat(name_ecg, &finfo) == FR_OK) continue; // existe, probar siguiente

        // crear ECG
        fr = f_open(f_ecg, name_ecg, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
        if (fr != FR_OK) return fr;

        // crear TST con mismo índice
        snprintf(name_tst, sizeof(name_tst), "FILE_%03d.TST", i);
        fr = f_open(f_tst, name_tst, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

        if (fr != FR_OK)
        {
            f_close(f_ecg);
            return fr;
        }

        return FR_OK; // ambos creados
    }
    return FR_DISK_ERR;
}

//FIltro digitl
static inline uint16_t process_sample(uint16_t raw)
{
#if USE_FILTER
    q15_t x = ADC_TO_Q15(raw);
    q15_t y;
    arm_biquad_cascade_df1_q15(&S, &x, &y, 1);
    return Q15_TO_ADC(y);
#else
    return raw;
#endif
}

///////////////////////////////////////+++++++++++++++++++++++++++++++
#include <math.h>  // necesario para lroundf
#include <stdbool.h>
static const uint8_t  NOTCH_HZ = 60;      // 50, 60 o 0 (desactivado)
/* ============== FILTROS ============== */
/* Parámetros Hampel / Media móvil */
#define HAMP_WIN 7
#define HAMP_K   6.0f
#define MA_WIN   9

/* Estado de filtros */
struct ECGFilt {
  // High-pass 0.5 Hz (biquad)
  float hp_b0, hp_b1, hp_b2, hp_a1, hp_a2;
  float hp_x1, hp_x2, hp_y1, hp_y2;

  // Notch opcional
  bool  notch_en;
  float no_b0, no_b1, no_b2, no_a1, no_a2;
  float no_x1, no_x2, no_y1, no_y2;

  // Hampel
  float ring[HAMP_WIN];
  int   idx;
  bool  primed;

  // Media móvil
  float ma_ring[MA_WIN];
  int   ma_idx;
  float ma_sum;
  bool  ma_primed;
} g;


static inline float biquad_step(float x,
        float b0, float b1, float b2,
        float a1, float a2,
        float *x1, float *x2,
        float *y1, float *y2)
{
    float y = b0*x + b1*(*x1) + b2*(*x2) - a1*(*y1) - a2*(*y2);
    *x2 = *x1;
    *x1 = x;
    *y2 = *y1;
    *y1 = y;
    return y;
}


/* Inicializa High-pass 0.5 Hz (bilineal, Q ~ 0.707) */
void biquad_init_hp_0p5Hz() {
  const float fs = (float)FS_HZ;
  const float fc = 0.5f;
  const float q  = 0.707f;
  const float K  = tanf(PI * fc / fs);
  const float norm = 1.0f / (1.0f + K/q + K*K);
  g.hp_b0 =  1.0f * norm;
  g.hp_b1 = -2.0f * norm;
  g.hp_b2 =  1.0f * norm;
  g.hp_a1 =  2.0f*(K*K - 1.0f) * norm;
  g.hp_a2 =  (1.0f - K/q + K*K) * norm;
  g.hp_x1 = g.hp_x2 = g.hp_y1 = g.hp_y2 = 0.0f;
}

/* Inicializa Notch (50/60 Hz) con Q ~ 30 */
void biquad_init_notch(uint8_t f0_hz) {
  if (f0_hz == 0) { g.notch_en = false; return; }
  const float fs   = (float)FS_HZ;
  const float q    = 30.0f;
  const float w0   = 2.0f*PI * (float)f0_hz / fs;
  const float cosw = cosf(w0), sinw = sinf(w0);
  const float alpha= sinw/(2.0f*q);

  float b0=1.0f, b1=-2.0f*cosw, b2=1.0f;
  float a0=1.0f+alpha, a1=-2.0f*cosw, a2=1.0f-alpha;

  g.no_b0 = b0/a0; g.no_b1 = b1/a0; g.no_b2 = b2/a0;
  g.no_a1 = a1/a0; g.no_a2 = a2/a0;
  g.no_x1 = g.no_x2 = g.no_y1 = g.no_y2 = 0.0f;
  g.notch_en = true;
}

/* Mediana y MAD de 7 (inserción) */
static float median7(const float *v) {
  float a[HAMP_WIN];
  for (int i=0;i<HAMP_WIN;i++) a[i]=v[i];
  for (int i=1;i<HAMP_WIN;i++){
    float key=a[i]; int j=i-1;
    while (j>=0 && a[j]>key){ a[j+1]=a[j]; j--; }
    a[j+1]=key;
  }
  return a[HAMP_WIN/2];
}
static float mad7(const float *v, float med) {
  float d[HAMP_WIN];
  for (int i=0;i<HAMP_WIN;i++) d[i]=fabsf(v[i]-med);
  for (int i=1;i<HAMP_WIN;i++){
    float key=d[i]; int j=i-1;
    while (j>=0 && d[j]>key){ d[j+1]=d[j]; j--; }
    d[j+1]=key;
  }
  return 1.4826f * d[HAMP_WIN/2] + 1e-6f;
}

/* Init pipeline */
void ecgFilt_init()
{
  memset(&g, 0, sizeof(g));
  biquad_init_hp_0p5Hz();
  biquad_init_notch(NOTCH_HZ); // 50, 60 o 0
}

/* Una muestra raw (centrada) -> filtrada */
float ecgFilt_step(int16_t x_raw) {
  float x = (float)x_raw;

  // 1) High-pass 0.5 Hz
  x = biquad_step(x, g.hp_b0,g.hp_b1,g.hp_b2,g.hp_a1,g.hp_a2,
                     &g.hp_x1, &g.hp_x2, &g.hp_y1, &g.hp_y2);
  // 2) Notch 50/60 (opcional)
  if (g.notch_en) {
    x = biquad_step(x, g.no_b0,g.no_b1,g.no_b2,g.no_a1,g.no_a2,
    				&g.hp_x1, &g.hp_x2, &g.hp_y1, &g.hp_y2);
  }
  // 3) Hampel(7)
  g.ring[g.idx] = x;
  int center = g.idx;
  g.idx = (g.idx + 1) % HAMP_WIN;
  if (!g.primed && g.idx==0) g.primed = true;

  float x_h = x;
  if (g.primed) {
    float w[HAMP_WIN];
    for (int k=0;k<HAMP_WIN;k++){
      int pos = (center - (HAMP_WIN-1) + k + HAMP_WIN) % HAMP_WIN;
      w[k] = g.ring[pos];
    }
    float med = median7(w);
    float mad = mad7(w, med);
    if (fabsf(x - med) > HAMP_K * mad) x_h = med; // reemplaza outlier
  }
  // 4) Media móvil 9
  g.ma_sum -= g.ma_ring[g.ma_idx];
  g.ma_ring[g.ma_idx] = x_h;
  g.ma_sum += x_h;
  g.ma_idx = (g.ma_idx + 1) % MA_WIN;
  if (!g.ma_primed && g.ma_idx==0) g.ma_primed = true;

  float y = g.ma_primed ? (g.ma_sum/(float)MA_WIN) : x_h;
  return y;
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

//tal vez
static volatile uint16_t procBuf[BUF_SAMPLES];  // salida procesada

/* ---- SD/FatFS ---- */
FATFS SDFatFs;
//FIL   file;
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

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) full_ready = 1;

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

    return f_open(fp, name, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

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
	FIL file_ecg, file_tst;
	FRESULT fr;
	RTC_DateTime ts_next;
	uint32_t block_idx = 0;

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
  MX_I2C1_Init();

	#if USE_FILTER
  	  arm_biquad_cascade_df1_init_q15(&S, NUM_STAGES,
                                  biquadCoeffs_q15,
                                  biquadState_q15,
                                  POST_SHIFT);
  	#endif

  // Filtros
  ecgFilt_init();


  /* USER CODE BEGIN 2 */
  // Configurar: 11:34:20, jueves (4), 2 octubre 2025
  //DS3231_SetTimeDate(11, 34, 20, 4, 2, 10, 25);




  /////////////////////////////////////////////////////
log_uart("Inicialiazando....", 0 );

	/* Montar SD */
	//FRESULT fr;
	fr = f_mount(&SDFatFs, "", 0);
	log_uart("Montando SD", fr);
	if (fr != FR_OK)
	{
		log_uart("Error montando SD", 0 );
		while (1);  // aquí sabrás el código exacto
	}
//
////Crear un nuevo archivo
//fr = Open_Next_File(&file);
//log_uart("Creando nuevo archivo", fr);
//if (fr != FR_OK)
//{
//log_uart("Error creando nuevo archivo", 0 );
//while (1);
//}
//
////f_open(&file, "FILE_001.ECG", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);



	log_uart("Creando los pares de archivos", fr);
    fr = Open_Paired_Files(&file_ecg, &file_tst);
    if (fr != FR_OK)
    {
    	log_uart("Error Creando los pares de archivos", fr);
         //Error_Handler();
    	while (1);
     }

    log_uart("INIT ", 0);
    for (int i=0; i<(2*3); i++)
    {
    	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    	HAL_Delay(50);
    }

     // preparar primer timestamp
	 DS3231_GetTimeDate(&ts_next);
     /* Iniciar adquisición determinista: TIM2 -> ADC -> DMA (circular) */
    ECG_Start();

    /* Objetivo: 30 s @ 500 Hz => 15000 muestras */
    uint32_t targetSamples  = 5U * ECG_FS_HZ;
    uint32_t writtenSamples = 0;

    while (writtenSamples < targetSamples)
    {
      if (half_ready)
      {
    	log_uart("HR", 0);

        half_ready = 0;

        //Added
//        for (UINT i = 0; i < HALF_SAMPLES; i++)
//        {
//            adcBuf[i] = process_sample(adcBuf[i]);
//        }
        //
        // El offset de escritura es 0 (primera mitad de procBuf)
        uint16_t *dest_ptr = &procBuf[0];

        for (UINT i = 0; i < HALF_SAMPLES; i++)
        {
        	uint16_t raw  = adcBuf[i];

        	// 1. Centrado (Correcto)
			int16_t centered = (int16_t)((int32_t)raw - 2048);

			// 2. Filtrado (Correcto)
			float y = ecgFilt_step(centered);

			// 3. ¡Descentrado (Corrección)!

			// Convertimos de float a int32 (redondeo)
			int32_t s_centered = lroundf(y);

			// Sumamos el offset (2048) para devolver al rango 0-4095
			int32_t s_unsigned = s_centered + 2048;

			// Aseguramos que no nos salimos del rango (Clamping/Saturación)
			if (s_unsigned < 0) s_unsigned = 0;
			if (s_unsigned > 4095) s_unsigned = 4095;

			uint16_t u = (uint16_t)s_unsigned;

			dest_ptr[i] = u; // ¡Ahora escribe la señal filtrada en el rango 0-4095
        }


        /* Escribir PRIMERA mitad del buffer: 1024 muestras => 2048 bytes (big-endian) */
        write_timestamp_line(&file_tst, block_idx, &ts_next);
		//write_half_bigendian(&file_ecg, &adcBuf[0], HALF_SAMPLES);
		write_half_bigendian(&file_ecg, &procBuf[0], HALF_SAMPLES);

		// preparar siguiente timestamp
		DS3231_GetTimeDate(&ts_next);
		block_idx++;
		writtenSamples += HALF_SAMPLES;

		//
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		//

      }
      if (full_ready)
      {
    	log_uart("FR", 0);
        full_ready = 0;

        //added filtro
//        for (UINT i = 0; i < HALF_SAMPLES; i++)
//        {
//            adcBuf[HALF_SAMPLES + i] = process_sample(adcBuf[HALF_SAMPLES + i]);
//        }
        uint16_t *dest_ptr = &procBuf[HALF_SAMPLES];

        for (UINT i = 0; i < HALF_SAMPLES; i++)
		{
			uint16_t raw  = adcBuf[HALF_SAMPLES + i];


			// 1. Centrado (Correcto)
			int16_t centered = (int16_t)((int32_t)raw - 2048);

			// 2. Filtrado (Correcto)
			float y = ecgFilt_step(centered);

			// 3. ¡Descentrado (Corrección)!

			// Convertimos de float a int32 (redondeo)
			int32_t s_centered = lroundf(y);

			// Sumamos el offset (2048) para devolver al rango 0-4095
			int32_t s_unsigned = s_centered + 2048;

			// Aseguramos que no nos salimos del rango (Clamping/Saturación)
			if (s_unsigned < 0) s_unsigned = 0;
			if (s_unsigned > 4095) s_unsigned = 4095;

			uint16_t u = (uint16_t)s_unsigned;

			dest_ptr[i] = u; // ¡Ahora escribe la señal filtrada en el rango 0-4095!
		}


        /* Escribir SEGUNDA mitad del buffer */
        write_timestamp_line(&file_tst, block_idx, &ts_next);
        //write_half_bigendian(&file_ecg, &adcBuf[HALF_SAMPLES], HALF_SAMPLES);
        //write_half_bigendian(&file_ecg, &procBuf[0], HALF_SAMPLES);
        write_half_bigendian(&file_ecg, &procBuf[HALF_SAMPLES], HALF_SAMPLES);

        DS3231_GetTimeDate(&ts_next);
        block_idx++;
        writtenSamples += HALF_SAMPLES;

        //
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		//


      }

      /* Sync cada ~4 bloques (8KB) */
        if ((writtenSamples % (HALF_SAMPLES*4)) == 0)
        {
            f_sync(&file_ecg);
            f_sync(&file_tst);
        }
    }

    ECG_Stop();

    f_close(&file_ecg);
    f_close(&file_tst);

    //f_mount(NULL, (TCHAR const*)SDPath, 1);   // desmontar
    //f_mount(NULL, (TCHAR const*)USERPath, 1);   // desmontar

    /* Señal de fin OK (parpadeo LED si quieres) */
    while (1)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(300);
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
