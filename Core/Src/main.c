/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "ecg_acq.h"
#include "ecg_sim.h"
#include "rdetect.h"
#include "ttk8_test.h"
#include "uart_tx.h"
#if ECG_STREAM
#include "ecg_stream.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if ECG_ACQ_FS_HZ != RDETECT_FS_HZ
#error "MAX30003 sample rate (max30003_config.h) and RDETECT_FS_HZ (rdetect_config.h) differ"
#endif
#ifndef ECG_STREAM
#define ECG_STREAM 0      /* 1: stream ECG + heart rate for a serial plotter (set by CMake) */
#endif
#if ECG_ACQ_BLOCK_SAMPLES > RDETECT_MAX_SAMPLES
#error "ECG_ACQ_BLOCK_SECONDS is longer than rdetect accepts (RDETECT_MAX_SAMPLES)"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
static uint32_t peaks[RDETECT_MAX_PEAKS];
static int32_t  rr[RDETECT_MAX_PEAKS + 1];

/* Last R-peak of the previous block, so the interval across the block
   boundary also counts towards the pulse. */
static uint32_t prev_peak;
static int      have_prev_peak;
static uint32_t next_block_first;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void report_block(uint32_t first, int n, const uint32_t *pk, int count) __attribute__((unused));
static void pulse_trigger(void);
static int start_acquisition(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Pulse in tenths of bpm from the median RR interval (samples), 0 if there is none. */
static uint32_t pulse_bpm_x10(int32_t *v, int n)
{
  int i, j;
  int32_t median2;

  if (n <= 0) return 0;
  for (i = 1; i < n; i++)                  /* insertion sort, n is small */
  {
    int32_t key = v[i];
    for (j = i; j > 0 && v[j - 1] > key; j--) v[j] = v[j - 1];
    v[j] = key;
  }
  median2 = (n & 1) ? 2 * v[n / 2] : v[n / 2 - 1] + v[n / 2];   /* doubled median */
  if (median2 <= 0) return 0;
  /* bpm = 60 * fs / median = 120 * fs / median2; x10 and rounded */
  return (uint32_t)((1200u * RDETECT_FS_HZ + (uint32_t)median2 / 2u) / (uint32_t)median2);
}

/* One line per block:
 *   block <k> @ <first>: <n> samples, <count> peaks, pulse <bpm> bpm: <p1> <p2> ...
 * Peaks are running sample indices (divide by fs for seconds since start). */
static void report_block(uint32_t first, int n, const uint32_t *pk, int count)
{
  int n_rr = 0;
  uint32_t bpm10;

  if (first != next_block_first) have_prev_peak = 0;   /* a block was dropped: no interval across */
  next_block_first = first + (uint32_t)n;

  for (int i = 0; i < count; i++)
  {
    uint32_t p = first + pk[i];
    if (have_prev_peak) rr[n_rr++] = (int32_t)(p - prev_peak);
    prev_peak = p;
    have_prev_peak = 1;
  }
  bpm10 = pulse_bpm_x10(rr, n_rr);

  printf("block %lu @ %lu: %d samples, %d peaks, pulse ",
         (unsigned long)(first / ECG_ACQ_BLOCK_SAMPLES), (unsigned long)first, n, count);
  if (bpm10 != 0) printf("%lu.%lu bpm:", (unsigned long)(bpm10 / 10u), (unsigned long)(bpm10 % 10u));
  else            printf("- bpm:");
  for (int i = 0; i < count; i++)
  {
    printf(" %lu", (unsigned long)(first + pk[i]));
  }
  printf("\r\n");
}

/* 5 ms pulse on D7 to start a power measurement (X-NUCLEO-LPM01A trigger). */
static void pulse_trigger(void)
{
  HAL_GPIO_WritePin(PWR_TRIG_GPIO_Port, PWR_TRIG_Pin, GPIO_PIN_SET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(PWR_TRIG_GPIO_Port, PWR_TRIG_Pin, GPIO_PIN_RESET);
}

/* Configure the MAX30003 and start acquisition; prints the outcome. */
static int start_acquisition(void)
{
  int e = ecg_acq_init(&hspi1);

  switch (e)
  {
  case MAX30003_OK:
    printf("MAX30003 ready (INFO 0x%06lx)\r\n", (unsigned long)max30003_info());
    break;
  case MAX30003_ERR_PLL:
    printf("MAX30003 PLL not locked: check FCLK on D9 (PC6). Running anyway.\r\n");
    break;
  case MAX30003_ERR_ID:
    printf("MAX30003 not found (INFO 0x%06lx): check board, 3V3 and SPI wiring. Press USER to retry.\r\n",
           (unsigned long)max30003_info());
    break;
  default:
    printf("MAX30003 SPI error %d. Press USER to retry.\r\n", e);
    break;
  }
  have_prev_peak = 0;
  next_block_first = 0;
#if ECG_STREAM
  ecg_stream_reset();
#endif
  return e;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  int err;
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
  MX_ICACHE_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /* FCLK for the MAX30003 must run before it is configured. */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  rdetect_init();
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */
  /* From here on printf only queues bytes; USART1's interrupt sends them. */
  uart_tx_init();
#ifdef TTK8_LOOPBACK
  /* TTK8 sensor board loopback test; does not return. */
  ttk8_run(&hspi1, &hdac1, &htim6);
#endif
  printf("\r\nmax30003_node: %u sps, %u s blocks\r\n", (unsigned)ECG_ACQ_FS_HZ, (unsigned)ECG_ACQ_BLOCK_SECONDS);
#if ECG_STREAM
  printf("ECG stream: ecg:<uV>,hr:<bpm>,r:<0|1> per sample, %lu.%lu s behind\r\n",
         (unsigned long)(ECG_STREAM_DELAY_SAMPLES / ECG_ACQ_FS_HZ),
         (unsigned long)(ECG_STREAM_DELAY_SAMPLES % ECG_ACQ_FS_HZ * 10u / ECG_ACQ_FS_HZ));
#endif
  err = start_acquisition();
#if ECG_SIM_ENABLE
  /* Test signal on PA4 (A2 -> ASIG). Needs JP1 on ASIG and JP2 on 0V9. */
  if (ecg_sim_start(&hdac1, &htim6, ECG_SIM_BPM) == 0)
  {
    printf("ECG sim on PA4: %lu bpm, R %lu uV on %lu mV\r\n", (unsigned long)ecg_sim_bpm(),
           (unsigned long)ECG_SIM_R_UV, (unsigned long)ECG_SIM_BASELINE_MV);
  }
  else
  {
    printf("ECG sim failed to start\r\n");
  }
#endif
  pulse_trigger();
  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    const int32_t *ecg;
    uint32_t first, rr_ms, errors;
    int n = ecg_acq_poll(&ecg, &first);

    if (n > 0)
    {
      int count = rdetect_detect(ecg, (size_t)n, peaks, RDETECT_MAX_PEAKS);
#if ECG_STREAM
      /* Copied into the playback ring before the buffer goes back to ecg_acq.
         A failed detection still streams the ECG, without peaks. */
      ecg_stream_block(first, ecg, n, peaks, count >= 0 ? count : 0);
#endif
      ecg_acq_release();
      if (count >= 0)
      {
#if !ECG_STREAM
        report_block(first, n, peaks, count);
#endif
      }
      else
      {
        printf("block %lu: rdetect error %d\r\n", (unsigned long)(first / ECG_ACQ_BLOCK_SAMPLES), count);
      }
    }

#if ECG_STREAM
    ecg_stream_pump(ecg_acq_count());
#endif
#if MAX30003_RTOR_ENABLE
    /* In stream mode INT2B is still serviced (the R-to-R interval is read in
       its interrupt), but the line is not printed: it would be ~11 s ahead of
       the plotted ECG and break the plotter format. */
    if (ecg_acq_rtor(&rr_ms) && rr_ms != 0u && !ECG_STREAM)
    {
      uint32_t bpm10 = (600000u + rr_ms / 2u) / rr_ms;
      printf("rtor %lu ms, %lu.%lu bpm\r\n", (unsigned long)rr_ms,
             (unsigned long)(bpm10 / 10u), (unsigned long)(bpm10 % 10u));
    }
#else
    (void)rr_ms;
#endif

    errors = ecg_acq_take_errors();
    if (errors != 0u)
    {
      printf("ecg_acq error 0x%lx%s%s%s\r\n", (unsigned long)errors,
             (errors & ECG_ACQ_ERR_SPI)      ? " spi" : "",
             (errors & ECG_ACQ_ERR_OVERFLOW) ? " fifo-overflow" : "",
             (errors & ECG_ACQ_ERR_OVERRUN)  ? " block-overrun" : "");
    }

    if (BspButtonState == BUTTON_PRESSED)
    {
      BspButtonState = BUTTON_RELEASED;
      /* Start over at sample 0 and trigger the power monitor, as in ecg_node.
         If the MAX30003 was not found at start-up, try to configure it again. */
      if (err == MAX30003_OK || err == MAX30003_ERR_PLL)
      {
        ecg_acq_restart();
        have_prev_peak = 0;
        next_block_first = 0;
#if ECG_STREAM
        ecg_stream_reset();
#endif
        printf("Button pressed, acquisition restarted\r\n");
#if ECG_SIM_ENABLE
        {
          /* Step the simulated heart rate so the detector is tested at several rates. */
          static const uint32_t rates[] = { 60u, 75u, 90u, 120u, 45u };
          static unsigned r = 0;
          r = (r + 1u) % (sizeof rates / sizeof rates[0]);
          (void)ecg_sim_set_bpm(rates[r]);
          printf("ECG sim: %lu bpm\r\n", (unsigned long)ecg_sim_bpm());
        }
#endif
      }
      else
      {
        err = start_acquisition();
      }
      pulse_trigger();
    }
    ecg_acq_sleep();
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

  /** Configure the System Power Supply
  */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Epod Booster
  */
  if (HAL_RCCEx_EpodBoosterClkConfig(RCC_EPODBOOSTER_SOURCE_MSIS, RCC_EPODBOOSTER_DIV1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_PWREx_EnableEpodBooster() != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set Flash latency before increasing MSIS
  */
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSIS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.MSISState = RCC_MSI_ON;
  RCC_OscInitStruct.MSISSource = RCC_MSI_RC0;
  RCC_OscInitStruct.MSISDiv = RCC_MSI_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSIS;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};
  DAC_AutonomousModeConfTypeDef sAutonomousMode = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Autonomous Mode
  */
  sAutonomousMode.AutonomousModeState = DAC_AUTONOMOUS_MODE_DISABLE;
  if (HAL_DACEx_SetConfigAutonomousMode(&hdac1, &sAutonomousMode) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */
  /* Master, mode 0, 8-bit, 96 MHz / 16 = 6 MHz (MAX30003: <= 12 MHz).
     KeepIOState holds SCLK low between HAL calls while CSB stays low. */
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
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* 96 MHz / (2929 + 1) = 32.7645 kHz on CH1 (PC6, D9) = MAX30003 FCLK (-107 ppm). */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2929;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1465;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim6.Init.Prescaler = 95;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAX_CSB_GPIO_Port, MAX_CSB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_TRIG_GPIO_Port, PWR_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MAX_INT2B_Pin */
  GPIO_InitStruct.Pin = MAX_INT2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MAX_INT2B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX_CSB_Pin */
  GPIO_InitStruct.Pin = MAX_CSB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAX_CSB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_TRIG_Pin */
  GPIO_InitStruct.Pin = PWR_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_JTMS_SWDIO_Pin */
  GPIO_InitStruct.Pin = DEBUG_JTMS_SWDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(DEBUG_JTMS_SWDIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX_INTB_Pin */
  GPIO_InitStruct.Pin = MAX_INTB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MAX_INTB_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI8_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI8_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

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
