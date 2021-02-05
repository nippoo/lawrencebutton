/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AUDIO_BUFFER_SIZE	8192
#define LED_NUM_PIXELS 256
#define LED_BYTES (LED_NUM_PIXELS * 8 * 3)

// WS2812 definitions

#define LED_PRESCALER 0
#define LED_AUTORELOAD 104
#define LED_NULL_BYTES 500

#define LED_ZERO (LED_AUTORELOAD - 76) // This gives a HIGH signal of 350ns (and stays 900ns LOW)
#define LED_ONE 76                 // This gives a HIGH signal of 950ns (and stays 300ns LOW)
#define LED_FRAMEBUFFER_SIZE (LED_BYTES + LED_NULL_BYTES)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch2;

/* USER CODE BEGIN PV */

/* Ping-Pong buffer used for audio play */
uint16_t Audio_DMA_Buffer [AUDIO_BUFFER_SIZE]; // playing now, DMA circular buffer
uint16_t Audio_Next_Buffer [AUDIO_BUFFER_SIZE]; // playing next, precached for minimum latency

uint8_t Track_Max = 99; // maximum track on SD card (<=99)
uint8_t Track_Next = 0; // currently selected track on screen, will play next
char Track_Current_Path[8] = "000.wav";

uint32_t LED_Framebuffer	[LED_FRAMEBUFFER_SIZE] = {0};

FIL Audio_Current_fil; // currently playing file (streaming now!)
FIL Audio_Next_fil; // next up (to pre-buffer for instant playback)
FIL LED_fil; // current screen track
FATFS fatfs;

FRESULT fr;
FILINFO fi;
UINT br = 0;
uint32_t i = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


__HAL_TIM_SET_PRESCALER(&htim3, LED_PRESCALER);
__HAL_TIM_SET_AUTORELOAD(&htim3, LED_AUTORELOAD);

  // Fill screenbuffer with LOWs
  for (i = 0; i < LED_BYTES; i++)
  {
	  LED_Framebuffer[i] = LED_ZERO;
  }

  // Start circular DMA
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, LED_Framebuffer, sizeof(LED_Framebuffer) / sizeof(uint32_t));


  // Mount SD card
  fr = f_mount(&fatfs, (TCHAR const*)SDPath, 1);

  if (fr != FR_OK)
  {
	  Error_Handler();
  }

  // Find highest file (looking contiguously, no gaps allowed!)
  for (i = 0; i < 100; i++)
  {
	  sprintf(Track_Current_Path, "%03lu.wav", i);
	  fr = f_stat(Track_Current_Path, &fi);
	  if (fr != FR_OK)
	  {
		  if (i == 0) // 000.wav doesn't exist
		  {
			  Error_Handler();
		  }
		  Track_Max = i-1;
		  break;
	  }
  }

  LoadTrack();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
//	  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, GPIO_PIN_SET);
//	  HAL_Delay(10);
//	  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(500);

	  if (HAL_GPIO_ReadPin(GPIOA, SENSOR_Pin) == GPIO_PIN_SET)
	  {
		  PlayTrack(&hi2s2);
		  HAL_Delay(200);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 164;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 120;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 32;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 104;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MIDI_OUT_Pin|LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_A_Pin|LCD_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_C_Pin|LCD_D_Pin|GPIO1_Pin|GPIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_E_Pin|LCD_F_Pin|LCD_G_Pin|LCD_DP_Pin 
                          |LCD_DIG1CC_Pin|LCD_DIG2CC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO3_Pin|GPIO4_Pin|GPIO5_Pin|GPIO6_Pin 
                          |GPIO7_Pin|GPIO8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MIDI_OUT_Pin LED_STATUS_Pin */
  GPIO_InitStruct.Pin = MIDI_OUT_Pin|LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_A_Pin LCD_B_Pin */
  GPIO_InitStruct.Pin = LCD_A_Pin|LCD_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_C_Pin LCD_D_Pin GPIO1_Pin GPIO2_Pin */
  GPIO_InitStruct.Pin = LCD_C_Pin|LCD_D_Pin|GPIO1_Pin|GPIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_E_Pin LCD_F_Pin LCD_G_Pin LCD_DP_Pin 
                           LCD_DIG1CC_Pin LCD_DIG2CC_Pin */
  GPIO_InitStruct.Pin = LCD_E_Pin|LCD_F_Pin|LCD_G_Pin|LCD_DP_Pin 
                          |LCD_DIG1CC_Pin|LCD_DIG2CC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : B_PREV_Pin B_NEXT_Pin B_STOP_Pin */
  GPIO_InitStruct.Pin = B_PREV_Pin|B_NEXT_Pin|B_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO3_Pin GPIO4_Pin GPIO5_Pin GPIO6_Pin 
                           GPIO7_Pin GPIO8_Pin */
  GPIO_InitStruct.Pin = GPIO3_Pin|GPIO4_Pin|GPIO5_Pin|GPIO6_Pin 
                          |GPIO7_Pin|GPIO8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CD_Pin */
  GPIO_InitStruct.Pin = SDIO_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_Pin */
  GPIO_InitStruct.Pin = SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SENSOR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//void DMA1_Stream5_IRQHandler(void) // this function must be included to avoid DMA to crash!
//{
//  HAL_DMA_IRQHandler(&hdma_spi2_tx);
//}

void LoadTrack(void)
{
	// Loads the track in Track_Next into the Audio_Next_fil
	  fr = f_open(&Audio_Next_fil, "003.wav", FA_READ);
	  if (fr != FR_OK)
	  {
		  Error_Handler();
	  }

	  f_rewind(&Audio_Next_fil);
	  f_read(&Audio_Next_fil, &Audio_Next_Buffer[0], AUDIO_BUFFER_SIZE*2, &br);
}

void PlayTrack(I2S_HandleTypeDef *hi2s)
{
	// Moves Audio_Next_Buffer into Audio_DMA_Buffer, starts playback
	memcpy(Audio_DMA_Buffer, Audio_Next_Buffer, AUDIO_BUFFER_SIZE*2);
	Audio_Current_fil = Audio_Next_fil;
	HAL_I2S_Transmit_DMA (hi2s, Audio_DMA_Buffer, AUDIO_BUFFER_SIZE);
}

//void StartAudioBuffers(I2S_HandleTypeDef *hi2s)
//{
//  // clear buffer
////  memset (dma_buffer,0, sizeof (dma_buffer ));
//  HAL_GPIO_WritePin(GPIOE, LCD_E_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOE, LCD_DIG1CC_Pin, GPIO_PIN_SET);
//
//
////  AudioRemSize = WaveDataLength - br;
//  HAL_I2S_Transmit_DMA (hi2s,  dma_buffer, AUDIO_BUFFER_SIZE);
//}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  // second half finished, filling it up again while first  half is playing
//  FillBuffer  (&(dma_buffer [AUDIO_BUFFER_SIZE  >> 1]), AUDIO_BUFFER_SIZE >> 1);
    f_read(&Audio_Current_fil,
           &Audio_DMA_Buffer[AUDIO_BUFFER_SIZE/2],
           AUDIO_BUFFER_SIZE,
           (void *)&br);

	HAL_GPIO_WritePin(GPIOE, LCD_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, LCD_F_Pin, GPIO_PIN_RESET);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  // first half finished, filling it up again while second half is playing
    f_read(&Audio_Current_fil,
           &Audio_DMA_Buffer[0],
           AUDIO_BUFFER_SIZE,
           (void *)&br);

	HAL_GPIO_WritePin(GPIOE, LCD_F_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, LCD_G_Pin, GPIO_PIN_RESET);
//  FillBuffer  (&(dma_buffer [0]), AUDIO_BUFFER_SIZE >> 1);
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
  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, GPIO_PIN_SET);
  while (1) {}
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
