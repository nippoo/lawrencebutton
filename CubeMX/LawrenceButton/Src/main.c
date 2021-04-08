/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  * Main code for Lawrence's "big button" - low-latency sample playback with
  * WS2812b flex LED panel light.
  *
  * Contact Max Hunter for queries or hardware!
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct GPIOPin_TypeDef
{
	GPIO_TypeDef *port;
	uint16_t *pin;
} GPIOPin_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LED_NUM_PIXELS 256

#define AUDIO_BUFFER_SIZE 4096
#define LED_COLOUR_BUFFER_SIZE 1024

#define LED_ZERO 28 // HIGH signal of 350ns, then 900ns LOW
#define LED_ONE 76  // HIGH signal of 950ns, then 300ns LOW

#define LED_BYTES (LED_NUM_PIXELS * 8 * 3)
#define LED_NULL_BYTES 500
#define LED_FRAMEBUFFER_SIZE (LED_BYTES + LED_NULL_BYTES)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define GRB_WORD(R, G, B) (((G) << 16) | ((R) << 8) | (B))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch2;

/* USER CODE BEGIN PV */

static GPIOPin_TypeDef LCDs[7] = {
	LCD_A_GPIO_Port, LCD_A_Pin,
	LCD_B_GPIO_Port, LCD_B_Pin,
	LCD_C_GPIO_Port, LCD_C_Pin,
	LCD_D_GPIO_Port, LCD_D_Pin,
	LCD_E_GPIO_Port, LCD_E_Pin,
	LCD_F_GPIO_Port, LCD_F_Pin,
	LCD_G_GPIO_Port, LCD_G_Pin,
};

const uint8_t b7SegmentTable[11] = {
0x3F, /*0*/
0x06, /*1*/
0x5B, /*2*/
0x4F, /*3*/
0x66, /*4*/
0x6D, /*5*/
0x7D, /*6*/
0x07, /*7*/
0x7F, /*8*/
0x6F, /*9*/
0x79  /*E (error)*/
};

const uint16_t Debounce_ms[4] = {80, 200, 200, 200}; // TRIGGER, PREV, NEXT, STOP debounce in ms
uint16_t Debounce_current[4] = {0};


/* Ping-Pong buffer used for audio play */
uint16_t Audio_DMA_Buffer [AUDIO_BUFFER_SIZE]; // playing now, DMA circular buffer
uint16_t Audio_Next_Buffer [AUDIO_BUFFER_SIZE]; // playing next, precached for minimum latency

uint8_t Track_Max = 99; // maximum track on SD card (<=99)
uint8_t Track_Next = 0; // currently selected track on screen, will play next
char Track_Next_Path[8] = "000.wav";
uint8_t Track_Preloaded = 0; // BOOL is the first chunk of the track preloaded into RAM?
uint8_t Track_Playing = 0; // BOOL is the track playing...

uint32_t LED_Framebuffer	[LED_FRAMEBUFFER_SIZE] = {0};
uint32_t LED_Colour_Buffer	[LED_COLOUR_BUFFER_SIZE] = {0};
uint32_t LED_Current_Frame = 0; // what frame of the file we're on
uint32_t LED_Last_Frame = 0; // number of frames total
char LED_Current_Path[8] = "000.led";

FIL Audio_Current_fil; // currently playing file (streaming now!)
FIL Audio_Next_fil; // next up (to pre-buffer for instant playback)
FIL LED_fil; // current screen track
FATFS fatfs;

FRESULT fr;
FILINFO fi;
UINT br = 0;
uint32_t i = 0;
uint8_t curr_digit = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


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
	  sprintf(Track_Next_Path, "%03lu.wav", i);
	  fr = f_stat(Track_Next_Path, &fi);
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

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);

  LoadTrack();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, GPIO_PIN_SET);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, GPIO_PIN_RESET);
	  HAL_Delay(500);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 83968;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 41983;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
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

  /*Configure GPIO pin : TRIGGER_Pin */
  GPIO_InitStruct.Pin = TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TRIGGER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == TRIGGER_Pin)
	{
		if (Debounce_current[0] == 0)
		{
			Debounce_current[0] = Debounce_ms[0];

			if (Track_Playing)
			{
			  StopTrack(&hi2s2);
			}

			PlayTrack(&hi2s2);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
// Main loop. 2 timers - TIM2 every 1ms (main switch detection loop) and TIM4 every 40ms (display)
{
	  if (htim == &htim2) // every 1ms
	  {
		  // MAIN TRIGGER
		  if (Debounce_current[0] != 0)
		  {
			  Debounce_current[0]--;
		  }

		  // PREV button
		  if (Debounce_current[1] == 0)
		  {
			  if (HAL_GPIO_ReadPin(B_PREV_GPIO_Port, B_PREV_Pin) == GPIO_PIN_RESET)
			  {
				  Debounce_current[1] = Debounce_ms[1];

				  if (Track_Next != 0)
				  {
					  Track_Next--;
					  Track_Preloaded = 0;
				  }
				  LoadTrack();
			  }
		  }
		  else
		  {
			  Debounce_current[1]--;
		  }

		  // NEXT button
		  if (Debounce_current[2] == 0)
		  {
			  if (HAL_GPIO_ReadPin(B_NEXT_GPIO_Port, B_NEXT_Pin) == GPIO_PIN_RESET)
			  {
				  Debounce_current[2] = Debounce_ms[2];

				  if (Track_Next < Track_Max)
				  {
					  Track_Next++;
					  Track_Preloaded = 0;
				  }
				  LoadTrack();
			  }
		  }
		  else
		  {
			  Debounce_current[2]--;
		  }

		  // STOP button
		  if (Debounce_current[3] == 0)
		  {
			  if (HAL_GPIO_ReadPin(B_STOP_GPIO_Port, B_STOP_Pin) == GPIO_PIN_RESET)
			  {
				  Debounce_current[3] = Debounce_ms[3];

				  StopTrack(&hi2s2);
			  }
		  }
		  else
		  {
			  Debounce_current[3]--;
		  }

		  UpdateLCD(curr_digit);
		  curr_digit = 1 - curr_digit;
	  }

	  else if (htim == &htim4) // every 40ms, 25fps
	  {
		  if (Track_Playing)
		  {
			  if(LED_Current_Frame < LED_COLOUR_BUFFER_SIZE)
			  {
				  Set_LED_Colour(LED_Colour_Buffer[LED_Current_Frame]);
				  LED_Current_Frame++;
			  }
		  }
	}

}

void LoadLEDBuffer(void)
{
	// Loads the entire file into LED buffer.

	for(uint32_t i = 0; i < LED_COLOUR_BUFFER_SIZE; i++)
	{
		LED_Colour_Buffer[i] = 0;
	}

	sprintf(LED_Current_Path, "%03d.led", Track_Next);
	fr = f_open(&LED_fil, LED_Current_Path, FA_READ);

	if (fr == FR_OK)
	{
		char linebuffer[8] = "0000000";
		for(uint32_t i = 0; i < LED_COLOUR_BUFFER_SIZE; i++)
		{
			f_gets(linebuffer, 8, &LED_fil);
			if f_eof(&LED_fil)
			{
				f_close(&LED_fil);
				return;
			}
			else
			{
				LED_Colour_Buffer[i] = strtol(linebuffer, '\0', 16);

				// swap RGB to GRB (?!!) for DMA to WS2812
				LED_Colour_Buffer[i] = (LED_Colour_Buffer[i] & 255) // B
						+ ((LED_Colour_Buffer[i] & 0xff00) << 8) // G
						+ ((LED_Colour_Buffer[i] & 0xff0000) >> 8); // R
			}
		}
	}
	f_close(&LED_fil);
}

void StopTrack(I2S_HandleTypeDef *hi2s)
{
	// Stop what's now playing.

	uint8_t data[2];
	data[0] = 0x03;
	data[1] = 255;

	HAL_I2C_Master_Transmit(&hi2c1, 0x98, data, 2, 100); // soft-mute the audio to avoid clicks

    // Wait for playback to fade out before stopping DMA (4ms is -20dB... that should be enough)
    HAL_Delay(4);
    HAL_I2S_DMAStop(hi2s);

    // Unmute!
    data[1] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x98, data, 2, 100);
    Track_Playing = 0;

    // Fill screenbuffer with LOWs
    for (i = 0; i < LED_BYTES; i++)
    {
  	  LED_Framebuffer[i] = LED_ZERO;
    }
}

void LoadTrack(void)
{
	// Loads the track in Track_Next into the Audio_Next_fil
	  fr = f_close(&Audio_Next_fil);

	  sprintf(Track_Next_Path, "%03d.wav", Track_Next);
	  fr = f_open(&Audio_Next_fil, Track_Next_Path, FA_READ);
	  if (fr != FR_OK)
	  {
		  Error_Handler();
	  }

	  f_rewind(&Audio_Next_fil);
	  f_read(&Audio_Next_fil, &Audio_Next_Buffer[0], AUDIO_BUFFER_SIZE*2, &br);
	  Track_Preloaded = 1;
}

void PlayTrack(I2S_HandleTypeDef *hi2s)
{
	// Moves Audio_Next_Buffer into Audio_DMA_Buffer, starts playback
	memcpy(Audio_DMA_Buffer, Audio_Next_Buffer, AUDIO_BUFFER_SIZE*2);
	Audio_Current_fil = Audio_Next_fil;
	HAL_I2S_Transmit_DMA (hi2s, Audio_DMA_Buffer, AUDIO_BUFFER_SIZE);
	LED_Current_Frame = 0;
	LoadLEDBuffer();

	Track_Playing = 1;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	// First half finished: refill first half while second half is playing
    f_read(&Audio_Current_fil,
           &Audio_DMA_Buffer[0],
           AUDIO_BUFFER_SIZE,
           &br);

    if(!br)
    {
    	StopTrack(&hi2s2);
    }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	// Second half finished: refill second half while first half is playing
    f_read(&Audio_Current_fil,
           &Audio_DMA_Buffer[AUDIO_BUFFER_SIZE/2],
           AUDIO_BUFFER_SIZE,
           &br);

    if(!br)
    {
    	StopTrack(&hi2s2);
    }
}

void Set_LED_Colour(uint32_t grb_value)
{
	// Set the whole WS2812 string to the same colour, a GRB word.

	for (i = 0; i < 24; i++)
	{
		// Since the LEDs expect MSB-first, we need to do some fiddling to fire
		// that out first.

		uint8_t grb_bit_offset = 23 - i;
		uint8_t grb_bit = (grb_value >> grb_bit_offset) & 1;

		for (uint32_t j = 0; j < LED_BYTES; j += 24)
			{
			if (grb_bit == 1)
			{
				LED_Framebuffer[i+j] = LED_ONE;
			}
			else
			{
				LED_Framebuffer[i+j] = LED_ZERO;
			}
		}
	}
}

void UpdateLCD(uint8_t Display_Number)
{
	// Calculate pinout for LCD and set pins. Flip-flop between multiple LCD quickly enough to avoid flicker.

	uint8_t digit = Track_Next;
	if (Display_Number == 1)
	{
		digit %= 10;
		HAL_GPIO_WritePin(LCD_DIG1CC_GPIO_Port, LCD_DIG1CC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_DIG2CC_GPIO_Port, LCD_DIG2CC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_DP_GPIO_Port, LCD_DP_Pin, Track_Playing);
	}
	else
	{
		digit /= 10;
		HAL_GPIO_WritePin(LCD_DIG1CC_GPIO_Port, LCD_DIG1CC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_DIG2CC_GPIO_Port, LCD_DIG2CC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCD_DP_GPIO_Port, LCD_DP_Pin, GPIO_PIN_RESET);
	}

	for (uint8_t i = 0; i < 8; i++)
	{
		uint8_t truth = (b7SegmentTable[digit] >> i) & 1;
		HAL_GPIO_WritePin(LCDs[i].port, LCDs[i].pin, truth);
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
	HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_DIG1CC_GPIO_Port, LCD_DIG1CC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_DIG2CC_GPIO_Port, LCD_DIG2CC_Pin, GPIO_PIN_SET);

	for (uint8_t i = 0; i < 8; i++)
	{
		uint8_t truth = (b7SegmentTable[10] >> i) & 1;
		HAL_GPIO_WritePin(LCDs[i].port, LCDs[i].pin, truth);
	}
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
