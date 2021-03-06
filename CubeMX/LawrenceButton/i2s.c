/* generic I2S example for any STM32duino HAL core F4
Original code by Rene Böllhoff
translated to STM32duino by Matthias Diro ("madias" -> STM32duino forum)
Features: Circular Buffer with DMA IRQ (half full, full), Master Clock enabled
This example uses the SPI3 port tested on STM32F407VET "black" board. On other boards please define LED0_BUILTIN and LED1_BUILTIN
*/
#define   I2S_BUFFER_SIZE    64
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;
uint32_t  dma_buffer  [I2S_BUFFER_SIZE];
// sinus oszillator
float     osc_phi     = 0;
float     osc_phi_inc = 440.0f / 44100.0f; // generating 440HZ
extern "C" void DMA1_Stream5_IRQHandler(void) // this function must be included to avoid DMA to crash! 
{
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
}
void setup() {
 // HAL_MspInit(); // not important by default
  HAL_I2S_MspInit(&hi2s3);// setting up pins and clocks; routing SPI3 to DMA
  MX_DMA_Init();
  MX_I2S3_Init(); 
  pinMode(LED0_BUILTIN, OUTPUT);
  pinMode(LED1_BUILTIN, OUTPUT);
  digitalWrite(LED0_BUILTIN, 0);
  StartAudioBuffers (&hi2s3);
  digitalWrite(LED0_BUILTIN, 1);
}
void loop() {
  // just a dummy code in loop, audio out is generated by ISR
  digitalWrite(LED1_BUILTIN, 1);
  delay(250);
  digitalWrite(LED1_BUILTIN, 0);
  delay(250);
}
void Error_Handler(byte errorcode) { // if something goes wrong counter the blinks for rudimentary debugging
  digitalWrite(LED0_BUILTIN, 1);
  while (1)
  {
    for (int x = 0; x < errorcode; x++) {
      digitalWrite(LED0_BUILTIN, 1);
      delay(100);
      digitalWrite(LED0_BUILTIN, 0);
      delay(100);
    }
    delay(1000);
  }
}
void FillBuffer (uint32_t *buffer, uint16_t len)
{
  float     a;
  int16_t   y;
  uint16_t  c;
  for (c = 0; c < len; c++)
  {
    // calculate sin
    a = (float) sin (osc_phi * 6.2832f) * 0.20f;
    osc_phi += osc_phi_inc;
    osc_phi -= (float) ((uint16_t) osc_phi);
    //   float to integer
    y = (int16_t) (a * 32767.0f);
    // auf beide kanäle
    buffer [c] =  ((uint32_t) (uint16_t) y) <<  0  |
                  ((uint32_t) (uint16_t) y) << 16;
  }
}
void StartAudioBuffers (I2S_HandleTypeDef *hi2s)
{
  // clear buffer
  memset (dma_buffer,0, sizeof (dma_buffer ));
  // start circular dma  
  HAL_I2S_Transmit_DMA (hi2s, (uint16_t *) dma_buffer, I2S_BUFFER_SIZE << 1);
}
extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  // second half finished, filling it up again while first  half is playing
  FillBuffer  (&(dma_buffer [I2S_BUFFER_SIZE  >> 1]), I2S_BUFFER_SIZE >> 1);
}
extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  // first half finished, filling it up again while second half is playing
  FillBuffer  (&(dma_buffer [0]), I2S_BUFFER_SIZE >> 1);
}
// setting up I2S
extern "C" void MX_I2S3_Init(void)
{
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  //hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler(1); // on error: one blink
  }
}
// setting up DMA
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}
// setting up pins and clocks; routing SPI3 to DMA
extern "C"  void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s )
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI3_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
/* I2S standard configurations:
  SPI2
  PB15 DIN
  PB12 LRC
  PB13 SCLK
  PC6 MCK
  SPI3
  PB5 DIN
  PA4 LRC
  PB3 SCLK
  PC7 MCK
*/
  //I2S3 used GPIO configuration in this example:
  // PB5 DIN / SD
  // PA4 LRC /WD
  // PB3 SCLK /CK
  // PC7 MCK
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // master clock
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate =  GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  // Peripheral DMA init
  hdma_spi3_tx.Instance = DMA1_Stream5;
  hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
  hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_spi3_tx.Init.Mode = DMA_CIRCULAR;
  hdma_spi3_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
  {
    Error_Handler(5); // on error: five blinks
  }
  __HAL_LINKDMA(hi2s, hdmatx, hdma_spi3_tx);
}
extern "C"  void  HAL_MspInit(void) // maybe useful, not included in this example
{
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}