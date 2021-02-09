/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MIDI_OUT_Pin GPIO_PIN_6
#define MIDI_OUT_GPIO_Port GPIOA
#define WS2812_Pin GPIO_PIN_7
#define WS2812_GPIO_Port GPIOA
#define LCD_A_Pin GPIO_PIN_4
#define LCD_A_GPIO_Port GPIOC
#define LCD_B_Pin GPIO_PIN_5
#define LCD_B_GPIO_Port GPIOC
#define LCD_C_Pin GPIO_PIN_0
#define LCD_C_GPIO_Port GPIOB
#define LCD_D_Pin GPIO_PIN_1
#define LCD_D_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_7
#define LCD_E_GPIO_Port GPIOE
#define LCD_F_Pin GPIO_PIN_8
#define LCD_F_GPIO_Port GPIOE
#define LCD_G_Pin GPIO_PIN_9
#define LCD_G_GPIO_Port GPIOE
#define LCD_DP_Pin GPIO_PIN_10
#define LCD_DP_GPIO_Port GPIOE
#define LCD_DIG1CC_Pin GPIO_PIN_11
#define LCD_DIG1CC_GPIO_Port GPIOE
#define LCD_DIG2CC_Pin GPIO_PIN_12
#define LCD_DIG2CC_GPIO_Port GPIOE
#define B_PREV_Pin GPIO_PIN_13
#define B_PREV_GPIO_Port GPIOE
#define B_NEXT_Pin GPIO_PIN_14
#define B_NEXT_GPIO_Port GPIOE
#define B_STOP_Pin GPIO_PIN_15
#define B_STOP_GPIO_Port GPIOE
#define GPIO1_Pin GPIO_PIN_14
#define GPIO1_GPIO_Port GPIOB
#define GPIO2_Pin GPIO_PIN_15
#define GPIO2_GPIO_Port GPIOB
#define GPIO3_Pin GPIO_PIN_8
#define GPIO3_GPIO_Port GPIOD
#define GPIO4_Pin GPIO_PIN_9
#define GPIO4_GPIO_Port GPIOD
#define GPIO5_Pin GPIO_PIN_10
#define GPIO5_GPIO_Port GPIOD
#define GPIO6_Pin GPIO_PIN_11
#define GPIO6_GPIO_Port GPIOD
#define GPIO7_Pin GPIO_PIN_12
#define GPIO7_GPIO_Port GPIOD
#define GPIO8_Pin GPIO_PIN_13
#define GPIO8_GPIO_Port GPIOD
#define LED_STATUS_Pin GPIO_PIN_8
#define LED_STATUS_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_9
#define SDIO_CD_GPIO_Port GPIOA
#define TRIGGER_Pin GPIO_PIN_12
#define TRIGGER_GPIO_Port GPIOA
#define TRIGGER_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
