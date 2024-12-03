/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void AD9833_Write(uint16_t data);
void AD9833_SetFrequencyAndMode(uint32_t frequency, uint8_t phase, uint16_t mode);
void update_screen(void);
void handle_buttons(void);
void loopback_test(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USR_LED_Pin GPIO_PIN_13
#define USR_LED_GPIO_Port GPIOC
#define AD9833_SS_Pin GPIO_PIN_4
#define AD9833_SS_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_12
#define CLK_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_13
#define MOSI_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_14
#define CS_GPIO_Port GPIOB
#define PREV_MODE_Pin GPIO_PIN_8
#define PREV_MODE_GPIO_Port GPIOA
#define NEXT_MODE_Pin GPIO_PIN_9
#define NEXT_MODE_GPIO_Port GPIOA
#define PREV_FREQ_Pin GPIO_PIN_10
#define PREV_FREQ_GPIO_Port GPIOA
#define NEXT_FREQ_Pin GPIO_PIN_11
#define NEXT_FREQ_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define BUTTONS_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
