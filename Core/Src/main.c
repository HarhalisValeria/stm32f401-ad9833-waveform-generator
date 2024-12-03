/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define F_MCLK 25000000

#define CONTROL_SINE    0x2000  // Command for FREQ0, Sine Wave (Mode = 0)
#define CONTROL_TRIANGLE 0x2002  // Command for FREQ0, Triangle Wave (Mode = 1)
#define CONTROL_SQUARE  0x2028  // Command for FREQ0, Square Wave (Mode = 0, OPBITEN = 1, DIV/2 = 1)
// For reset set second nibble to 1

#define WRITE_RESET		0x2100 // Command reset bits and write 16bit words
#define CLEAN_RESET 	0x2000 // Clear reset bit

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Waveform modes
const char *modes[] = {"Sine", "Triangle", "Meander"};
const uint16_t mode_commands[] = {CONTROL_SINE, CONTROL_TRIANGLE, CONTROL_SQUARE};
const uint8_t num_modes = sizeof(modes) / sizeof(modes[0]);
uint8_t current_mode = 0;

// Frequencies in Hz
const char *freq_str[] = {"1 Hz", "10 Hz", "100 Hz", "1 000 Hz", "10 000 Hz", "100 000 Hz", "1 000 000 Hz"};
const uint32_t frequencies[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
const uint8_t num_frequencies = sizeof(frequencies) / sizeof(frequencies[0]);
uint8_t current_frequency = 3;

#define DEBOUNCE_DELAY 200
#define SEND_DELAY 3000
uint32_t last_button_press = 0;

typedef uint8_t bool;
#define true 1
#define false 0

bool data_sent = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  update_screen();
  AD9833_SetFrequencyAndMode(frequencies[current_frequency], 60, mode_commands[current_mode]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  handle_buttons();
	  if ((HAL_GetTick() - last_button_press >= SEND_DELAY) && !data_sent) {
		  AD9833_SetFrequencyAndMode(frequencies[current_frequency], 60, mode_commands[current_mode]);
		  data_sent = true;  // Mark data as sent to prevent repeated transmission

		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(0, 30);
		  ssd1306_WriteString("Config changed", Font_7x10, White);
		  ssd1306_UpdateScreen();
		  HAL_Delay(1000);

		  update_screen();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AD9833_SS_GPIO_Port, AD9833_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_Pin|MOSI_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USR_LED_Pin */
  GPIO_InitStruct.Pin = USR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USR_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AD9833_SS_Pin */
  GPIO_InitStruct.Pin = AD9833_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD9833_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Pin MOSI_Pin CS_Pin */
  GPIO_InitStruct.Pin = CLK_Pin|MOSI_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PREV_MODE_Pin NEXT_MODE_Pin PREV_FREQ_Pin NEXT_FREQ_Pin */
  GPIO_InitStruct.Pin = PREV_MODE_Pin|NEXT_MODE_Pin|PREV_FREQ_Pin|NEXT_FREQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void AD9833_Write(uint16_t data) {
    // Transmit 16-bit data
	  GPIOB->BSRR = CLK_Pin;
	  GPIOB->BSRR = CS_Pin<<16;
	  for(uint8_t i=0;i<16;i++){
	    if(data & 0x8000){GPIOB->BSRR = MOSI_Pin;}
	    else{GPIOB->BSRR = MOSI_Pin<<16;}

	    GPIOB->BSRR = CLK_Pin<<16;
	    data = data<<1;
	    if(data & 0x8000){}
	    GPIOB->BSRR = CLK_Pin;
	  }
	  GPIOB->BSRR = CS_Pin;
	  GPIOB->BSRR = MOSI_Pin<<16;
}

void AD9833_SetFrequencyAndMode(uint32_t frequency, uint8_t phase, uint16_t mode) {
	uint64_t freq_word = (uint64_t)((double)frequency / F_MCLK * (1ULL << 28));

    uint16_t freq_msb = (uint16_t)((freq_word & 0xFFFC000) >> 14);  // Upper 14 bits (MSB)
    uint16_t freq_lsb = (uint16_t)(freq_word & 0x3FFF);  // Lower 14 bits (LSB)

    // 1. Write the control command
    AD9833_Write(mode);

    // 2. Write to FREQ0 LSB
    uint16_t command_lsb = 0x4000 | freq_lsb; // 0x4000 selects FREQ0, LSB
    AD9833_Write(command_lsb);  // Send LSB

    // 3. Write to FREQ0 MSB
    uint16_t command_msb = 0x4000 | freq_msb; // 0x4000 selects FREQ0, MSB
    AD9833_Write(command_msb);  // Send MSB

    uint16_t phase_word = (phase*(4096/360))|0xC000;

	// 4. Write to PHASE0
	AD9833_Write(phase_word);  // Send LSB
}

void update_screen(void) {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Waveform:", Font_7x10, White);
  ssd1306_WriteString(modes[current_mode], Font_7x10, White);

  ssd1306_SetCursor(0, 30);
  ssd1306_WriteString("Freq:", Font_7x10, White);
  ssd1306_WriteString(freq_str[current_frequency], Font_7x10, White);

  ssd1306_UpdateScreen();
}

void handle_buttons(void) {
	uint32_t current_time = HAL_GetTick();

	 if ((current_time - last_button_press) < DEBOUNCE_DELAY) {
		 return;
	 }

	 if (HAL_GPIO_ReadPin(BUTTONS_Port, PREV_MODE_Pin) == GPIO_PIN_SET) {
		 current_mode = (current_mode == 0) ? num_modes - 1 : current_mode - 1;
		 update_screen();
		 last_button_press = current_time;
		 data_sent = false;
	 } else if (HAL_GPIO_ReadPin(BUTTONS_Port, NEXT_MODE_Pin) == GPIO_PIN_SET) {
		 current_mode = (current_mode + 1) % num_modes;
		 update_screen();
		 last_button_press = current_time;
		 data_sent = false;
	 } else if (HAL_GPIO_ReadPin(BUTTONS_Port, PREV_FREQ_Pin) == GPIO_PIN_SET) {
		 if (current_frequency == 0) return;
		 current_frequency = current_frequency - 1;
		 update_screen();
		 last_button_press = current_time;
		 data_sent = false;
	 } else if (HAL_GPIO_ReadPin(BUTTONS_Port, NEXT_FREQ_Pin) == GPIO_PIN_SET) {
		 if (current_frequency == num_frequencies - 1) return;
		 current_frequency = current_frequency + 1;
		 update_screen();
		 last_button_press = current_time;
		 data_sent = false;
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
