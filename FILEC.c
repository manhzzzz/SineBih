/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "ssd1306_conf_template.h"
#include "stdio.h"
#include "string.h"
#include "max30102_for_stm32_hal.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;


/* USER CODE BEGIN PV */

uint32_t red_val, ir_val;
char uartBuf[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
max30102_t max30102;
void MAX30102_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, MAX30102_I2C_ADDR, reg,
                      I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

// Đọc 1 byte từ thanh ghi
uint8_t MAX30102_ReadReg(uint8_t reg)
{
    uint8_t value;
    HAL_I2C_Mem_Read(&hi2c1, MAX30102_I2C_ADDR, reg,
                     I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    return value;
}

// Đọc FIFO (6 byte: RED và IR)
void MAX30102_ReadFIFO(uint8_t *buffer)
{
    HAL_I2C_Mem_Read(&hi2c1, MAX30102_I2C_ADDR, 0x07,
                     I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY);
}

// Khởi tạo MAX30102
void MAX30102_Init(void)
{
    // Reset
    MAX30102_WriteReg(0x09, 0x40);
    HAL_Delay(100);

    // FIFO config: sample avg = 4, rollover enable
    MAX30102_WriteReg(0x08, 0x4F);

    // Mode config: SpO2 mode (0x03)
    MAX30102_WriteReg(0x09, 0x03);

    // SpO2 config: 0x27 = 100Hz, 18-bit, 4096nA
    MAX30102_WriteReg(0x0A, 0x27);

    // LED amplitudes: RED & IR ~7mA
    MAX30102_WriteReg(0x0C, 0x24); // LED1_RED
    MAX30102_WriteReg(0x0D, 0x24); // LED2_IR
}

// Đọc dữ liệu raw (RED, IR)
void MAX30102_ReadRawData(uint32_t *red_led, uint32_t *ir_led)
{
    uint8_t buffer[6];
    MAX30102_ReadFIFO(buffer);

    *red_led = ((uint32_t)(buffer[0] & 0x03) << 16) |
               ((uint32_t)buffer[1] << 8) | buffer[2];
    *ir_led  = ((uint32_t)(buffer[3] & 0x03) << 16) |
               ((uint32_t)buffer[4] << 8) | buffer[5];
}

void shutdown()
{
	ssd1306_SetDisplayOn(0);
   max30102_shutdown(&max30102, 1);
    HAL_Delay(100);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnterSTANDBYMode();
}
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  ssd1306_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_PWR_CLK_ENABLE();

     if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
     {
         __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
     }
     while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);

  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
  printf("STM32 + MAX30102 Init...\r\n");

  // Khởi tạo MAX30102
  max30102_init(&max30102, &hi2c1);
  max30102_reset(&max30102);

  // Cấu hình chế độ SpO2
  max30102_set_mode(&max30102, max30102_spo2);

  // FIFO: trung bình 4 mẫu, rollover enable, FIFO full ngưỡng 15
  max30102_set_fifo_config(&max30102, max30102_smp_ave_4, 1, 0x0F);

  // LED: IR và RED ~7mA
  max30102_set_led_current_1(&max30102, 7.0f);
  max30102_set_led_current_2(&max30102, 7.0f);

  printf("MAX30102 Ready!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  max30102_read_fifo(&max30102);

	        uint32_t ir  = max30102._ir_samples[0];
	        uint32_t red = max30102._red_samples[0];

	        // Gửi qua UART
	        sprintf(uartBuf, "IR: %lu | RED: %lu\r\n", ir, red);
	        HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);

	        // Hiển thị OLED
	        ssd1306_Fill(Black);
	        ssd1306_SetCursor(2, 10);
	        ssd1306_WriteString("IR:", Font_7x10, White);
	        sprintf(uartBuf, "%lu", ir);
	        ssd1306_SetCursor(40, 10);
	        ssd1306_WriteString(uartBuf, Font_7x10, White);

	        ssd1306_SetCursor(2, 30);
	        ssd1306_WriteString("RED:", Font_7x10, White);
	        sprintf(uartBuf, "%lu", red);
	        ssd1306_SetCursor(40, 30);
	        ssd1306_WriteString(uartBuf, Font_7x10, White);

	        ssd1306_UpdateScreen();

	        HAL_Delay(200);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
	  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);
	    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  {}
  }
  /* USER CODE END Error_Handler_Debug */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
