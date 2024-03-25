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

#include "main.h"

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// Enable GPIOB and GPIOC in the RCC.
	RCC->AHBENR  |= (RCC_AHBENR_GPIOBEN)|(RCC_AHBENR_GPIOCEN); // Enables the GPIOB/GPIOC clock in the RCC.
	// Enable the I2C2 peripheral in the RCC.
	RCC->APB1ENR |= (RCC_APB1ENR_I2C2EN);
	
	// Set PB11 to alternate function mode, open-drain output type, and select I2C2_SDA as its alternate function.
	GPIOB->MODER |= (1 << 23);  // Set PB11 to alternate mode. (1s)
	GPIOB->MODER &= ~(1 << 22); // Set PB11 to alternate mode. (0s)
	GPIOB->OTYPER |= (1 << 11); // Set PB11 to open-drain.
	GPIOB->AFR[1] |= (1 << 12); // Set PB11 to AF1 I2C2_SDA. (1s)
	GPIOB-> AFR[1] &= ~((1 << 13)|(1 << 14)|(1 << 15)); // Set PB11 to AF1 I2C2_SDA. (0s)
	
	// Set PB13 to alternate function mode, open-drain output type, and select I2C2_SCL as its alternate function.
	GPIOB->MODER |= (1 << 27);  // Set PB13 to alternate mode. (1s)
	GPIOB->MODER &= ~(1 << 26); // Set PB13 to alternate mode. (0s)
	GPIOB->OTYPER |= (1 << 13); // Set PB13 to open-drain.
	GPIOB->AFR[1] |= (1 << 20)|(1 << 22); // Set PB13 to AF1 I2C2_SCL. (1s)
	GPIOB-> AFR[1] &= ~((1 << 21)|(1 << 23)); // Set PB13 to AF1 I2C2_SCL. (0s)
	
	// Set PB14 to output mode, push-pull output type, and initialize/set the pin high.
	GPIOB->MODER |= (1 << 28);  // Set PB14 to output mode. (1s)
	GPIOB->MODER &= ~(1 << 29); // Set PB14 to output mode. (0s)
	GPIOB->OTYPER &= ~(1 << 14); // Set PB14 to push-pull.
	GPIOB->BSRR |= (1 << 14); // Initialize PB14 to high.
	
	// Set PC0 to output mode, push-pull output type, and initialize/set the pin high.
	GPIOC->MODER |= (1 << 0); // Set PC0 to output mode. (1s)
	GPIOC->MODER &= ~(1 << 1); // Set PC0 to output mode. (0s)
	GPIOC->OTYPER &= ~(1 << 0); // Set PC0 to push-pull.
	GPIOC->BSRR |= (1 << 0); // Initialize PC0 to high.
	
	// Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C.
	I2C2->TIMINGR |= (0x13 << 0) | (0x0F << 8) | (0x2 << 16) | (0x4 << 20) | (0x1 << 28);
	// Enable the I2C peripheral using the PE bit in the CR1 register.
	I2C2->CR1 |= (1 << 0);
	
  while (1)
  {
		
  }
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
