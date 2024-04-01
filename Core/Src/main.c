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
void setRED(char value);
void toggleRED(void);
void setBLUE(char value);
void toggleBLUE(void);
void setGREEN(char value);
void toggleGREEN(void);
void setORANGE(char value);
void toggleORANGE(void);
char READ_I2C(char reg);
void WRITE_I2C(char reg, char val);

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
	
	// Configures GPIOC Pins 6 and 7 (RED LED and BLUE LED)
	GPIOC->MODER   |=  (1 << 12) | (1 << 14);
	GPIOC->OTYPER  &= ~((1 << 6) | (1 << 7));
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14));
	GPIOC->PUPDR   &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15));
	
	// Configures GPIOC Pins 8 and 9 (ORANGE LED and GREEN LED)
	GPIOC->MODER   |=  (1 << 16) | (1 << 18);
	GPIOC->OTYPER  &= ~((1 << 8) | (1 << 9));
	GPIOC->OSPEEDR &= ~((1 << 16) | (1 << 18));
	GPIOC->PUPDR   &= ~((1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	
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
	
	
	/* Clear the NBYTES and SADD bit fields
	* The NBYTES field begins at bit 16, the SADD at bit 0
	*/
	//I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	/* Set NBYTES = 42 and SADD = 0x14
	* Can use hex or decimal values directly as bitmasks.
	* Remember that for 7-bit addresses, the lowest SADD bit
	* is not used and the mask must be shifted by one.
	*/
	//I2C2->CR2 |= (1 << 16) | (0x69 << 1);
	
	// Set to write.
	//I2C2->CR2 |= (1 << 10);
	// Set start bit to 1.
	//I2C2->CR2 |= (1 << 13);
	int test = 0;
	
	char OUT_X_L = 0x00;
	char OUT_X_H = 0x00;
	char OUT_Y_L = 0x00;
	char OUT_Y_H = 0x00;
	
	int x_val = 0;
	int y_val = 0;
	
	WRITE_I2C(0x20, 0x0F);
	
  while (1)
  {
		OUT_X_L = READ_I2C(0x28);
		OUT_X_H = READ_I2C(0x29);
		x_val = (OUT_X_H << 8) + OUT_X_L; 
		OUT_Y_L = READ_I2C(0x2A);
		OUT_Y_H = READ_I2C(0x2B);
		y_val = (OUT_Y_H << 8) + OUT_Y_L; 
		
		if(y_val < 32767 && y_val > 2000){
			setRED('1');
			setBLUE('0');
		}
		else if(y_val > 32767 && y_val < 63535) {
			setRED('0');
			setBLUE('1');
		}
		
		if(x_val < 32767 && x_val > 2000){
			setORANGE('0');
			setGREEN('1');
		}
		else if(x_val > 32767 && x_val < 63535){
			setORANGE('1');
			setGREEN('0');
		}
		
		HAL_Delay(100);
	}
}

/* READ FROM GYRO I2C */
char READ_I2C(char reg){
	HAL_Delay(5);
	
	char RETURNVALUE = 0x00;
	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); // Clears NBYTES and SADD bit fields and sets them all to zero.
	I2C2->CR2 |= (1 << 16) | (0x69 << 1); // Sets NBYTES = 1 and SADD to 0x69 or the addr. of the slave device(gyro).
	
	I2C2->CR2 &= ~(1 << 10); // Set to write.
	I2C2->CR2 |= (1 << 13); // Set start bit to 1.
	
	while( !((I2C2->ISR & 0x2) | (I2C2->ISR & 0x10))){} // 0x2 = TXIS and 0x10 = NACKF.
	
	if(I2C2->ISR & 0x10){ // NACKF FLAG
		//toggleRED();
	}
	else if(I2C2->ISR & 0x2) { // TXIS FLAG	
		I2C2->TXDR = reg;
	}
	
	while( !(I2C2->ISR & 0x40) ) {
	}
			
	if(I2C2->ISR & 0x40){	
		//SET UP FOR READ OPERATION
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); // Clears NBYTES and SADD bit fields and sets them all to zero.
		I2C2->CR2 |= (1 << 16) | (0x69 << 1); // Sets NBYTES = 1 and SADD to 0x69 or the addr. of the slave device(gyro).
		I2C2->CR2 |= (1 << 10); // Set to read.
		I2C2->CR2 |= (1 << 13); // Set start bit to 1 for RESTART.
	}
	
	while( !((I2C2->ISR & 0x4) | (I2C2->ISR & 0x10))){} // WAITS FOR RXNE or NACKF to set.
		
	if (I2C2->ISR & 0x10){ // NACKF FLAG
		//toggleRED();
	}
	else if(I2C2->ISR & 0x4){ // RXNE FLAG
	}
	
	while( !(I2C2->ISR & 0x40) ) {}
		
	if(I2C2->ISR & 0x40){
		
		RETURNVALUE = I2C2->RXDR;
		
		I2C2->CR2 |= (1 << 14); // STOPS Transmission
	}
	return RETURNVALUE;
}

/* READ FROM GYRO I2C */
void WRITE_I2C(char reg, char val){
	HAL_Delay(5);

	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); // Clears NBYTES and SADD bit fields and sets them all to zero.
	I2C2->CR2 |= (2 << 16) | (0x69 << 1); // Sets NBYTES = 2 and SADD to 0x69 or the addr. of the slave device(gyro).
	
	I2C2->CR2 &= ~(1 << 10); // Set to write.
	I2C2->CR2 |= (1 << 13); // Set start bit to 1.
	
	while( !((I2C2->ISR & 0x2) | (I2C2->ISR & 0x10))){} // 0x2 = TXIS and 0x10 = NACKF.
	
	if(I2C2->ISR & 0x10){ // NACKF FLAG
		//toggleRED();
	}
	else if(I2C2->ISR & 0x2) { // TXIS FLAG	
		I2C2->TXDR = reg;
	}
	
	while( !((I2C2->ISR & 0x2) | (I2C2->ISR & 0x10))){} // 0x2 = TXIS and 0x10 = NACKF.
	
	if(I2C2->ISR & 0x10){ // NACKF FLAG
		//toggleRED();
	}
	else if(I2C2->ISR & 0x2) { // TXIS FLAG	
		I2C2->TXDR = val;
	}
	
	while( !(I2C2->ISR & 0x40) ) {}
			
	if(I2C2->ISR & 0x40){	
		I2C2->CR2 |= (1 << 14); // STOPS Transmission
	}
	
}

void toggleRED(void){
	// Toggle Pin PC6 (RED).
	if(GPIOC->IDR & 0x40){
		GPIOC->BSRR |= (1 << 22); // Resets State of PC6.
	}
	else{
		GPIOC->BSRR |= (1 << 6); // Sets State of PC6.
	}
}

/**
	* @brief Sets RED LED.
	* @retval None
	*/
void setRED(char value){
	if(value == '1'){
		GPIOC->BSRR |= (1 << 6); // Sets State of PC6.
	}
	else if(value == '0'){
		GPIOC->BSRR |= (1 << 22); // Resets State of PC6.
	}
}

/**
	* @brief Toggles BLUE LED.
	* @retval None
	*/
void toggleBLUE(void){
	// Toggle Pin PC7 (BLUE).
	if(GPIOC->IDR & 0x80){
		GPIOC->BSRR |= (1 << 23); // Resets State of PC7.
	}
	else{
		GPIOC->BSRR |= (1 << 7); // Sets State of PC7.
	}
}

/**
	* @brief Sets BLUE LED.
	* @retval None
	*/
void setBLUE(char value){
	if(value == '1'){
		GPIOC->BSRR |= (1 << 7); // Sets State of PC7.
	}
	else if(value == '0'){
		GPIOC->BSRR |= (1 << 23); // Resets State of PC7.
	}
}

/**
	* @brief Toggles ORANGE LED.
	* @retval None
	*/
void toggleORANGE(void){
	// Toggle Pin PC8 (ORANGE).
	if(GPIOC->IDR & 0x100){
		GPIOC->BSRR |= (1 << 24); // Resets State of PC8.
	}
	else{
		GPIOC->BSRR |= (1 << 8); // Sets State of PC8.
	}
}

/**
	* @brief Sets ORANGE LED.
	* @retval None
	*/
void setORANGE(char value){
	if(value == '1'){
		GPIOC->BSRR |= (1 << 8); // Sets State of PC8.
	}
	else if(value == '0'){
		GPIOC->BSRR |= (1 << 24); // Resets State of PC8.
	}
}

/**
	* @brief Toggles GREEN LED.
	* @retval None
	*/
void toggleGREEN(void){
	// Toggle Pin PC9 (GREEN).
	if(GPIOC->IDR & 0x200){
		GPIOC->BSRR |= (1 << 25); // Resets State of PC9.
	}
	else{
		GPIOC->BSRR |= (1 << 9); // Sets State of PC9.
	}
}

/**
	* @brief Sets GREEN LED.
	* @retval None
	*/
void setGREEN(char value){
	if(value == '1'){
		GPIOC->BSRR |= (1 << 9); // Sets State of PC9.
	}
	else if(value == '0'){
		GPIOC->BSRR |= (1 << 25); // Resets State of PC9.
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
