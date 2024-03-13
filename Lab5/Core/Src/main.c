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

#define ADDR 0x69
#define WHO_AM_I_REG 0x0f
#define WHO_AM_I 		 0b11010011

#define RED 		6
#define BLUE 		7
#define ORANGE	8
#define GREEN		9

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void LEDSet(uint16_t LED, uint8_t value);
void GPIOInit(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();


  SystemClock_Config();

	GPIOInit();


	// Initilize I2C2
	RCC->APB1ENR |= 0b1 << 22; // Enable I2C2 Clock
	I2C2->TIMINGR |= (0x1 << 28)|(0x4 << 20)|(0x2 << 16)|(0xF << 8)|0x13;
	I2C2->CR1 |= 0b1;
	
	
	// Start a transaction
	I2C2->CR2 |= ADDR << 1; // Set address to transmit to
	I2C2->CR2 |= 1 << 16;   // Set the number of bytes to transmit
	I2C2->CR2 &= ~(0b1 << 10);		// Set the transfer to write
	I2C2->CR2 |= 0b1 << 13; // Start the transfer
	
	uint32_t val = 0;
	
	// Wait for TXIS or NACKF
	while((val & 0b10010) == 0){
		val = I2C2->ISR;
	}
	
	if((val & 0b10000) != 0){
		LEDSet(RED, 1);
	}
	
	if((val & 0b10) != 0){
		LEDSet(GREEN, 1);
	}

	
	
	// Write the WHO_AM_I address
	I2C2->TXDR |= WHO_AM_I_REG;
	//I2C2->CR2 |= 0b1 << 13; // Start the transfer
	
	// Wait for TD flag
	while((I2C2->ISR & (0b1 <<6)) == 0){
		__NOP();
	}
	
	
	
	// Start a transaction
	I2C2->CR2 |= ADDR << 1; // Set address to transmit to
	I2C2->CR2 |= 1 << 16;   // Set the number of bytes to transmit
	I2C2->CR2 |= 0b1 << 10;				// Set the transfer to read
	I2C2->CR2 |= 0b1 << 13; // Start the transfer
	
	val = 0;
	
	// Wait for RXNE or NACKF
	while((val & 0b10100) == 0){
		val = I2C2->ISR;
	}
	
	if((val & 0b10000) != 0){
		LEDSet(RED, 1);
	}
	
	if((val & 0b100) != 0){
		LEDSet(GREEN, 1);
	}
	
	// Wait for TD flag
	while((I2C2->ISR & (0b1 <<6)) == 0){
		__NOP();
	}
	
	// Check the value that was read
	uint8_t read_val = I2C2->RXDR;
	
	// Check the value read from the WHO_AM_I register
	if(read_val == WHO_AM_I){
		LEDSet(BLUE, 1);
	}else{
		LEDSet(ORANGE, 1);
	}
	
	
  while (1)
  {

  }

}



/**
  * @brief Configure the GPIO pins for the LEDs and user button
  * @retval None
  * 
  * PC6 = Red Led
  * PC7 = Blue Led
  * PC8 = Orange Led
  * PC9 = Green Led
  * PA0 = User Button
  *
  */
void GPIOInit(void){
	

	
	//Enable the peripheral clock for GPIO port A
	RCC->AHBENR |= 0b1<<17;
	
	//Enable the peripheral clock for GPIO port B
	RCC->AHBENR |= 0b1<<18;

	//Enable the peripheral clock for GPIO port C
	RCC->AHBENR |= 0b1<<19; 	
	
	// Set PC0, PC6, PC7, PC8, PC9 to general purpose output
	GPIOC->MODER |= 0b01;
	GPIOC->MODER |= 0b01 << (6 * 2);
	GPIOC->MODER |= 0b01 << (7 * 2);
	GPIOC->MODER |= 0b01 << (8 * 2);
	GPIOC->MODER |= 0b01 << (9 * 2);
	
	// Set PA0 to general purpose input
	GPIOA->MODER &= ~(0b11);
	
	// Set up PB11 and PB13 for I2C
	GPIOB->MODER |= 0b10 << 22;
	GPIOB->MODER |= 0b10 << 26;
	
	GPIOB->OTYPER |= 0b1 << 11;
	GPIOB->OTYPER |= 0b1 << 13;
	
	GPIOB->AFR[1] |= 0b0001 << 12; // PB11 SDA
	GPIOB->AFR[1] |= 0b0101 << 20; // PB13 SCL
	
	// Set PB14 to pushpull output
	GPIOB->MODER |= 0b01 << (2 * 14);
	
	// Set PB14 and PC0 on
	GPIOB->ODR |= (0b1 << 14); 
	GPIOC->ODR |= 0b1;
	
	// Set the output type to push pull for GPIO port c
	GPIOC->OTYPER  = 0x00000000;
	
	// Set the speed to slow for GPIO port c
	GPIOC->OSPEEDR = 0x00000000; 
	
	// Set the speed to slow for GPIO PA0
	GPIOA->OSPEEDR &= ~(0b11); 	
	
	// Set the pullup and pulldown to off for GPIO port c
	GPIOC->PUPDR = 0x00000000; 

	// Set PA0 to pulldown
	GPIOA->PUPDR |= 0b10;
	
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

/**
  * @brief The state of an LED
  * @retval None
	* @param LED: which LED to set
  * @param value: value to set the LED 
  */
void LEDSet(uint16_t LED, uint8_t value){
	if(value){
		GPIOC->ODR |= 0b1 << LED;
	}else{
		GPIOC->ODR &= ~(0b1 << LED);
	}
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
