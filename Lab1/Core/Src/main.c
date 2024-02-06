/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ECE 5780 Lab 1
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
  * ECE 5780 Lab 1 
	* Owen Leishman
	* 2/5/2024
	*
	*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Defines -------------------------------------------------------------------*/
#define part 1 // Which part of the lab to run (1 or 2)


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GPIOInit(void);
void LEDToggle(void);
uint8_t ButtonRead(void);




/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	
	SystemClock_Config(); //Configure the system clock
	
	GPIOInit(); // Configures the LEDs and the User Button
	
	GPIOC->ODR |= (0b1 << 6); // Turn on the Red led to start

	uint32_t debouncer = 0;
	
	while (1) {
		
		// Run the code for part 1 if selected
		if(part == 1){
			
			HAL_Delay(200); // Delay 200ms
			LEDToggle(); // Toggle LEDs
		
		}
		
		// Run the code for part 2 if selected
		if(part == 2){
		
			debouncer = (debouncer << 1); // Always shift every loop iteration			

			if (ButtonRead()) { // If input signal is set/high
				debouncer |= 0x01; // Set lowest bit of bit-vector
			}

			// Runs only once when the button state is changing from low to high
			if (debouncer == 0x7FFFFFFF) {
				LEDToggle(); //Toggle Leds
			}		

		}
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
	
	//Enable the peripheral clock for GPIO port C
	RCC->AHBENR |= 0b1<<19; 
	
	//Enable the peripheral clock for GPIO port A
	RCC->AHBENR |= 0b1<<17;
	
	// Set PC6, PC7, PC8, PC9 to general purpose output
	GPIOC->MODER |= 0b01 << (6 * 2);
	GPIOC->MODER |= 0b01 << (7 * 2);
	GPIOC->MODER |= 0b01 << (8 * 2);
	GPIOC->MODER |= 0b01 << (9 * 2);
	
	// Set PA0 to general purpose input
	GPIOA->MODER &= ~(0b11);
	
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
  * @brief Toggle the state of the onboard Red and Blue LEDs
  * @retval None
  */
void LEDToggle(void){
		GPIOC->ODR ^= (0b1 <<6);
		GPIOC->ODR ^= (0b1 <<7);
}

/**
  * @brief Read the value of the User Button
  * @retval Button State
  */
uint8_t ButtonRead(void){
	return GPIOA->IDR & 0b1;
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
