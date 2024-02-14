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


#define RED 		6
#define BLUE 		7
#define ORANGE	8
#define GREEN		9


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void TIM2_IRQHandler(void);
void GPIOInit(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  
  //HAL_Init();

  SystemClock_Config();

	GPIOInit();
	GPIOC->ODR |= (0b1 << GREEN); // Start with the green LED on
	
	RCC->APB1ENR |= 0b1; 			// Enable TIM2
	//RCC->APB1ENR |= 0b1 << 1; // Enable TIM3
	
	TIM2->PSC = 50; // Set the prescaler to 50 so the timer frequency is 160,000Hz
	TIM2->ARR = 200;    // 160,000Hz / 200 = 800Hz
	
	//TIM2->DIER |= 0b1; // Enables update interupt UIE
	
	//NVIC_EnableIRQ(TIM2_IRQn);
	
	//TIM2->CR1 |= 0b1;  // Enable the clock
	
	TIM2->CCMR1 &= ~(0b11 ); 			// Set CC2S to output
	TIM2->CCMR1 &= ~(0b11 << 8); 	// Set CC2S to output
	
	TIM2->CCMR1 |= 0b110 << 12;		// Set OC2M to output PWM mode 1
	TIM2->CCMR1 |= 0b111 << 4; 		// Set OC1M to output PWM mode 2

	TIM2->CCMR1 |= 0b1   << 3;		// Enable output compare preload channel 1
	TIM2->CCMR1 |= 0b1   << 11;   // Enable output compare preload channel 2
	
	TIM2->CCER |= 0b1; 			// Enable output for channel 1
	TIM2->CCER |= 0b1 << 4; // Enable output for channel 2
	
	TIM2->CCR1 = 200/5;
	TIM2->CCR2 = 200/5;
	
	
  while (1)
  {

  }

}

void TIM2_IRQHandler(void){

	// Toggle the Green and Orange LEDs
	GPIOC->ODR ^= (0b1 << GREEN);
	GPIOC->ODR ^= (0b1 << ORANGE);

	
	TIM2->SR &= ~(0b1); // Clear interrupt

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
