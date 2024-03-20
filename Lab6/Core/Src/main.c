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
#define ORANGE		8
#define GREEN		9

#define THRESHOLD1 50
#define THRESHOLD2 100
#define THRESHOLD3 150
#define THRESHOLD4 200

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GPIOInit(void);
void LEDSet(uint16_t LED, uint8_t value);
void ADCInit(void);
uint8_t ADCRead(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();

  SystemClock_Config();

	GPIOInit();
	ADCInit();
	
	
	uint8_t value = 0;
  while (1)
  {
		value = ADCRead();
		
		if(value > THRESHOLD1){
			LEDSet(RED, 1);
		}else{
			LEDSet(RED, 0);
		}

		if(value > THRESHOLD2){
			LEDSet(ORANGE, 1);
		}else{
			LEDSet(ORANGE, 0);
		}
		
		if(value > THRESHOLD3){
			LEDSet(BLUE, 1);
		}else{
			LEDSet(BLUE, 0);
		}

		if(value > THRESHOLD4){
			LEDSet(GREEN, 1);
		}else{
			LEDSet(GREEN, 0);
		}
		

		
  }

}

uint8_t ADCRead(void){
	return ADC1->DR;
}

void ADCInit(void){
	
	// Enable ADC1 clock
	RCC->APB2ENR |= 0b1 << 9;
	
	ADC1->CFGR1 |= 0b1 << 13; // set to continous conversion mode
	ADC1->CFGR1 &= ~(0b11 << 10); // turn off external trigger
	ADC1->CFGR1 |= 0b10 << 3;	// set resolution to 8 bits
	ADC1->CHSELR |= 0b1 << 10; // select channel 10
	
	// Start Calibration
	ADC1->CFGR1 |= 0b1 << 31; // start calibration
	
	// Wait until the calibration is complete
	while((ADC1->CFGR1 & (0b1 << 31)) != 0){
		__NOP();
	}
	
	// Enable the ADC
	ADC1->CR |= 0b1;
	
	// Start the analog conversions
	ADC1->CR |= 0b1 << 2;
	
}

/**
  * @brief Configure the GPIO pins for the LEDs and user button
  * @retval None
  * 
  * PC6 = Red Led
  * PC7 = Blue Led
  * PC8 = Orange Led
  * PC9 = Green Led
  *
  */
void GPIOInit(void){
	
	//Enable the peripheral clock for GPIO port C
	RCC->AHBENR |= 0b1<<19; 
	
	// Set PC6, PC7, PC8, PC9 to general purpose output
	GPIOC->MODER |= 0b01 << (6 * 2);
	GPIOC->MODER |= 0b01 << (7 * 2);
	GPIOC->MODER |= 0b01 << (8 * 2);
	GPIOC->MODER |= 0b01 << (9 * 2);
	
	// Set PC0 to analog mode
	GPIOC->MODER |= 0b11;
	
	// Set PA0 to general purpose input
	GPIOA->MODER &= ~(0b11);
	
	// Set the output type to push pull for GPIO port c
	GPIOC->OTYPER  = 0x00000000;
	
	// Set the speed to slow for GPIO port c
	GPIOC->OSPEEDR = 0x00000000; 
	
	// Set the pullup and pulldown to off for GPIO port c
	GPIOC->PUPDR = 0x00000000; 

	
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
