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

#define BAUD_RATE 115200

#define RED 		6
#define BLUE 		7
#define ORANGE	8
#define GREEN		9


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GPIOInit(void);
void LEDTogle(uint16_t LED);
void USARTTransmitBlocking(uint8_t data);
void USARTTransmitBlockingString(uint8_t *data);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	
	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();

	GPIOInit();
	
	// TX PB10
	// RX PB11
	
	
	RCC->AHBENR  |= 0b1<<18; // Enable the clock for IO port B
	RCC->APB1ENR |= 0b1<<18; // Enable the clock for USART3
	
	GPIOB->MODER |= 0b10<<20; // Set PB10 to alternate function
	GPIOB->MODER |= 0b10<<22; // Set PB11 to alternate function
	
	GPIOB->AFR[1] |= 0b0100<<8;  // Set PB10 alternate function to AF4
	GPIOB->AFR[1] |= 0b0100<<12; // Set PB11 alternate function to AF4
	
	GPIOB->OTYPER  = 0x00000000;
	
	uint32_t baud_divider = HAL_RCC_GetHCLKFreq()/BAUD_RATE; // Calculate baud rate
	//uint32_t baud_divider = HAL_RCC_GetSysClockFreq()/BAUD_RATE;
	
	USART3->BRR = baud_divider; // Set Baud rate
	
	USART3->CR1 |= 0b1<<3; // Enable USART3 transmit
	USART3->CR1 |= 0b1<<2; // Enable USART3 receiver
	USART3->CR1 |= 0b1;		 // Enable USART3

	
	uint8_t rx_data = 0;
	uint32_t rx_flag = 0;

uint8_t ErrorMesage[] = "\033[1;31mERROR: \033[mUnrecognized command Try 'r' 'g' 'b' 'o'\n\r";

  while (1)
  {
		rx_flag = (USART3->ISR & (0b1<<5)) >> 5;
		
		if(rx_flag){
			rx_data = USART3->RDR;
		}
		
		switch(rx_data){
			case 0:
				break;
			
			case 'r':
				LEDTogle(RED);
				break;
			case 'b':
				LEDTogle(BLUE);
				break;
			case 'g':
				LEDTogle(GREEN);
				break;
			case 'o':
				LEDTogle(ORANGE);
				break;
		
			default: 
				USARTTransmitBlockingString(ErrorMesage);
				break;
		}
		
		rx_data = 0;
		
  }
}

void USARTTransmitBlocking(uint8_t data){

	while((USART3->ISR & (0b1<<7)) == 0){
		__NOP();
	}
	
	USART3->TDR = data;
	
}

void USARTTransmitBlockingString(uint8_t *data){

	for(uint8_t i = 0; data[i] != 0; i++){
		USARTTransmitBlocking(data[i]);
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
  * @brief System Clock Configuration
  * @retval None
  */

void LEDTogle(uint16_t LED){
	GPIOC->ODR ^= (0b1 << LED);
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
