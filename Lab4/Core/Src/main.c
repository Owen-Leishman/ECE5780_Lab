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

#define LAB_PART 0 // Choose which part of the lab to run (part 1 or 2)

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
void USART3_4_IRQHandler(void);
void LABPart1(void);
void LABPart2(void);
void LEDSet(uint16_t LED, uint8_t value);

// Global interupt variables
volatile uint8_t interrupt_data;
volatile uint8_t interrupt_status;

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
	
	USART3->BRR = baud_divider; // Set Baud rate
	
	USART3->CR1 |= 0b1<<5; // Enable USART3 recieve not empty interrupt
	USART3->CR1 |= 0b1<<3; // Enable USART3 transmit
	USART3->CR1 |= 0b1<<2; // Enable USART3 receiver
	USART3->CR1 |= 0b1;		 // Enable USART3

	// Run lab part 1
	if(LAB_PART == 1){
		LABPart1();
	}
	
	// Run lab part 2
	if(LAB_PART == 2){
		LABPart2();
	}
}

/**
  * @brief Transmits a single character over USART3
  * @retval None
* @param data: the data to be transmitted
  */
void USARTTransmitBlocking(uint8_t data){

	// Check if the USART transmit register is full
	while((USART3->ISR & (0b1<<7)) == 0){
		__NOP();
	}
	
	// Write data to the USART transmit register
	USART3->TDR = data;
	
}

/**
  * @brief Transmits a array over uart
  * @retval None
  * @param *data: pointer to null terminated array to transmit
  */
void USARTTransmitBlockingString(uint8_t *data){

	for(uint16_t i = 0; data[i] != 0; i++){
		USARTTransmitBlocking(data[i]);
	} 
	
}

/**
  * @brief Reads data from USART3 when interrupt is turned on
  * @retval None
  */
void USART3_4_IRQHandler(void){
	interrupt_data = USART3->RDR;
	interrupt_status = 1;
}

/**
  * @brief Lab part 2 code
  * @retval None
  */
void LABPart2(void){
	
	// Enable the USART3 interrupt
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);


	// Messages to transmit over USART
	uint8_t startMessage[] = "\r\nPlease enter a command:\r\n";
	uint8_t redMessage[] = "Red LED";
	uint8_t blueMessage[] = "Blue LED";
	uint8_t greenMessage[] = "Green LED";
	uint8_t orangeMessage[] = "Orange LED";
	
	uint8_t onMessage[] = " turned on.\r\n";
	uint8_t offMessage[] = " turned off.\r\n";
	uint8_t togleMessage[] = " toggled.\r\n";

	uint8_t errorMessage[] = "\033[1;31mERROR \033[mUnrecognized command, try '-h' for help\r\n";

	uint8_t helpMessage[] = "LED Control software\r\n\n\tcommand format: [LED COLOR][LED COMMAND]\r\n\n\tLED COLOR:\r\n\t\tr - RED\r\n\t\tg - GREEN\r\n\t\tb - BLUE\r\n\t\to - ORANGE\r\n\n\tLED COMMAND\r\n\t\t0 - turn LED off\r\n\t\t1 - turn LED on\r\n\t\t2 - toggle LED";

	// Program state variables
	uint8_t lastRead = 0;
	uint8_t start = 0;

  while (1)
  {
		
		// If the start message has not been printed, print the start message
		if(start == 0){
			USARTTransmitBlockingString(startMessage);
			start = 1;
		}
		
		// If an interrupt has happened interpret the data
		if(interrupt_status){
			
			// If this is the first value read store it
			if(lastRead == 0){
				
				lastRead = interrupt_data;
				interrupt_status = 0;
				
			// If this is the second value read run the command	
			}else{
				
				// Interpret the command
				switch(lastRead){
					case 'r':
						switch(interrupt_data){
							case '0':
								LEDSet(RED, 0);
								USARTTransmitBlockingString(redMessage);
								USARTTransmitBlockingString(offMessage);
								break;
							
							case '1':
								LEDSet(RED, 1);
								USARTTransmitBlockingString(redMessage);
								USARTTransmitBlockingString(onMessage);
								break;
								
							case '2':
								LEDTogle(RED);
								USARTTransmitBlockingString(redMessage);
								USARTTransmitBlockingString(togleMessage);
								break;
								
							default:
								USARTTransmitBlockingString(errorMessage);
								break;
						}
						break;

					case 'g':
						switch(interrupt_data){
							case '0':
								LEDSet(GREEN, 0);
								USARTTransmitBlockingString(greenMessage);
								USARTTransmitBlockingString(offMessage);
								break;
							
							case '1':
								LEDSet(GREEN, 1);
								USARTTransmitBlockingString(greenMessage);
								USARTTransmitBlockingString(onMessage);
								break;
								
							case '2':
								LEDTogle(GREEN);
								USARTTransmitBlockingString(greenMessage);
								USARTTransmitBlockingString(togleMessage);
								break;
								
							default:
								USARTTransmitBlockingString(errorMessage);
								break;
						}
						break;
						
					case 'b':
						switch(interrupt_data){
							case '0':
								LEDSet(BLUE, 0);
								USARTTransmitBlockingString(blueMessage);
								USARTTransmitBlockingString(offMessage);
								break;
							
							case '1':
								LEDSet(BLUE, 1);
								USARTTransmitBlockingString(blueMessage);
								USARTTransmitBlockingString(onMessage);
								break;
								
							case '2':
								LEDTogle(BLUE);
								USARTTransmitBlockingString(blueMessage);
								USARTTransmitBlockingString(togleMessage);
								break;
								
							default:
								USARTTransmitBlockingString(errorMessage);
								break;
						}
						break;

					case 'o':
						switch(interrupt_data){
							case '0':
								LEDSet(ORANGE, 0);
								USARTTransmitBlockingString(orangeMessage);
								USARTTransmitBlockingString(offMessage);
								break;
							
							case '1':
								LEDSet(ORANGE, 1);
								USARTTransmitBlockingString(orangeMessage);
								USARTTransmitBlockingString(onMessage);
								break;
								
							case '2':
								LEDTogle(ORANGE);
								USARTTransmitBlockingString(orangeMessage);
								USARTTransmitBlockingString(togleMessage);
								break;
								
							default:
								USARTTransmitBlockingString(errorMessage);
								break;
						}
						break;

					case '-':
						switch(interrupt_data){
							case 'h':
								USARTTransmitBlockingString(helpMessage);
								break;
									
							default:
								USARTTransmitBlockingString(errorMessage);
								break;
							}
							break;

					default:
						USARTTransmitBlockingString(errorMessage);
						break;
				}
				
				lastRead = 0;
				interrupt_status = 0;
				interrupt_data = 0;
				start = 0;
			}
		}
  }
}


/**
  * @brief Lab part 1 code
  * @retval None
  */
void LABPart1(void){
	uint8_t rx_data = 0;
	uint32_t rx_flag = 0;
	
	uint8_t ErrorMesage[] = "\033[1;31mERROR: \033[mUnrecognized command, try 'r' 'g' 'b' 'o'\n\r";
	
	
	
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
  * @brief Toggle an led
  * @retval None
* @param LED: which LED to toggle
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
