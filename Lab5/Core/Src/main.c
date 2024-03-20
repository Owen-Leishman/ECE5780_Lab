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
#include "stdio.h"

#define ADDR 						0x69
#define WHO_AM_I_REG 		0x0f
#define CTRL_REG1		 		0x20
#define ACCEL_CFG				0b00001111
#define ACCEL_BASE_REG	0xA8
#define ACCEL_X_L				0x28
#define ACCEL_X_H				0x29
#define ACCEL_Y_L				0x2a
#define	ACCEL_Y_H				0x2b



#define BAUD_RATE 			115200

#define WHO_AM_I 		 		0b11010011

#define THRESHOLD				200

#define RED 		6
#define BLUE 		7
#define ORANGE	8
#define GREEN		9

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void LEDSet(uint16_t LED, uint8_t value);
void GPIOInit(void);
void I2CInit(void);
void USARTTransmitBlocking(char data);
void USARTTransmitBlockingString(char *data);
void USARTInit(void);
uint8_t I2CWriteBlocking(uint8_t addr, uint8_t *data, uint8_t length, uint8_t stop);
uint8_t I2CReadBlocking(uint8_t addr, uint8_t *data, uint8_t length);
uint8_t I3G4250DInit(void);
uint8_t I3G4250DRead(int16_t* data);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();


  SystemClock_Config();

	GPIOInit();
	USARTInit();

	I2CInit();

	uint8_t error = I3G4250DInit();
	
	int16_t accel[2];
	int32_t average[2];
	
	while (1)
  {
		
		if(error){
			break;
		}
		
		HAL_Delay(100);
		
		error = I3G4250DRead(accel);
		if(error){
			break;
		}
		
		average[0] += accel[0];
		average[1] += accel[1];
		
		average[0] = average[0] >> 1;
		average[1] = average[1] >> 1;
		
		char debug[100];
		char message[] = "X = %d, \tY = %d\r\n";
		
		sprintf(debug, message, average[0], average[1]);
		
		USARTTransmitBlockingString(debug);
		
		if(average[0] > THRESHOLD){
			LEDSet(GREEN, 1);
			LEDSet(ORANGE, 0);
		}

		if(average[0] < (-1 * THRESHOLD)){
			LEDSet(ORANGE, 1);
			LEDSet(GREEN, 0);
		}
		
		if(average[1] < (-1 * THRESHOLD)){
			LEDSet(BLUE, 1);
			LEDSet(RED, 0);
		}	

		if(average[1] > THRESHOLD){
			LEDSet(RED, 1);
			LEDSet(BLUE, 0);
		}
		
		

  }
	
	LEDSet(RED, 1);
	LEDSet(GREEN, 1);
	LEDSet(BLUE, 1);
	LEDSet(ORANGE, 1);

}


uint8_t I3G4250DRead(int16_t* data){
	uint8_t error = 0;
	uint8_t x_l;
	uint8_t x_h;
	uint8_t y_l;
	uint8_t y_h;
	
	uint8_t reg = ACCEL_X_L;
	error = I2CWriteBlocking(ADDR, &reg, 1, 1);
	error = I2CReadBlocking(ADDR, &x_l, 1);

	reg = ACCEL_X_H;
	error |= I2CWriteBlocking(ADDR, &reg, 1, 1);
	error |= I2CReadBlocking(ADDR, &x_h, 1);

	reg = ACCEL_Y_L;
	error |= I2CWriteBlocking(ADDR, &reg, 1, 1);	
	error |= I2CReadBlocking(ADDR, &y_l, 1);
	
	reg = ACCEL_Y_H;
	error |= I2CWriteBlocking(ADDR, &reg, 1, 1);
	error |= I2CReadBlocking(ADDR, &y_h, 1);
	
	if(error){
		return 1;
	}
	
	int16_t xraw = ((x_h) << 8)|(x_l);
	int16_t yraw = ((y_h) << 8)|(y_l);
	
	data[0] = xraw;
	data[1] = yraw;
		
	return 0;
}


uint8_t I3G4250DInit(void){

	uint8_t read_val = 0;
	uint8_t write_val = WHO_AM_I_REG;
	
	uint8_t error = I2CWriteBlocking(ADDR, &write_val, 1, 0); 

	if(error){
		return 1;
	}

	error = I2CReadBlocking(ADDR, &read_val, 1);

	if(error){
		return 1;
	}
	
	if(read_val != WHO_AM_I){

		char debug[100];
		char message[] = "Error WHO_AM_I incorrect, val = %d\r\n";
		sprintf(debug, message, read_val);
		USARTTransmitBlockingString(debug);
		
		return 1;
	}else{
		
		char debug[100];
		char message[] = "Device detected \r\n";
		sprintf(debug, message, read_val);
		USARTTransmitBlockingString(debug);
	
	}


	uint8_t config[] = {CTRL_REG1, ACCEL_CFG};
	
	error |= I2CWriteBlocking(ADDR, &config[0], 1, 0);
	error |= I2CWriteBlocking(ADDR, &config[1], 1, 1);
	


	
	if(error){
		
		char debug[100];
		char message[] = "Error writting configuration \r\n";
		sprintf(debug, message, read_val);
		USARTTransmitBlockingString(debug);
		
		return 1;

	}else{
	
		char debug[100];
		char message[] = "Configuration written \r\n";
		sprintf(debug, message, read_val);
		USARTTransmitBlockingString(debug);
		
	}


	uint8_t config_addr = CTRL_REG1;
	
	error = I2CWriteBlocking(ADDR, &config_addr, 1, 0);

	if(error){
		char debug[100];
		char message[] = "pls work \r\n";
		sprintf(debug, message, read_val);
		USARTTransmitBlockingString(debug);
		
		return 1;
	}	
	
	read_val = 0;
	error = I2CReadBlocking(ADDR, &read_val, 1);
	
	if(read_val != ACCEL_CFG){

		char debug[100];
		char message[] = "Error config incorrect val = %d\r\n";
		sprintf(debug, message, read_val);
		USARTTransmitBlockingString(debug);
		
		return 1;
	}else{
		
		char debug[100];
		char message[] = "Configuration checked \r\n";
		sprintf(debug, message, read_val);
		USARTTransmitBlockingString(debug);	
	
	}		
	
	if(error){
		return 1;
	}		



	return 0;
}

/**
  * @brief Transmits a single character over USART3
  * @retval None
* @param data: the data to be transmitted
  */
void USARTTransmitBlocking(char data){

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
void USARTTransmitBlockingString(char *data){

	for(uint16_t i = 0; data[i] != 0; i++){
		USARTTransmitBlocking(data[i]);
	} 
	
}


void I2CInit(){
	// Initilize I2C2
	RCC->APB1ENR |= 0b1 << 22; // Enable I2C2 Clock
	I2C2->TIMINGR |= (0x1 << 28)|(0x4 << 20)|(0x2 << 16)|(0xF << 8)|0x13;
	I2C2->CR1 |= 0b1;
}


uint8_t I2CReadBlocking(uint8_t addr, uint8_t *data, uint8_t length){

	// Start a transaction
	
	I2C2->CR2 |= addr << 1; // Set address to transmit to
	I2C2->CR2 |= length << 16;   // Set the number of bytes to transmit
	I2C2->CR2 |= 0b1 << 10;				// Set the transfer to read
	I2C2->CR2 |= 0b1 << 13; // Start the transfer	
	
	I2C2->CR2 &= ~(0b1 << 25);
	
	uint32_t val = 0;
	
	for(int i = 0; i < length; i ++){
		
		// Wait for RXNE or NACKF
		while((val & 0b10100) == 0){
			val = I2C2->ISR;
		}
	
		// Check if NACKF
		if((val & 0b10000) != 0){
			return 1;
		}	
		
		data[i] = I2C2->RXDR;
		
		
		
	}

	I2C2->CR2 &= ~(length << 16);
	
	// Wait for TD flag
	while((I2C2->ISR & (0b1000000)) == 0){
		__NOP();
	}
	
	// Send stop condition
	I2C2->CR2 |= 0b1 << 14;
	
	return 0;
}

uint8_t I2CWriteBlocking(uint8_t addr, uint8_t *data, uint8_t length, uint8_t stop){
	
	// Start a transaction
	I2C2->CR2 |= addr << 1; // Set address to transmit to
	I2C2->CR2 |= length << 16;   // Set the number of bytes to transmit
	I2C2->CR2 &= ~(0b1 << 10);		// Set the transfer to write
	I2C2->CR2 |= 0b1 << 13; // Start the transfer
	
	
	uint32_t val = 0;
	
	
	val = 0;
	
	for(int i = 0; i < length; i++){
	
		// Wait for TXIS or NACKF
		while((val & 0b10010) == 0){
			val = I2C2->ISR;
		}
	
		// Check if NACKF
		if((val & 0b10000) != 0){
			return 1;
		}	
		
		// Transmit data
		I2C2->TXDR = data[i];
	
	}

	
	
	// Wait for TC flag 
	while((val & (0b1010000)) == 0){
		val = I2C2->ISR;
		if((val & (0b10)) != 0){
			//I2C2->ISR |= 0b1;
		}
		__NOP();
	}
	
	// Check if NACKF
	if((val & 0b10000) != 0){
		return 1;
	}	
	
	// Send stop condition
	I2C2->CR2 |= stop << 14;	

	return 0;
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
	
	
	// Initialize USART GPIOS
	// TX PC10
	// RX PC11
	
	RCC->APB1ENR |= 0b1<<18; // Enable the clock for USART3
	
	GPIOC->MODER |= 0b10<<20; // Set PC10 to alternate function
	GPIOC->MODER |= 0b10<<22; // Set PC11 to alternate function
	
	GPIOC->AFR[1] |= 0b0001<<8;  // Set PC10 alternate function to AF1
	GPIOC->AFR[1] |= 0b0001<<12; // Set PC11 alternate function to AF1	
	
	
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

void USARTInit(){
	uint32_t baud_divider = HAL_RCC_GetHCLKFreq()/BAUD_RATE; // Calculate baud rate
	
	USART3->BRR = baud_divider; // Set Baud rate
	
	USART3->CR1 |= 0b1<<5; // Enable USART3 recieve not empty interrupt
	USART3->CR1 |= 0b1<<3; // Enable USART3 transmit
	USART3->CR1 |= 0b1<<2; // Enable USART3 receiver
	USART3->CR1 |= 0b1;		 // Enable USART3
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
