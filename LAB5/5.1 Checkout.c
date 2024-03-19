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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
		RCC -> AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; //Enable GPIOB and GPIOC in the RCC
		//Set PB11 to alternate function mode, open-drain output type, and select I2C2_SDA as its alternate function
		//Set PB13 to alternate function mode, open-drain output type, and select I2C2_SCL as its alternate function
		//Set PB14 to output mode, push-pull output type, and initialize/set the pin high.
		//Set PC0 to output mode, push-pull output type, and initialize/set the pin high.
		// Merged the process for the readability 
		GPIOB -> MODER |= (1 << 27) | (1 << 23) | (0 << 22) | (1<<28); 
		GPIOB -> OTYPER |= (1 << 13) | (1 << 11);
		GPIOB -> PUPDR |= (1 << 22) | (1 << 26);
		GPIOB -> AFR[1] |= (1 << 12) | (1 << 22) | (1 << 20); 
		GPIOB -> ODR |= (1 << 14);
		GPIOC -> MODER |= (1 << 0); 
		GPIOC -> ODR |= (1 << 0);
		GPIOC -> MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
	 
	
	
		//Enable the I2C2 peripheral in the RCC
		RCC -> APB1ENR |= RCC_APB1ENR_I2C2EN;
		//Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C.
		I2C2 -> TIMINGR |= ((1 << 28) | (1 << 4) | (1 << 0) | (1 << 1) | (1 << 11) | (1 << 10) | (1 << 9) | (1 << 8) | (1 << 17) | (1 << 22));
		//Enable the I2C peripheral using the PE bit in the CR1 register
		I2C2 -> CR1 |= (1 << 0); 
	
	
	
		//Set the L3GD20 slave address = 0x69
		I2C2 -> CR2 |= (0x69 << 1);
		//Set the number of bytes to transmit = 1.
		I2C2 -> CR2 &= ~(0xFF << 16);
		I2C2 -> CR2 |= (1 << 16);
		//Set the RD_WRN bit to indicate a write operation. by defalut
		I2C2 -> CR2 &= ~(1 << 10);
		//Set the START bit.
		I2C2 -> CR2 |= I2C_CR2_START;
	
  
	
    //Wait until either of the TXIS (Transmit Register Empty/Ready) or NACKF (Slave Not-Acknowledge) flags are set.
		while (1) {
			if(I2C2->ISR & (I2C_ISR_TXIS)) {
				break;
																		 }
		//If the NACKF flag is set, the slave did not respond to the address frame. You may have a wiring or configuration error.
			if(I2C2->ISR & I2C_ISR_NACKF)  { 
				while (1){GPIOC->BSRR |= (1 << 6);}
																	   }
	
							}
    //Continue if the TXIS flag is set.

							
							
		//Write the address of the "WHO_AM_I" register into the I2C transmit register (TXDR)
		I2C2->TXDR |= 0x0F;

							
							
    //Wait until the TC (Transfer Complete) flag is set.
		while (1) {
			if(I2C2->ISR & (I2C_ISR_TC)) {break;}
							}
    //Reload the CR2 register with the same parameters as before
		I2C2 -> CR2 |= (0x69 << 1);
		I2C2 -> CR2 &= ~(0xFF << 16);
		I2C2 -> CR2 |= (1 << 16);
		//but set the RD_WRN bit toindicate a read operation. 
		I2C2 -> CR2 |= (1 << 10);
		//Set the START bit to begin the address frame
		I2C2->CR2 |= I2C_CR2_START;


		//Wait until either of the RXNE (Receive Register Not Empty) or NACKF (Slave Not-Acknowledge) flags are set
		while (1) {
			if(I2C2->ISR & (I2C_ISR_RXNE)) {break;}

			if(I2C2->ISR & I2C_ISR_NACKF) { 
				while (1){GPIOC->BSRR |= (1 << 6);}
																		}
	
							}
		int32_t data = I2C2 -> RXDR;
		//Check the contents of the RXDR register to see if it matches 0xD3. (expected value of the WHO_AM_I”  register)
		if(data ==0xD3) {
			GPIOC -> BSRR |= 1 << 7;
										}
		// Wait until the TC (Transfer Complete) flag is set
		while (1) {
								if(I2C2->ISR & (I2C_ISR_TC)) {break;}
							}
		//Set the STOP bit in the CR2 register to release the I2C bus. 
		I2C2 -> CR2 |= 1 << 14;

		while (I2C2 -> CR2 & (1 << 14)) {
																		
																		}
		//Set the STOP bit in the CR2 register to release the I2C bus. 
		I2C2 -> CR2 &= ~ ( 1 << 14);
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