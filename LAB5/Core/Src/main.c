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
 
  RCC->AHBENR |=RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN; // Enable GPIOB and GPIOC in the RCC
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // Enable the I2C2 peripheral in the RCC.
	
	GPIOB -> MODER  |= (1 << 27 ) |(1 << 23 ) | ( 1 << 28) ;  
	GPIOB -> OTYPER |= (1 << 13 )| (1 << 11) ;
	GPIOB->AFR[1]   |= (1 << 12 ) | (1 << 14) | ( 1 << 22) | ( 1 << 20); 
	GPIOB -> BSRR   |= ( 1 << 14 );
	
	GPIOC -> MODER  |= ( 1 << 0 ) | (1 << 14);
	GPIOC -> BSRR   |= (1);

  I2C2->TIMINGR |= (1 << 28) | (1 << 0) | (1 << 1) | (1 << 4) | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 17) | (1 << 22);
  I2C2 -> CR1    |= (1 << 0) ; 
	
	    // Set Transaction Parameters in CR2 Register
    I2C2->CR2 |= (0x69 << I2C_CR2_SADD_Pos); // Set slave address
    I2C2->CR2 |= (1 << I2C_CR2_NBYTES_Pos);  // Set number of bytes to transmit
    I2C2->CR2 &= ~(I2C_CR2_RD_WRN);          // Set as write operation 0
    I2C2->CR2 |= (I2C_CR2_START);            // Start condition

   
	 


	 // Wait for TXIS or NACKF Flags
    while (!((I2C2->ISR & I2C_ISR_TXIS) || (I2C2->ISR & I2C_ISR_NACKF))) {	
		 if (I2C2->ISR & I2C_ISR_NACKF) {
            // Handle NACK error
            break;
        }
    }

	
    // Write the Address of the "WHO_AM_I" Register
    I2C2->TXDR = 0x0F;

    // Wait for Transfer Complete Flag
    while (!(I2C2->ISR & I2C_ISR_TC)); // Wait until transfer complete

    // Reload CR2 Register for Read Operation
    I2C2->CR2 |= (0x69 << I2C_CR2_SADD_Pos); // Set slave address
    I2C2->CR2 |= (1 << I2C_CR2_NBYTES_Pos);  // Set number of bytes to receive
    I2C2->CR2 |= (I2C_CR2_RD_WRN);           // Set as read operation 1 
    I2C2->CR2 |= (I2C_CR2_START);            // Start condition

    // Wait for RXNE or NACKF Flags
    while (!((I2C2->ISR & I2C_ISR_RXNE) || (I2C2->ISR & I2C_ISR_NACKF))) {
        if (I2C2->ISR & I2C_ISR_NACKF) {
            // Handle NACK error
            break;
        }
    }

    // Wait for Transfer Complete Flag
    while (!(I2C2->ISR & I2C_ISR_TC)); // Wait until transfer complete

    // Check Contents of RXDR Register
    uint8_t received_data = I2C2->RXDR;
    if (received_data == 0xD4) {
        // WHO_AM_I register matches expected value
  
    } else {
        // WHO_AM_I register does not match expected value
        // Handle failure
    }

    // Set STOP Bit in CR2 Register
    I2C2->CR2 |= I2C_CR2_STOP; // Stop condition
	
while(1) {}
	
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
