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
#include "stm32f0xx.h"    
#include "stm32f072xb.h"  // Device header


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
	volatile char receivedData;
 volatile int newDataFlag = 0;
	
	
	
	void USART3_IRQHandler(void) {
    // Check if the receive register is not empty
    if (USART3->ISR & USART_ISR_RXNE) {
        // Save the received character to a global variable
        receivedData = USART3->RDR;
        // Set the flag indicating new data
        newDataFlag = 1;
    }}

		void singlecharacter(char letter) {
	
	while (!(USART3 -> ISR & USART_ISR_TXE)){}
		
		USART3 -> TDR =  letter;
		
		
	}
	
	
	
	void singleString(char* str) {
	
	int i = 0;
	while(str[i] != '\0') {
		singlecharacter(str[i]);
		i++;
	}
}
	
	void toggleLED(char letter) {
    switch (letter) {
        case 'r':
            GPIOC->ODR ^= (1 << 6);
            break;
        case 'g':
            GPIOC->ODR ^= (1 << 9);
            break;
        case 'b':
            GPIOC->ODR ^= (1 << 7);
            break;
        default:
            // Print an error message for unrecognized command
            singleString("Error: Unrecognized command '");
            singlecharacter(letter);
            singleString("'\n");
    }
}
		
		
		
		
		
	
	
	

	
	
	
	
	
int main(void)
{
	
	RCC->AHBENR |=RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |=RCC_APB1ENR_USART3EN;
  GPIOC -> MODER |= (1 << 12)| (1 << 14)| (1 << 18);
	GPIOB -> MODER |= (1 << 21 ); // PINS PC4 AND PC5
	GPIOB -> MODER |= (1 << 23 ); 
	GPIOB->AFR[1]|= (1 << 10 )|(1 << 14) ; //AF1 on PC4 and PC5
	//NVIC_SetPriority(USART3_4_IRQn , 0);
  //NVIC_EnableIRQ(USART3_4_IRQn );
	
	char* testArray = "JinJeong Who Made This";

	
	
	//uint32_t  baud_rate =115200;
	//uint32_t  system_clock = HAL_RCC_GetHCLKFreq();
	//uint32_t  USART_div = (system_clock/baud_rate);
	USART3 ->BRR = 69;
	USART3 -> CR1 |= (1 << 0) | (1 << 2) | (1 << 3) ;//| (1 << 5); //in the USART initialization, enable the receive register not empty interrupt
 
  while (1)
  {
  // singlecharacter('J');
	//	singleString(testArray);
		
		while (!(USART3->ISR & USART_ISR_RXNE)) {}

        // Read the received character
        char receivedChar = USART3->RDR;

        // Toggle the appropriate LED based on the received character
        toggleLED(receivedChar);
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
