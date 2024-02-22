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
void processCMD(char letter, char action);
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
	volatile char receivedData[2];
volatile int newDataFlag = 0;

void USART3_4_IRQHandler(void) {
    // Check if the receive register is not empty
    if (USART3->ISR & USART_ISR_RXNE) {
        // Save the received character to the array
        receivedData[newDataFlag] = USART3->RDR;
        
        // Check if we have received two characters
        if (newDataFlag == 1) {
            newDataFlag = 0;  // Reset the flag for the next reception
            // Process the received command with two characters
            processCMD(receivedData[0], receivedData[1]);
        } else {
            newDataFlag++;  // Increment the flag for the next reception
        }
    }
}

		void singleCharacter(char letter) {
	
	while (!(USART3 -> ISR & USART_ISR_TXE)){}
		
		USART3 -> TDR =  letter;
		
		
	}
	
	
	
	void singleString(char* str) {
	
	int i = 0;
	while(str[i] != '\0') {
		singleCharacter(str[i]);
		i++;
	}
}
	void turnOffLED(char color) {
    switch (color) {
        case 'r':
            GPIOC->ODR &= ~(1 << 6); // Turn off the red LED on PC6
            break;
        case 'g':
            GPIOC->ODR &= ~(1 << 9); // Turn off the green LED on PC9
            break;
        case 'b':
            GPIOC->ODR &= ~(1 << 7); // Turn off the blue LED on PC7
            break;
        default:
            // Print an error message for unrecognized color
            singleString("Error: Unrecognized color '");
            singleCharacter(color);
            singleString("'\n");
            break;
    }
}

void turnOnLED(char color) {
    switch (color) {
        case 'r':
            GPIOC->ODR |= (1 << 6); // Turn on the red LED on PC6
            break;
        case 'g':
            GPIOC->ODR |= (1 << 9); // Turn on the green LED on PC9
            break;
        case 'b':
            GPIOC->ODR |= (1 << 7); // Turn on the blue LED on PC7
            break;
        default:
            // Print an error message for unrecognized color
            singleString("Error: Unrecognized color '");
            singleCharacter(color);
            singleString("'\n");
            break;
    }
}

void toggleLED(char color) {
    switch (color) {
        case 'r':
            GPIOC->ODR ^= (1 << 6); // Toggle the red LED on PC6
            break;
        case 'g':
            GPIOC->ODR ^= (1 << 9); // Toggle the green LED on PC9
            break;
        case 'b':
            GPIOC->ODR ^= (1 << 7); // Toggle the blue LED on PC7
            break;
        default:
            // Print an error message for unrecognized color
            singleString("Error: Unrecognized color '");
            singleCharacter(color);
            singleString("'\n");
            break;
    }
}
void processCMD(char letter, char action) {
    switch (letter) {
        case 'r':
        case 'g':
        case 'b':
            switch (action) {
                case '0':
                    turnOffLED(letter);
                    break;
                case '1':
                    turnOnLED(letter);
                    break;
                case '2':
                    toggleLED(letter);
                    break;
                default:
                    singleString("Error: Unrecognized action '");
                    singleCharacter(action);
                    singleString("'\n");
                    break;
            }
            break;
        default:
            singleString("Error: Unrecognized command '");
            singleCharacter(letter);
            singleString("'\n");
            break;
    }
}
		

	void printCommandPrompt() {
   singleString("CMD?");
		singleString("\n");
}
		
	
	
	

	
	
	
	
	
int main(void)
{
	
	RCC->AHBENR |=RCC_AHBENR_GPIOBEN|RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |=RCC_APB1ENR_USART3EN;
  GPIOC -> MODER |= (1 << 12)| (1 << 14)| (1 << 18);
	GPIOB -> MODER |= (1 << 21 ); // PINS PC4 AND PC5
	GPIOB -> MODER |= (1 << 23 ); 
	GPIOB->AFR[1]|= (1 << 10 )|(1 << 14) ; //AF1 on PC4 and PC5

  NVIC_EnableIRQ(USART3_4_IRQn );
	
	NVIC_SetPriority(USART3_4_IRQn , 1);
	
	char* testArray = "Jin Jeong Who Made This";

	
	
	//uint32_t  baud_rate =115200;
	//uint32_t  system_clock = HAL_RCC_GetHCLKFreq();
	//uint32_t  USART_div = (system_clock/baud_rate);
	USART3 ->BRR = 69;
	USART3 -> CR1 |= (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5); //in the USART initialization, enable the receive register not empty interrupt
 
 char action = '0'; // Initialize with a default action or compute dynamically

 while (1) {
        printCommandPrompt();

        // Wait for two characters to be received
        while (newDataFlag != 2) {}

        // Process the received data
        processCMD(receivedData[0], receivedData[1]);

        // Clear the flag for the next reception
        newDataFlag = 0;
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
