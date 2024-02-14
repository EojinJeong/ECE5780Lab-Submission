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
#include "stm32f072xb.h"     
#include "core_cm0.h"// Device header


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
	
	void EXTI0_1_IRQHandler(void) //Use the handler name to declare the handler function in either main.c or stm32f0xx_it.h. (2.5)
{
    
	GPIOC -> ODR ^= (1 << 8) | (1 << 9) ; //. Toggle both the green and orange LEDs (PC8 & PC9) in the EXTI interrupt handler (2.5)
	  
	for (volatile uint32_t delay = 0; delay < 1500000; ++delay)
    {
        // Add a delay loop of roughly 1-2 seconds to the EXTI interrupt handler (2.6)
    }
		
	  GPIOC->ODR ^= (1 << 8) | (1 << 9); //Add a second LED toggle so that the green and orange LEDs should exchange once before and after the delay loop (2.6)
		
	EXTI -> PR |= 0; //Clear the appropriate flag for input line 0 in the EXTI pending register within the handler. (2.5)
	
	
}


int main(void)
{
 
   HAL_Init();
   SystemClock_Config();
	
	 RCC->AHBENR |=RCC_AHBENR_GPIOCEN |RCC_AHBENR_GPIOAEN| RCC_APB2ENR_SYSCFGCOMPEN 	; // Clock ENable for every Part.. (2.x)
	

   GPIOC -> MODER |= (1 << 12)| (1 << 14)| (1 << 16) | (1 << 18); //Initialize all of the LED pins in the main function (2.1)
	
   GPIOC -> ODR |= (1 << 9); // Set the green LED (PC9) high (we will use this later in the lab) (2.1)
	
	
	 GPIOA -> PUPDR |= (1 << 1); // Configure the button pin (PA0) to input-mode at low-speed, with the internal pull-down resistor enabled. (2.2)
   EXTI -> IMR |= 1;  //Pin PA0 connects to the EXTI input line 0 (EXTI0)  --> he first 16 inputs to the EXTI are for external interrupts; for example, EXTI3 is the 3rd input line which means EXTI0 is 0 input line (2.2)
   EXTI -> EMR |= 0;  //Enable/unmask interrupt generation on EXTI input line 0 (EXTI0) (2.2)
	 EXTI -> RTSR |= 1; //Configure the EXTI input line 0 to have a rising-edge trigger. (2.2)
	 
	
	 SYSCFG ->EXTICR[0] |= 0; // Det\\\\\\\\\\\\\\\\\\\\
	the EXTI peripheral. ITS in fact 0000 (2.3) 
	
	 NVIC_EnableIRQ (EXTI0_1_IRQn);      //<--- (Comment THis for 2.1 with the other codes) Checkout Enable the selected EXTI interrupt by passing its defined name to the NVIC_EnableIRQ() function. (2.4)

  // NVIC_SetPriority (EXTI0_1_IRQn, 1); // Set the priority for the interrupt to 1 (high-priority) with the NVIC_SetPriority() function (2.4)
	 NVIC_SetPriority (EXTI0_1_IRQn, 3); // Change the EXTI interrupt to have priority 3. (lowest priority) (2.7)

	
  	
	 NVIC_SetPriority (SysTick_IRQn, 2); // Change the SysTick interrupt priority to 2 (medium priority) (2.7)
	
	 
  while (1)
  {
   

		
		GPIOC -> ODR ^= (1 << 6); // Toggle the red LED (PC6) (2.1)
		
		HAL_Delay(500);          //with a moderately-slow delay (400-600ms) in the infinite loop. (2.1)
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
