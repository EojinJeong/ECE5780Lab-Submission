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
#include "stm32f072xb.h"

	// Device header


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

void TIM2_IRQHandler (void) //Look for an interrupt containing the name of the timer you are using (3.1)
	
{
	
	GPIOC->ODR ^= (1 << 8) | (1 << 9) ; //Toggle between the green (PC8) and orange (PC9) LEDs in the interrupt handler. (3.1)
   
	TIM2 -> SR &= 0; //Don't forget to clear the pending flag for the update interrupt in the status register.
}

int main(void)
{	  
	
	HAL_Init();
  SystemClock_Config();
	
	RCC->AHBENR |=RCC_AHBENR_GPIOCEN ; // Clock ENable for every Part.. (3.1)
  RCC ->APB1ENR |= RCC_APB1ENR_TIM2EN| RCC_APB1ENR_TIM3EN ;  //Enable the timer 2 peripheral (TIM2) in the RCC (3.1) Enable the timer 3 peripheral (TIM3) in the RCC (3.2)
  TIM2 -> PSC = 0x7CF;   // hex value  --> 1999 because it is Zero indexed //80000000/2000 = 4khz (3.1)
	TIM2 -> ARR = 0x3E8;  // hex value --> 1000 because // 1K for the upper bound (3.1)
	TIM2 -> DIER |= 1;   //Configure the timer to generate an interrupt on the UEV event. (3.1)
	TIM2 -> CR1 |= 1;   //Configure and enable/start the timer (3.1)
	TIM2 -> CR2 = 1 << 4 ;
   NVIC_EnableIRQ (TIM2_IRQn);  // Set up the timers interrupt handler, and enable in the NVIC. (3.1)
	
	 GPIOC -> MODER |= (1 << 16) | (1 << 18); //Initialize Green and Orange LED pins in the main function (3.1)
	

	 GPIOC -> ODR |= (1 << 9);
 
	 TIM3 -> PSC = 0x63; // hex value  --> 99 because it is Zero indexed //80000000/100 = 80khz (3.2)
	 TIM3 -> ARR = 0x64;  // hex value --> 100 because // 100 for the upper bound (3.2)
	TIM3 -> CCMR1 &= 0;
	TIM3->CCMR1 |= (1 << 15 ) | (1 << 14) |(1 << 13) | ( 1 << 12) |  (1 << 7) | (1 << 6) | (1 << 5);    // PWM mode 1 for Channel 1 and Channel 2(3.2)  
	
	
  TIM3 -> CCMR1 |= (1 << 3) | (1 << 11); //Use the Capture/Compare Mode Register 1 (CCMR1) register to configure the output channels to PWM mode (3.2)
	TIM3 -> CCER |= (1 << 4) | 1  ; //et the output enable bits for channels 1 & 2 in the CCER register. (3.2)
	
  TIM3->CCR1 = 20;   //set the capture/compare registers (CCRx) for both channels to 20% of your ARR value. (3.2)
  TIM3->CCR2 = 20;  
             
  // TIM3_CH2(7) ,TIM3_CH1(6)  (3.3)
	 GPIOC -> MODER |= (1 << 15) | (1 << 13);		
	 GPIOC->AFR[0] = 0;
	 TIM3 ->CR1  = 1;
	 
		TIM3->CR2 |= (1 << 4);		
 
	
	
  while (1)
  {
    
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
