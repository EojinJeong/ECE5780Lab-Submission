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
// Sine Wave: 8-bit, 32 samples/cycle
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
// Triangle Wave: 8-bit, 32 samples/cycle
const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
// Sawtooth Wave: 8-bit, 32 samples/cycle
const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
// Square Wave: 8-bit, 32 samples/cycle
const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
const uint8_t THRESHOLDS[4] = {10, 20, 30, 40};
int main(void)
{

  HAL_Init();

  SystemClock_Config();
	//1. Initialize the LED pins to output.
  RCC -> AHBENR  |=  RCC_AHBENR_GPIOCEN; 
  GPIOC -> MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);
	//2. Select a GPIO pin to use as the ADC input.
	GPIOC -> MODER |= (1 << 0)| (1 << 1); // Set PC0 as analog mode
  //3. Enable the ADC1 in the RCC peripheral.
  RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN; 
	// 4. Configure the ADC to 8-bit resolution, continuous conversion mode, hardware triggers disabled (software trigger only).--> by default.
  ADC1->CFGR1 |= (1 << 4) | (1 << 13) ; 
	// 5. Select/enable the input pin’s channel for ADC conversion.
  ADC1->CHSELR |= (1 << 10); // Select ADC channel 10 (connected to PC0)
	//6. Perform a self-calibration, enable, and start the ADC
	ADC1->CR |= (ADC_CR_ADCAL); 
	while(1)
	{	if(~(ADC1->CR) & ( ADC_CR_ADCAL)) { break; }} 
	ADC1->CR |= ADC_CR_ADEN; 

	while(1) {	if(ADC1->ISR & (ADC_ISR_ADRDY)) {break;}}
	ADC1->CR |= ADC_CR_ADSTART; 
	
	
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	// Select GPIO pin as DAC output (DAC_OUT1 in this example)
	GPIOA->MODER |= (1 << 8) |(1 << 9); // Set PA4 as analog mode --> DAC channel 1
 
  DAC1->CR |= ((1 << 5) | (1 << 4) | (1 << 3));
	
	//Enable Channel 1
	DAC1->CR |= (1 << 0);
	
	uint32_t index = 0; // Initialize index variable

	
	while (1) {
    // Write next value in sine table to DAC data register
    DAC1->DHR8R1 = sine_table[index];

    // Increment index for next value
    index = (index + 1) % 32;

    // Delay for 1ms
    HAL_Delay(1);
						
						}
	
  while (1) {
    // Read ADC data register
    uint8_t adc_value = ADC1->DR;
 
		if (adc_value < 50 ){
		GPIOC -> ODR |= 1 << 6;
	  GPIOC->ODR &= ~((1 << 7) | (1 << 8) | (1 << 9));
		}
			if (adc_value > 50 && adc_value < 100 ){
		GPIOC -> ODR |= 1 << 7;
	  GPIOC->ODR &= ~((1 << 6) | (1 << 8) | (1 << 9));
		}
   	if (adc_value > 100 && adc_value < 150 ){
		GPIOC -> ODR |= 1 << 8;
		GPIOC->ODR &= ~((1 << 6) | (1 << 7)  | (1 << 9));
		}
   	if (adc_value > 150 && adc_value < 200 ){
		GPIOC -> ODR |= 1 << 9;
		GPIOC->ODR &= ~((1 << 6) | (1 << 7)  | (1 << 8));
		}
   
   
 
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
