/**
Copyright (c) 2020 Out of the BOTS
MIT License (MIT) 
Author: Shane Gingell
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
ll copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/



#include "main.h"
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);


#define timer_freq 84.0  //timer clock freq in MHz
#define T0H 0.5  //each different clone can have their own timings
#define T1H 1.2  //timing here are in us
#define T0L 2.0
#define T1L 1.3

long double period;

uint16_t pos;
uint8_t mask = 0B10000000;
uint16_t low_CCR1, low_ARR, high_CCR1, high_ARR;
uint8_t LED_data[180]; //I have strip with 60 LEDs and need 3 bytes/LED


void Neopixel_setup(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->PSC = 0;   //set prescale to zero as timer has to go as fast as posible
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0b110<<4)) | (0b110<<4); //set PWM mode 110
	TIM4->CCER &= ~TIM_CCER_CC1E; //disable output to pin so that it will be low until transmission starts.
	TIM4->CR1 &= ~TIM_CR1_CEN; //Disable channel 1. This bit is used to start and stop transmission.
	TIM4->CR1 |= TIM_CR1_ARPE; //this allows for imediate updates for register to take effect
	TIM4->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)

	//calculate all the timings.
	period = 1 / timer_freq;
	low_CCR1 = round(T0H / period);
	low_ARR = round((T0H + T0L) / period);
	high_CCR1 = round(T1H / period);
	high_ARR = round((T1H + T1L) / period);
}


void show_neopixels(){

	//setup timer for first bit
	if(LED_data[0] & 0B10000000){
				TIM4->CCR1 = high_CCR1;
				TIM4->ARR = high_ARR;
			}else{
				TIM4->CCR1 = low_CCR1;
				TIM4->ARR = low_ARR;
			}

	pos = 0; //set the interupt to start at first byte
	mask = 0B01000000; //set the interupt to start at second bit
	TIM4->CCER |= TIM_CCER_CC1E; //enable output to pin
	TIM4->CR1 |= TIM_CR1_CEN;    //enable channel 1
}


int main(void){

  HAL_Init();
  SystemClock_Config();  //this is a cube_MX function for setting up all the clocks

  MX_GPIO_Init(); //this is a cube_MX function that setups pin D12 to AF timer pin

  Neopixel_setup(); //setup the neopixels


  uint8_t i;
  for (i = 0; i < 10; ++i){ //fill first 10 LEDs green
	  LED_data[i*3] = 10;
  }
  for (i = 10; i < 20; ++i){ //fill next 10 LEDs red
	  LED_data[i*3+1] = 250;
  }
  for (i = 20; i < 30; ++i){  //fill next 10 LEDs blue
	  LED_data[i*3+2] = 255;
  }
  for (i = 30; i < 40; ++i){  //fill next 10 LEDs yellow
	  LED_data[i*3] = 255;
	  LED_data[i*3+1] = 255;
  }
  for (i = 40; i < 50; ++i){  //fill next 10 LEDs purple
	  LED_data[i*3+1] = 255;
	  LED_data[i*3+2] = 255;
  }
  for (i = 50; i < 60; ++i){  //fill last 10 LEDs orange
	  LED_data[i*3] = 165;
	  LED_data[i*3+1] = 255;
  }

  show_neopixels();  //transmit the data to the neopixel strip.

  while (1){
  }
}



void TIM4_IRQHandler(void){
	if(TIM4->SR & TIM_SR_UIF){ // if UIF flag is set
		TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag
	}

	if(pos<sizeof(LED_data)){
		if(LED_data[pos] & mask){
			TIM4->CCR1 = high_CCR1;
			TIM4->ARR = high_ARR;
		}else{
			TIM4->CCR1 = low_CCR1;
			TIM4->ARR = low_ARR;
		}
		if(mask==1){
			mask = 0B10000000;
			pos+=1;
		}else mask = mask >> 1;
	}else{
		TIM4->CCER &= ~TIM_CCER_CC1E; //disable output to pin so that it will be low.
		TIM4->CR1 &= ~TIM_CR1_CEN; //Disable channel 1
	}
}




/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PD12   ------> S_TIM4_CH1
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
