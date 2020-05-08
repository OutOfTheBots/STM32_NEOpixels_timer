#include "main.h"
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);


#define timer_freq 84.0  //timer clock freq in MHz
#define T0H 0.5  //each different clone can have their own timings
#define T1H 1.2  //timing here are in us
#define T0L 2.0
#define T1L 1.3



uint8_t LED_data[180]; //I have strip with 60 LEDs and need 3 bytes/LED

uint16_t pos;
uint8_t mask = 0B10000000;
uint8_t lastbit;

long double period;
uint16_t low_CCR1, low_ARR, high_CCR1, high_ARR;




void Neopixel_setup(void){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //enable port D clock
	GPIOD->MODER |= GPIO_MODER_MODER12_1; //setup pin 12 on port d to AF mode
	GPIOD->AFR[1] = (GPIOD->AFR[1] & (0b1111<<(4*(12-8))) | 0b0010<<(4*(12-8))); //setup pin 12 on port D to AF timer 2-5

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enable the timer4 clock
	TIM4->PSC = 0;   //set prescale to zero as timer has to go as fast as posible
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0b110<<4)) | (0b110<<4); //set PWM mode 110
	TIM4->CCR1 = 0; //set to zero so that the pin stay low until transmission
	TIM4->ARR = 100; //dummy amount shouldn't matter as long as not too low
	TIM4->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM4->CR1 |= TIM_CR1_CEN; //Disable channel 1. This bit is used to start and stop transmission.
	TIM4->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE; //buffer CCR1
	TIM4->DIER &= ~TIM_DIER_UIE; // ensure we are not enabling interrupt flag to be generated this bit is used to start/stop transmission
	TIM4->CR1 |= TIM_CR1_CEN; //enable channel 1.

	NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)


	//calculate all the timings.
	period = 1 / timer_freq;

	low_CCR1 = round(T0H / period);
	low_ARR = round((T0H + T0L) / period);

	high_CCR1 = round(T1H / period);
	high_ARR = round((T1H + T1L) / period);
}


void show_neopixels(){
	pos = 0; //set the interupt to start at first byte
	lastbit = 0;
	mask = 0B10000000; //set the interupt to start at second bit

	TIM4->CNT = 0; //reset timer to zero
	TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag
	TIM4->DIER |= TIM_DIER_UIE; //enable interupt flag to be generated to start transmission
}


int main(void){

  HAL_Init();
  SystemClock_Config();  //this is a cube_MX function for setting up all the clocks


  Neopixel_setup(); //setup the neopixels


  while (1){
	  //fill the array with repeate pattern of Green-Red-Blue
	  for (uint8_t i = 0; i < 180; i+=9){
		  LED_data[i] = 0;  //use low values so that it does blind the camera
		  LED_data[i+4] = 0;
		  LED_data[i+8] = 25;
	  }

	  show_neopixels();  //transmit the data to the neopixel strip.

	  HAL_Delay(1000);

	  //fill the array with repeate pattern of Green-Red-Blue
	  for (uint8_t i = 0; i < 180; i+=9){
		  LED_data[i] = 25;  //use low values so that it does blind the camera
		  LED_data[i+4] = 25;
		  LED_data[i+8] = 25;
	  }

	  show_neopixels();  //transmit the data to the neopixel strip.

	  HAL_Delay(1000);

  }
}



void TIM4_IRQHandler(void){
	if(TIM4->SR & TIM_SR_UIF){ // if UIF flag is set

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
			TIM4->CCR1 = 0; //set to zero so that pin stays low
			TIM4->DIER &= ~TIM_DIER_UIE; //disable interrupt flag to end transmission.
		}
		TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag
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

