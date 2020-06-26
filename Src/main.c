/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c 
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use GPIOs through 
  *          the STM32F3xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include <stdbool.h>
#include "main.h"

static void SystemClock_Config(void);
static void Error_Handler(void);

#define 	FUNC_SPACE		"RAM_0x1000"
#define 	VARS_SPACE		"RAM_0x2000"

static uint64_t gapsqrt64(uint64_t a)		__attribute__((section(FUNC_SPACE)));
static void GPIO_INIT(GPIO_TypeDef* port,uint16_t pin,uint32_t mode,uint8_t alternateFunction);

#define COMMON_PORT		GPIOA
#define COMMON_PIN		GPIO_PIN_3
int main(void)
{
  static uint64_t RESULT  							__attribute__((section(VARS_SPACE)))=0;
	
  HAL_Init();

  /* Configure the system clock to 72 Mhz */
  SystemClock_Config();

	GPIO_INIT(COMMON_PORT,COMMON_PIN,GPIO_MODE_OUTPUT_PP,NULL);

	while(true){
				COMMON_PORT->ODR ^= COMMON_PIN;
				RESULT = gapsqrt64(0x20000000000);		
				RESULT = 0;	
	}

}


uint64_t gapsqrt64(uint64_t a) {
	
	static	 uint64_t rem  __attribute__((section(VARS_SPACE)))=0;
	static	 uint64_t root __attribute__((section(VARS_SPACE)))=0;
	static 	 uint8_t  i 	 __attribute__((section(VARS_SPACE)))=0;

	rem  =0;
	root = 0;
	i=0;
	
	for (i = 64 / 2; i > 0; i--)
	{
			root <<= 1;
			rem = (rem << 2) | (a >> (64 - 2));
			a <<= 2;
			if (root < rem) {
					rem -= root | 1;
					root += 2;
			}
	}
	return root >> 1;
	
}

static void GPIO_INIT(GPIO_TypeDef* port,uint16_t pin,uint32_t mode,uint8_t alternateFunction){

				GPIO_InitTypeDef GPIO_InitStruct;

				if (port == GPIOA)
				{
						__HAL_RCC_GPIOA_CLK_ENABLE();
				}
				else if (port == GPIOB)
				{
						__HAL_RCC_GPIOB_CLK_ENABLE();
				}
				else if(port == GPIOC)
				{
						__HAL_RCC_GPIOC_CLK_ENABLE();
				}
				else if (port == GPIOD)
				{
						__HAL_RCC_GPIOD_CLK_ENABLE();
				}
				else if (port == GPIOE)
				{
						__HAL_RCC_GPIOE_CLK_ENABLE();
				}
				else if (port == GPIOF)
				{
						__HAL_RCC_GPIOF_CLK_ENABLE();
				}

				GPIO_InitStruct.Pin = pin;
				GPIO_InitStruct.Mode = mode;
				
				if (mode != GPIO_MODE_INPUT){
					GPIO_InitStruct.Pull = GPIO_PULLDOWN;
				}	else {
					GPIO_InitStruct.Pull = GPIO_NOPULL;
				}
				
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
				GPIO_InitStruct.Alternate = alternateFunction;
				HAL_GPIO_Init(port, &GPIO_InitStruct);
				
				HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    Error_Handler();
  }
}





/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
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
void assert_failed(char* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
