/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "nt.h"
#include "lsm6dsl.h"
#include "myFunc.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t xgDRDY = 0;
volatile uint8_t rxChar;
ntImuConf imuConfig;
uint8_t connection = 255;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void UART_EnableRIT(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  imuConfig.ntUart = &huart1;
  ntInit(&imuConfig);
  UART_EnableRIT(&huart1);
	htim16.Init.Prescaler = 9999;
	if (imuConfig.myID == NTBUS_ID_IMU1) {
		htim16.Init.Period = (SystemCoreClock / (htim16.Init.Prescaler+1)) / 5 - 1;
	}
	else {
		htim16.Init.Period = (SystemCoreClock / (htim16.Init.Prescaler+1)) / 2 - 1;
	}
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
	{
		Error_Handler();
	}
	while (LSM6DSL_Init(&hspi1));
	HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t start = HAL_GetTick();
  while (1)
  {
	int16_t acc[3], gyro[3], temperature;
	if(xgDRDY || ((HAL_GetTick() - start) > 1000)) {
		xgDRDY = 0;
		start = HAL_GetTick();
		uint8_t temp;
		LSM6DSL_ACC_GYRO_ReadReg(LSM6DSL_ACC_GYRO_STATUS_REG, &temp, 1);
		if (temp & 0x01) {
		  //read func
		  LSM6DSL_AccRaw(acc);
		  if ((-32768 < imuConfig.imuData->AccX && imuConfig.imuData->AccX < 32767) &&
				  (-32768 < imuConfig.imuData->AccY && imuConfig.imuData->AccY < 32767) &&
				  (-32768 < imuConfig.imuData->AccZ && imuConfig.imuData->AccZ < 32767)) {
			  imuConfig.imuData->ImuStatus |= NTBUS_IMU_IMUSTATUS_ACCDATA_OK;
			  imuConfig.imuData->ImuStatus &= ~(NTBUS_IMU_IMUSTATUS_ACC_CLIPPED);
		  }
		  else {
			  imuConfig.imuData->ImuStatus |= NTBUS_IMU_IMUSTATUS_ACC_CLIPPED;
			  imuConfig.imuData->ImuStatus &= ~(NTBUS_IMU_IMUSTATUS_ACCDATA_OK);
		  }
		}
		if (temp & 0x02) {
		  //read function
		  LSM6DSL_GyroRaw(gyro);
		  if ((-32768 < imuConfig.imuData->GyroX && imuConfig.imuData->GyroX < 32767) &&
							  (-32768 < imuConfig.imuData->GyroY && imuConfig.imuData->GyroY < 32767) &&
							  (-32768 < imuConfig.imuData->GyroZ && imuConfig.imuData->GyroZ < 32767)) {
			  imuConfig.imuData->ImuStatus |= NTBUS_IMU_IMUSTATUS_GYRODATA_OK;
			  imuConfig.imuData->ImuStatus &= ~(NTBUS_IMU_IMUSTATUS_GYRO_CLIPPED);
		  }
		  else {
			  imuConfig.imuData->ImuStatus |= NTBUS_IMU_IMUSTATUS_GYRO_CLIPPED;
			  imuConfig.imuData->ImuStatus &= ~(NTBUS_IMU_IMUSTATUS_GYRODATA_OK);
		  }
		}
		if (temp & 0x04) {
		  LSM6DSL_TempRaw(&temperature);
		}
		memcpy(&imuConfig.imuData->AccX, acc, 6);
		memcpy(&imuConfig.imuData->GyroX, gyro, 6);
		memcpy(&imuConfig.imuData->Temp, &temperature, 2);
	}
	if (connection-- > 1) {
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	} else {
		if (connection == 0) connection = 1;
		HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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

/* USER CODE BEGIN 4 */
void UART_EnableRIT(UART_HandleTypeDef *huart) {
	UART_MASK_COMPUTATION(huart);
	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

	/* Enable the UART Parity Error and Data Register not empty Interrupts */
	SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
}



void UART_RIT(UART_HandleTypeDef *huart) {
	if ((huart->Instance->ISR & USART_ISR_RXNE) && (huart->Instance->CR1 & USART_CR1_RXNEIE)) {
		uint8_t data = huart->Instance->RDR & 0xff;
		ntParseCommand(data);
		connection = 255;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		UART_EnableRIT(&huart1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == INT_2_Pin) {
		xgDRDY = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim16) {
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */

  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
