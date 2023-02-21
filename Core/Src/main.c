
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);



/* Private user code ---------------------------------------------------------*/
static uint8_t spi_flag;
static uint8_t spiSendBuff[SPI_TX_BUFF_LEN];
static uint8_t spiRcvBuff[SPI_RX_BUFF_LEN];
static uint8_t u8seq;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  Uart_Init();
  u8Spi_Slave_init();

  printf("init ready\r\n");

  memcpy(&spiSendBuff[0], "SLV", SPI_TX_BUFF_LEN);
  u8Spi_Slave_rcvOnly(spiRcvBuff, SPI_RX_BUFF_LEN);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  /* Infinite loop */
  while (1)
  {


	  if (spi_flag & SPI_WRITE_CPLT)
	  {
		  spi_flag 	&= ~SPI_WRITE_CPLT;
		  u8Spi_Slave_run();
		  printf("spi write complete\r\n");
	  }
	  else if (spi_flag & SPI_READ_CPLT)
	  {
		  spi_flag 	&= ~SPI_READ_CPLT;
		  memset(spiRcvBuff, 0, SPI_RX_BUFF_LEN);
		  u8Spi_Slave_run();
		  printf("spi read complete\r\n");
	  }
	  else if (spi_flag & SPI_WR_UPDATE)
	  {
		  spi_flag 	&= ~SPI_WR_UPDATE;
		  printf("spi write update\r\n");

		  memset(&spiSendBuff[0], 0, SPI_TX_BUFF_LEN);

		  if (u8seq == 0)
		  {
			  u8seq = 1;
		  	  memcpy(spiSendBuff, "1ST", SPI_TX_BUFF_LEN);
		  }
		  else if (u8seq == 1)
		  {
			  u8seq = 2;
			  memcpy(spiSendBuff, "2ND", SPI_TX_BUFF_LEN);
		  }
		  else
		  {
			  u8seq = 0;
			  memcpy(spiSendBuff, "3RD", SPI_TX_BUFF_LEN);
		  }

		  u8Spi_Slave_sendOnly(spiSendBuff, SPI_TX_BUFF_LEN);

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

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /* Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType 		= RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState 			= RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState 		= RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource 		= RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM 			= 16;
  RCC_OscInitStruct.PLL.PLLN 			= 336;
  RCC_OscInitStruct.PLL.PLLP 			= RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ 			= 2;
  RCC_OscInitStruct.PLL.PLLR 			= 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource 	= RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider 	= RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider 	= RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider 	= RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
}




/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin 	= B1_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Enable and set EXTI lines 15 to 10 Interrupt */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin 	= LD2_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}



/************************************************************
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  ***********************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
	  spi_flag |= SPI_WR_UPDATE;
  }

}


/************************************************************
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  ***********************************************************/
void SPI_Callback(eSPIop_t eOps)
{
  switch (eOps)
  {
	case SPI_READ_OP:
	case SPI_WRnRD_OP:
	  spi_flag |= SPI_READ_CPLT;
	  break;

	case SPI_WRITE_OP:
	  spi_flag |= SPI_WRITE_CPLT;
	  break;

	case SPI_IDLE_OP:
	default:
	  break;
  }

}







/************************************************************
  * @brief  in case of error occurrence.
  * @retval None
  ************************************************************/
void Error_Handler(char * file, int line)
{
	__disable_irq();
	//printf("\r\nERROR: %s, line: %d \r\n",file,line);

	while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
