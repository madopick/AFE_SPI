
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
TIM_HandleTypeDef 	htim1;

/* External variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static uint8_t vSPIcmdParse(uint8_t* u8p_buff, uint16_t u16_len);

/* Private user code ---------------------------------------------------------*/
static volatile uint8_t spi_flag;
static int16_t spiSendBuff[SPI_TX_BUFF_LEN];
static int16_t spiRcvBuff[SPI_RX_BUFF_LEN];
static uint8_t u8btnPressed;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t u8_spiParse;

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  Uart_Init();
  u8Spi_Slave_init();

  printf("\e[1;1H\e[2J");
  printf("INIT OK\r\n");

  u8Spi_Slave_rcvOnly(spiRcvBuff, SPI_RX_BUFF_LEN);

  /* Infinite loop */
  while (1)
  {
	  if (spi_flag & SPI_WRITE_CPLT)
	  {
		  spi_flag 	&= ~SPI_WRITE_CPLT;


		  if (!u8btnPressed)
		  {
			  HAL_TIM_Base_Stop_IT(&htim1);

//			  __HAL_RCC_SPI2_FORCE_RESET();
//			  __HAL_RCC_SPI2_RELEASE_RESET();
//			  u8Spi_Slave_init();

			  printf("spi write complete\r\n\n");
			  u8Spi_Slave_rcvOnly(spiRcvBuff, SPI_RX_BUFF_LEN);
		  }
	  }
	  else if (spi_flag & SPI_READ_CPLT)
	  {
		  spi_flag 	&= ~SPI_READ_CPLT;

		  memset(spiSendBuff, 0, SPI_TX_BUFF_LEN*sizeof(spiSendBuff[0]));
		  u8_spiParse = vSPIcmdParse((uint8_t*)spiRcvBuff, SPI_RX_BUFF_LEN * 2);

		  if (u8_spiParse == 1)
		  {
			  /* send data reply */
			  for(uint16_t i = 0; i < SPI_TX_BUFF_LEN; i++)
			  {
				  spiSendBuff[i] = i;
			  }
		  }
		  else
		  {
			  /* unknown cmd */
			  spiSendBuff[0] = 0xA5;
			  spiSendBuff[1] = 0x10;
			  spiSendBuff[2] = 0x15;
			  spiSendBuff[3] = 0x5A;

			  //memset(spiRcvBuff, 0, SPI_RX_BUFF_LEN*sizeof(spiRcvBuff[0]));
			  //u8Spi_Slave_rcvOnly(spiRcvBuff, SPI_RX_BUFF_LEN);
			  //printf("spi read complete\r\n\n");
		  }

		  spi_flag |= SPI_WR_UPDATE;

	  }
	  else if (spi_flag & SPI_WR_UPDATE)
	  {
		  spi_flag 	&= ~SPI_WR_UPDATE;
		  printf("\r\nspi write update\r\n");

		  if ((spi_flag & SPI_GPIO_INIT) == 0)
		  {
			  spi_flag |= SPI_GPIO_INIT;
			  u8Spi_Gpio_Init();
		  }

//		  __HAL_RCC_SPI2_FORCE_RESET();
//		  __HAL_RCC_SPI2_RELEASE_RESET();
//		  u8Spi_Slave_init();


		  HAL_TIM_Base_Start_IT(&htim1);
		  u8Spi_Slave_sendOnly(spiSendBuff, SPI_TX_BUFF_LEN);

	  }
	  else if (spi_flag & SPI_TIMEOUT)
	  {
		  spi_flag 	&= ~SPI_TIMEOUT;

		  __HAL_RCC_SPI2_FORCE_RESET();
		  __HAL_RCC_SPI2_RELEASE_RESET();
		  u8Spi_Slave_init();

		  u8Spi_Slave_rcvOnly(spiRcvBuff, SPI_RX_BUFF_LEN);
		  u8btnPressed = 0;

		  printf("timeout OK\r\n\n");
	  }

  }
}


/******************************************************************************
  * @brief  Hanlde SPI command parsing
  * @param
  * @retval None
  ****************************************************************************/
static uint8_t vSPIcmdParse(uint8_t* u8p_buff, uint16_t u16_len)
{
	uint8_t retval = 0;

#if 0
	printf("CMD: %d - %d - %d %d \r\n", u8p_buff[SPI_MODE_BYTE],
										u8p_buff[SPI_WRITE_CMD_BYTE],
										u8p_buff[SPI_WRITE_PRM_BYTE],
										u8p_buff[SPI_WRITE_PRM_BYTE+1]);
#endif

	switch (u8p_buff[SPI_MODE_BYTE])
	{
		case AFE_REQ_WRITE:
			if (u8p_buff[SPI_WRITE_CMD_BYTE] == AFE_CMD_ON)
			{
				printf("scan ON\r\n");
				retval = 1;
			}
			else if (u8p_buff[SPI_WRITE_CMD_BYTE] == AFE_CMD_OFF)
			{
				printf("scan OFF\r\n");
				retval = 1;
			}
			else if (u8p_buff[SPI_WRITE_CMD_BYTE] == AFE_CMD_SCAN_NOISE)
			{
				printf("scan noise\r\n");
				retval = 1;
			}
			else if (u8p_buff[SPI_WRITE_CMD_BYTE] == AFE_CMD_SCAN_SELF_TX)
			{
				printf("scan self tx\r\n");
				retval = 1;
			}
			else if (u8p_buff[SPI_WRITE_CMD_BYTE] == AFE_CMD_SCAN_SELF_RX)
			{
				printf("scan self rx\r\n");
				retval = 1;
			}
			else if (u8p_buff[SPI_WRITE_CMD_BYTE] == AFE_CMD_SCAN_MUTUAL)
			{
				printf("scan mutual\r\n");
				retval = 1;
			}
			break;

		case AFE_REQ_READ:
			printf("read request\r\n");
			break;

		default:
			printf("unknown cmd\r\n");
			break;
	}

	return (retval);
}


/******************************************************************************
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  ****************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t u8TimCnt;

	if (htim->Instance == TIM1)
	{
		u8TimCnt += 1;

		if(u8TimCnt > 100)
		{
			u8TimCnt = 0;
			HAL_TIM_Base_Stop_IT(&htim1);
			printf("TIM1 CB\r\n");
			SPI_Callback(SPI_TO_OP);
		}
	}
}



/****************************************************************
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  ***************************************************************/
static void MX_TIM1_Init(void)
{
	uint32_t              uwTimclock = 0;
	uint32_t              uwPrescalerValue = 0;

	if(HAL_RCC_GetHCLKFreq() >= 180000000)
	{
		uwTimclock = 2*HAL_RCC_GetPCLK2Freq();
	}
	else
	{
		uwTimclock = HAL_RCC_GetPCLK2Freq();
	}

	/* TIM1 (use PCLK2) counter clock equal to 1MHz */
	uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);				//uwPrescalerValue = 180;

	htim1.Instance 				= TIM1;

	//htim1.Init.Period 			= ((1000000U / 10000U) - 1U);  				//0.1 ms time base.
	//htim1.Init.Period 			= ((1000000U / 1000U) - 1U);  				//1 ms time base.
	htim1.Init.Period 			= ((1000000U / 100U) - 1U);  					//10 ms time base.

	htim1.Init.Prescaler 		= uwPrescalerValue;
	htim1.Init.ClockDivision 	= 0;
	htim1.Init.CounterMode 		= TIM_COUNTERMODE_UP;

	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
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
  if ((GPIO_Pin == B1_Pin) && (u8btnPressed == 0))
  {
	  spi_flag |= SPI_WR_UPDATE;
	  u8btnPressed = 1;
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

	case SPI_TO_OP:
	  HAL_TIM_Base_Stop_IT(&htim1);
	  spi_flag |= SPI_TIMEOUT;
	  break;

	case SPI_IDLE_OP:
	default:
	  break;
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

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
