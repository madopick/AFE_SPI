
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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

extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern DMA_HandleTypeDef hdma_usart2_rx;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* External functions --------------------------------------------------------*/

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(hspi->Instance==SPI2)
	{
	/* Peripheral clock enable */

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/**SPI2 GPIO Configuration
	PC1     ------> SPI2_MOSI
	PC2     ------> SPI2_MISO
	PB10    ------> SPI2_SCK
	PB12	------> SPI2_NSS
	*/
	GPIO_InitStruct.Pin 		= GPIO_PIN_1;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate 	= GPIO_AF7_SPI2;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 		= GPIO_PIN_2;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate 	= GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 		= GPIO_PIN_10;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate 	= GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#ifdef NSS_HW
	GPIO_InitStruct.Pin 		= GPIO_PIN_12;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate 	= GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

	/* SPI2 DMA Init */
	/* SPI2_RX Init */
	hdma_spi2_rx.Instance 					= DMA1_Stream3;
	hdma_spi2_rx.Init.Channel 				= DMA_CHANNEL_0;
	hdma_spi2_rx.Init.FIFOMode 				= DMA_FIFOMODE_DISABLE;
	hdma_spi2_rx.Init.FIFOThreshold			= DMA_FIFO_THRESHOLD_FULL;
	hdma_spi2_rx.Init.MemBurst				= DMA_MBURST_INC4;
	hdma_spi2_rx.Init.PeriphBurst			= DMA_MBURST_INC4;
	hdma_spi2_rx.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
	hdma_spi2_rx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	hdma_spi2_rx.Init.MemInc 				= DMA_MINC_ENABLE;
	hdma_spi2_rx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_HALFWORD;
	hdma_spi2_rx.Init.MemDataAlignment 		= DMA_MDATAALIGN_HALFWORD;
	hdma_spi2_rx.Init.Mode 					= DMA_NORMAL;
	hdma_spi2_rx.Init.Priority 				= DMA_PRIORITY_HIGH;

	if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
	{
	  Error_Handler(__FILE__, __LINE__);
	}

	__HAL_LINKDMA(hspi,hdmarx,hdma_spi2_rx);

	/* SPI2_TX Init */
	hdma_spi2_tx.Instance 					= DMA1_Stream4;
	hdma_spi2_tx.Init.Channel 				= DMA_CHANNEL_0;
	hdma_spi2_tx.Init.FIFOMode 				= DMA_FIFOMODE_DISABLE;
	hdma_spi2_tx.Init.FIFOThreshold			= DMA_FIFO_THRESHOLD_FULL;
	hdma_spi2_tx.Init.MemBurst            	= DMA_MBURST_INC4;
	hdma_spi2_tx.Init.PeriphBurst         	= DMA_PBURST_INC4;
	hdma_spi2_tx.Init.Direction 			= DMA_MEMORY_TO_PERIPH;
	hdma_spi2_tx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	hdma_spi2_tx.Init.MemInc 				= DMA_MINC_ENABLE;
	hdma_spi2_tx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_HALFWORD;
	hdma_spi2_tx.Init.MemDataAlignment 		= DMA_MDATAALIGN_HALFWORD;
	hdma_spi2_tx.Init.Mode 					= DMA_NORMAL;
	hdma_spi2_tx.Init.Priority 				= DMA_PRIORITY_LOW;

	if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
	{
	  Error_Handler(__FILE__, __LINE__);
	}

	__HAL_LINKDMA(hspi,hdmatx,hdma_spi2_tx);

	/* SPI2 interrupt Init */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 13, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 13, 1);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

	HAL_NVIC_SetPriority(SPI2_IRQn, 13, 2);
	HAL_NVIC_EnableIRQ(SPI2_IRQn);
	}
}



/*******************************************************************************
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
********************************************************************************/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC1     ------> SPI2_MOSI
    PC2     ------> SPI2_MISO
    PB10    ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_2);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    /* SPI2 DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);

    /* SPI2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  }

}



/**************************************************************************
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
***************************************************************************/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin 		= USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull 		= GPIO_NOPULL;
    GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate 	= GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance 				= DMA1_Stream5;
    hdma_usart2_rx.Init.Channel 			= DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc 			= DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc 				= DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment 	= DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode 				= DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority 			= DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode 			= DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
    	Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }

}

/****************************************************************************
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
***************************************************************************/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	if(huart->Instance==USART2)
	{
	/* Peripheral clock disable */
	__HAL_RCC_USART2_CLK_DISABLE();

	/**USART2 GPIO Configuration
	PA2     ------> USART2_TX
	PA3     ------> USART2_RX
	*/
	HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

	/* USART2 DMA DeInit */
	HAL_DMA_DeInit(huart->hdmarx);

	/* USART2 interrupt DeInit */
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	}
}

/****************************************************************************
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*****************************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if(htim_base->Instance==TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 13, 0);
		HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	}
}

/****************************************************************************
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*****************************************************************************/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
	if(htim_base->Instance==TIM1)
	{
		__HAL_RCC_TIM1_CLK_DISABLE();
		HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
	}
}
