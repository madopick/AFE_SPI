/*
 * spi_slave_driver.c
 *
 *  Created on: Feb 20, 2023
 *      Author:
 */


#include "spi_slave_driver.h"


///External Variable
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;


static spiAFE_s spiAFE;

static uint8_t u8_spiSendBuff[SPI_BUFF_LEN];
static uint8_t u8_spiRcvBuff[SPI_BUFF_LEN];


uint8_t u8Spi_Slave_init(void)
{
	hspi2.Instance 					= SPI2;
	hspi2.Init.Mode 				= SPI_MODE_SLAVE;
	hspi2.Init.Direction 			= SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize 			= SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPhase 			= SPI_PHASE_1EDGE;
	hspi2.Init.CLKPolarity 			= SPI_POLARITY_LOW;
	hspi2.Init.NSS 					= SPI_NSS_SOFT;
	hspi2.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode 				= SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation 		= SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial 		= 10;

	memset(u8_spiRcvBuff, 0, sizeof(u8_spiRcvBuff));

	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	spiAFE.e_SPIop 		= SPI_IDLE_OP;
	spiAFE.u16_lenCnt 	= 0;
	spiAFE.u16_lenThrs 	= 0;
	spiAFE.u8p_Rcvbuf	= NULL;
	spiAFE.u8p_Sentbuf	= NULL;
	spiAFE.vf_callback	= NULL;


	if(HAL_SPI_TransmitReceive_DMA(&hspi2,
									(uint8_t *)u8_spiSendBuff,
									(uint8_t *)u8_spiRcvBuff,
									SPI_BUFF_LEN) != HAL_OK)

	{
		Error_Handler(__FILE__, __LINE__);
	}


	///AFE GPIO PIN SETUP
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin 	= GPIO_PIN_3;
	GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 	= GPIO_PULLUP;
	GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

	return (HAL_OK);
}



/********************************************
  * @name   SPI_slave_send
  * @brief 	SPI slave Send Function
  *******************************************/
uint8_t u8Spi_Slave_send(uint8_t *u8p_data, uint16_t length)
{
	memcpy(u8_spiSendBuff, u8p_data, length);

	//printf("SPI: [%d] - %s\r\n", u8_spiSendBuff[0], (char*)&u8_spiSendBuff[1]);

	return HAL_OK;
}


uint8_t u8Spi_Slave_run(void)
{
	if(HAL_SPI_TransmitReceive_DMA(&hspi2,
									(uint8_t *)u8_spiSendBuff,
									(uint8_t *)u8_spiRcvBuff,
									SPI_BUFF_LEN) != HAL_OK)

	{
		Error_Handler(__FILE__, __LINE__);
	}

	return HAL_OK;
}



void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		printf("TX CB\r\n");
		void SPI_Callback(eSPIop_t eOps);
		SPI_Callback(SPI_WRITE_CPLT);
	}
}



void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		printf("RX CB: %s\r\n", u8_spiRcvBuff);
		memset(u8_spiRcvBuff, 0, SPI_BUFF_LEN);
		SPI_Callback(SPI_READ_CPLT);
	}
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		printf("TXRX CB: %s\r\n", u8_spiRcvBuff);
		memset(u8_spiRcvBuff, 0, SPI_BUFF_LEN);
		SPI_Callback(SPI_READ_CPLT);
	}
}


/************************************************************
  * @brief  DMA TX & RX
  * @param
  * @retval None
  ***********************************************************/
void DMA1_Stream3_IRQHandler (void)
{
	HAL_DMA_IRQHandler(hspi2.hdmarx);
}

void DMA1_Stream4_IRQHandler (void)
{
	HAL_DMA_IRQHandler(hspi2.hdmatx);
}





