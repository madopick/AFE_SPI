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


/********************************************
  * @name   SPI_GPIO pins setup
  * @brief
  *******************************************/
uint8_t u8Spi_Slave_init(void)
{
	hspi2.Instance 					= SPI2;
	hspi2.Init.Mode 				= SPI_MODE_SLAVE;
	hspi2.Init.Direction 			= SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize 			= SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPhase 			= SPI_PHASE_1EDGE;
	hspi2.Init.CLKPolarity 			= SPI_POLARITY_LOW;

#ifdef NSS_HW
	hspi2.Init.NSS 					= SPI_NSS_HARD_INPUT
#else
	hspi2.Init.NSS 					= SPI_NSS_SOFT;
#endif

	hspi2.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode 				= SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation 		= SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial 		= 10;

	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}


	spiAFE.u16_len 		= 0;
	spiAFE.u8p_Rcvbuf	= NULL;
	spiAFE.u8p_Sentbuf	= NULL;
	spiAFE.vf_callback	= NULL;

	return (HAL_OK);
}


/********************************************
  * @name   SPI_GPIO pins setup
  * @brief
  *******************************************/
uint8_t u8Spi_Gpio_Init(void)
{
	///AFE GPIO PIN SETUP
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin 	= GPIO_PIN_3;
	GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull 	= GPIO_NOPULL;
	GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

	return (HAL_OK);
}


/********************************************
  * @name   SPI_slave_receive only
  * @brief 	SPI slave receive Only Function
  *******************************************/
uint8_t u8Spi_Slave_rcvOnly(uint8_t *u8p_RcvBuff, uint16_t u16_len)
{
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
	{
		HAL_Delay(10);
	}

	spiAFE.u8p_Rcvbuf 	= u8p_RcvBuff;
	spiAFE.u16_len		= u16_len;

	if (HAL_SPI_Receive_IT(&hspi2,
							(uint8_t *)spiAFE.u8p_Rcvbuf,
							SPI_RX_BUFF_LEN) != HAL_OK)

	{
		Error_Handler(__FILE__, __LINE__);
	}

	return HAL_OK;
}




/********************************************
  * @name   SPI_slave_send only
  * @brief 	SPI slave Send Only Function
  *******************************************/
uint8_t u8Spi_Slave_sendOnly(uint8_t *u8p_SendBuff, uint16_t u16_len)
{
//	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
//	{
//		HAL_Delay(1);
//	}

	spiAFE.u8p_Sentbuf 	= u8p_SendBuff;
	spiAFE.u16_len		= u16_len;

	if (HAL_SPI_Transmit_IT(&hspi2,
							(uint8_t *)spiAFE.u8p_Sentbuf,
							SPI_TX_BUFF_LEN) != HAL_OK)

	{
		Error_Handler(__FILE__, __LINE__);
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

	return HAL_OK;
}


/********************************************
  * @name   SPI_slave_send receive
  * @brief 	SPI slave Send Receive Function
  *******************************************/
uint8_t u8Spi_Slave_sendRcv(uint8_t *u8p_Senddata, uint8_t *u8p_Rcvdata, uint16_t length)
{
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
	{
		HAL_Delay(10);
	}

	spiAFE.u8p_Sentbuf 	= u8p_Senddata;
	spiAFE.u8p_Rcvbuf 	= u8p_Rcvdata;
	spiAFE.u16_len		= length;

	if (HAL_SPI_TransmitReceive_IT(&hspi2,
									(uint8_t *)spiAFE.u8p_Sentbuf,
									(uint8_t *)spiAFE.u8p_Rcvbuf,
									spiAFE.u16_len) != HAL_OK)

	{
		Error_Handler(__FILE__, __LINE__);
	}

	//printf("SPI: [%d] - %s\r\n", u8_spiSendBuff[0], (char*)&u8_spiSendBuff[1]);

	return HAL_OK;
}



void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		printf("TX CB\r\n");
		SPI_Callback(SPI_WRITE_OP);
	}
}



void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		printf("RX CB\r\n");
		SPI_Callback(SPI_READ_OP);
	}
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		printf("TXRX CB: %s\r\n", spiAFE.u8p_Rcvbuf);
		SPI_Callback(SPI_WRnRD_OP);
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

/*****************************************************************************
  * @brief This function handles SPI2 global interrupt.
  ****************************************************************************/
void SPI2_IRQHandler(void)
{
	HAL_SPI_IRQHandler(&hspi2);
}


/*****************************************************************************
  * @brief  SPI error callbacks
  * @param  hspi: SPI handle
  * @retval None
  ***************************************************************************/
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	 switch (hspi->ErrorCode)
	 {
		 case HAL_SPI_ERROR_MODF:
			 printf("SPIE_MODF\r\n");
			 break;

		 case HAL_SPI_ERROR_CRC:
			 printf("SPIE_CRC\r\n");
			 break;

		 case HAL_SPI_ERROR_OVR:
			 printf("SPIE_OVR\r\n");
			 break;

		 case HAL_SPI_ERROR_FRE:
			 printf("SPIE_FRE\r\n");
			 break;

		 case HAL_SPI_ERROR_DMA:
			 printf("SPIE_DMA\r\n");
			 break;

		 case HAL_SPI_ERROR_FLAG:
			 printf("SPIE_FLAG\r\n");
			 break;

		 case HAL_SPI_ERROR_ABORT:
			 printf("SPIE_ABORT\r\n");
			 break;

		 default:
			 printf("SPIE_DEF\r\n");
			 break;
	 }

	 SPI_Callback(SPI_TO_OP);
}




