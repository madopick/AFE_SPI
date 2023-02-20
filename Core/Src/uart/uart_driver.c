/*
 * uart_driver.c
 *
 *  Created on: Feb 20, 2023
 *      Author:
 */

#include "uart_driver.h"


UART_HandleTypeDef 	huart2;
DMA_HandleTypeDef 	hdma_usart2_rx;


static uint8_t u8arr_eventBuff[UART_BUF_SZ];
static uint8_t u8arr_uartEvent[UART_BUF_SZ];

/* Private user code ---------------------------------------------------------*/
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}



/**********************************************************
 * PARSING HEADER, Used in FW CONFIG - READ/WRITE Process
 **********************************************************/
#define CFG_LENGTH 				10
#define CFG_HEADER_NUM 			6
#define CFG_HEADER_CHARS_LEN 	5
#define CFG_HEADER_READ 		5
#define STRLENMAX				100

static char str_cfg_header[CFG_HEADER_NUM][CFG_HEADER_CHARS_LEN] =
{
	"{MSG:",
	"{CF1:",
	"{CF2:",
	"{CF3:",
	"{CF4:",
	"{RD1}"
};


int32_t i32_res[CFG_LENGTH] = {10,256,512,37,10,-45,123,46,-78,89};





/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void Uart_Init(void)
{
  huart2.Instance 			= USART2;
  huart2.Init.BaudRate 		= 115200;
  huart2.Init.WordLength 	= UART_WORDLENGTH_8B;
  huart2.Init.StopBits 		= UART_STOPBITS_1;
  huart2.Init.Parity 		= UART_PARITY_NONE;
  huart2.Init.Mode 			= UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;
  huart2.Init.OverSampling 	= UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, u8arr_eventBuff, UART_BUF_SZ);
}



/*********************************************************************
 * @name	: tinysh_dec
 * @brief	: string to decimal conversion (up to 15 chars).
 *********************************************************************/
unsigned long tinysh_dec(char *s)
{
  unsigned long res=0;
  uint8_t index = 0;
  int8_t min	= 1;

  while(*s)
  {
	  //printf("%c\r\n",*s);

	  res*=10;

	  if((*s == '-')&&(index == 0))
		  min = -1;
	  else if((*s == '0')&&(index == 0))
		  res = 0;
	  else if(*s>='0' && *s<='9')
		  res+=*s-'0';
	  else
		  break;

	  s++;
	  index++;

	  if(index > 15)
	  {
		 break;
	  }
  }

  return (res * min);
}



/********************************************************
 * 	Parsing incoming message						   	*
 * 	Example: {MSG:1;23;21009}							*
 * 			 {RD1}
 ********************************************************/
static void vShell_cmdParse(char *input)
{
	for(uint8_t u8_idx = 0; u8_idx < CFG_HEADER_NUM; u8_idx++)
	{
		if(!memcmp(input,(char*)&str_cfg_header[u8_idx][0], CFG_HEADER_CHARS_LEN))
		{
			char *pChar 		= &input[CFG_HEADER_CHARS_LEN];
			char *pChar2 		= &input[CFG_HEADER_CHARS_LEN];
			uint8_t u8_start 	= 0;
			uint8_t u8_stop 	= 0;
			uint8_t u8_cnt 		= 0;

			char str_res[20];

			puts("\r\n");

			if (u8_idx < CFG_HEADER_READ)
			{
				/* WRITE HEADER */
				while (*pChar)
				{
					if(*pChar == ';')
					{
						memset(&str_res[0], 0, 10);
						memcpy(&str_res[0], &pChar2[u8_stop], u8_start - u8_stop);
						i32_res[u8_cnt] = tinysh_dec(&str_res[0]);
						printf("val: %s - %ld\r\n", &str_res[0], i32_res[u8_cnt]);

						u8_stop = u8_start + 1;
						u8_cnt++;
					}
					else if (*pChar == '}')
					{
						memset(&str_res[0], 0, 10);
						memcpy(&str_res[0], &pChar2[u8_stop], u8_start - u8_stop);
						i32_res[u8_cnt] = tinysh_dec(&str_res[0]);
						printf("val: %s - %ld\r\n", &str_res[0], i32_res[u8_cnt]);

						u8_cnt++;
						break;
					}

					pChar++;
					u8_start++;
				}
				break;

			}
			else
			{
				/* READ HEADER */
				char sendStr[STRLENMAX];
				memset (sendStr, 0, STRLENMAX);
				snprintf(sendStr, STRLENMAX, "READ:%ld;%ld;%ld",i32_res[0] , i32_res[1], i32_res[2]);
				HAL_UART_Transmit(&huart2, (uint8_t *)sendStr, strlen(sendStr), 0xFFFF);
			}
		}
	}

}


/*****************************************************************
 * @name
 * @brief
 ****************************************************************/
void uartProcessing (uint8_t *u8p_buffer, uint16_t u16_size)
{
	//printf("UART RX(%d): %s\r\n", u16_size, (char*)u8p_buffer);
	vShell_cmdParse((char*)u8p_buffer);
}


/*****************************************************************
 * @name 	vUAFE_uart_handle
 * @brief	handle afe uart data copy
 ****************************************************************/
static void vUAFE_uart_handle(uint16_t Size)
{
	static uint16_t u16_oldPos = 0;
	static uint16_t u16_lenCnt = 0;
	uint16_t u16_numData;

	//printf("S(%d): %s\r\n", Size, (char*)u8arr_eventBuff);


	/* Check if number of received data in reception buffer has changed */
	if (Size != u16_oldPos)
	{
		if (Size > u16_oldPos)
		{
			/* Current position is higher than previous one */
			u16_numData = Size - u16_oldPos;
			memcpy(&u8arr_uartEvent[u16_lenCnt],&u8arr_eventBuff[u16_oldPos],u16_numData);
			u16_lenCnt += u16_numData;
		}
		else
		{
			/* End of buffer has been reached */
			u16_numData = UART_BUF_SZ - u16_oldPos;

			memcpy (&u8arr_uartEvent[u16_lenCnt], 			// copy data in that remaining space
					&u8arr_eventBuff[u16_oldPos],
					u16_numData);

			u16_lenCnt += u16_numData;

			memcpy (&u8arr_uartEvent[u16_lenCnt], 			// copy the remaining data
					&u8arr_eventBuff[0],
					Size);

			u16_lenCnt += Size;
		}

		/* Check for ready to process */
		if((u8arr_uartEvent[u16_lenCnt - 1] == '\n')&&(u8arr_uartEvent[u16_lenCnt - 2]== '\r'))
		{
			uartProcessing (u8arr_uartEvent, u16_lenCnt - 2); // remove \r & \n
			memset(u8arr_uartEvent, 0, UART_BUF_SZ);
			u16_lenCnt = 0;
		}

	}


	u16_oldPos = Size;
}



/*****************************************************************
 * @name HAL_UARTEx_RxEventCallback
 * @brief
 ****************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		vUAFE_uart_handle(Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, u8arr_eventBuff, UART_BUF_SZ);
	}
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}




