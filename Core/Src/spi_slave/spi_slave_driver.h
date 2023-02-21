/*
 * spi_slave_driver.h
 *
 *  Created on: Feb 20, 2023
 *      Author:
 */

#ifndef SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_
#define SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_

#include "main.h"

#define SPI_TX_BUFF_LEN                              10
#define SPI_RX_BUFF_LEN                              10


#define SPI_WRITE_CPLT								 (1 << 0)
#define SPI_READ_CPLT								 (1 << 1)
#define SPI_WR_UPDATE								 (1 << 2)


/************************************************************
 * @brief   SPI callback typedef
 *
 ************************************************************/
typedef void (*SPIPeriphCallback)(void);


///Slave Operation
typedef enum
{
	SPI_IDLE_OP,
	SPI_READ_OP,
	SPI_WRITE_OP,
	SPI_WRnRD_OP,
} eSPIop_t;

typedef struct
{
	SPIPeriphCallback 		vf_callback;
	uint8_t*				u8p_Sentbuf;
	uint8_t*				u8p_Rcvbuf;
	uint16_t 				u16_len;
} spiAFE_s;



uint8_t u8Spi_Slave_init(void);
uint8_t u8Spi_Slave_rcvOnly(uint8_t *u8p_RcvBuff, uint16_t u16_len);
uint8_t u8Spi_Slave_sendOnly(uint8_t *u8p_SendBuff, uint16_t u16_len);
uint8_t u8Spi_Slave_sendRcv(uint8_t *u8p_Senddata, uint8_t *u8p_Rcvdata, uint16_t length);
uint8_t u8Spi_Slave_run(void);
void SPI_Callback(eSPIop_t eOps);

#endif /* SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_ */
