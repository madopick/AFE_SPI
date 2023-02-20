/*
 * spi_slave_driver.h
 *
 *  Created on: Feb 20, 2023
 *      Author:
 */

#ifndef SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_
#define SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_

#include "main.h"

#define SPI_BUFF_LEN                                  ((uint8_t)0x05)


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
} eSPIop_t;

typedef struct
{
	SPIPeriphCallback 		vf_callback;
	uint8_t*				u8p_Sentbuf;
	uint8_t*				u8p_Rcvbuf;
	eSPIop_t				e_SPIop;
	uint16_t 				u16_lenThrs;
	uint16_t 				u16_lenCnt;
} spiAFE_s;



uint8_t spi_slave_init(void);
uint8_t u8Func_spi_slave_send(uint8_t *u8p_data, uint16_t length);

#endif /* SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_ */
