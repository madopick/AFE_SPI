/*
 * spi_slave_driver.h
 *
 *  Created on: Feb 20, 2023
 *      Author:
 */

#ifndef SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_
#define SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_

#include "main.h"

#define SPI_TX_BUFF_LEN                              608
#define SPI_RX_BUFF_LEN                              608


#define SPI_WRITE_CPLT								 (1 << 0)
#define SPI_READ_CPLT								 (1 << 1)
#define SPI_WR_UPDATE								 (1 << 2)
#define SPI_GPIO_INIT								 (1 << 3)
#define SPI_TIMEOUT									 (1 << 4)




/************************************************************
 * @brief   SPI Write Packet Bytes
 *
 ************************************************************/
//Byte 0
#define SPI_MODE_BYTE							0x00
#define SPI_WRITE_REQ							0x05
#define SPI_READ_REQ							0x0A

//================================================================//

//Byte 1 - WRITE COMMAND
#define SPI_WRITE_CMD_BYTE						0x01


#define SPI_SCAN_ON								0x10		//Scan ON.
#define SPI_SCAN_OFF							0x11		//Scan OFF.
#define SPI_SCAN_NOISE							0x12		//Start Scan Noise.
#define SPI_SCAN_SELF_TX						0x13		//Start Scan Self TX.
#define SPI_SCAN_SELF_RX						0x14		//Start Scan Self RX.
#define SPI_SCAN_MUTUAL							0x15		//Start Scan Mutual.


//Byte 2 - WRITE PARAMETER
#define SPI_WRITE_PRM_BYTE						0x02




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
	SPI_TO_OP
} eSPIop_t;

typedef struct
{
	SPIPeriphCallback 		vf_callback;
	uint8_t*				u8p_Sentbuf;
	uint8_t*				u8p_Rcvbuf;
	uint16_t 				u16_len;
} spiAFE_s;



uint8_t u8Spi_Slave_init(void);
uint8_t u8Spi_Gpio_Init(void);
uint8_t u8Spi_Slave_rcvOnly(uint8_t *u8p_RcvBuff, uint16_t u16_len);
uint8_t u8Spi_Slave_sendOnly(uint8_t *u8p_SendBuff, uint16_t u16_len);
uint8_t u8Spi_Slave_sendRcv(uint8_t *u8p_Senddata, uint8_t *u8p_Rcvdata, uint16_t length);

void SPI_Callback(eSPIop_t eOps);

#endif /* SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_ */
