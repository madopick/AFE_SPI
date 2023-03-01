/*
 * spi_slave_driver.h
 *
 *  Created on: Feb 20, 2023
 *      Author:
 */

#ifndef SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_
#define SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_

#include "main.h"


#define SPI_TX_BUFF_LEN                              (16*18)+2
#define SPI_RX_BUFF_LEN                              5				//10 bytes of uint8_t


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

typedef enum
{
	AFE_REQ_WRITE = 0x05,
	AFE_REQ_READ  = 0x0A,
} eAFE_REQ;

//================================================================//

//Byte 1 - WRITE COMMAND
#define SPI_WRITE_CMD_BYTE						0x01

typedef enum
{
	AFE_CMD_ON = 0x10,							//Scan ON.
	AFE_CMD_OFF,								//Scan OFF.
	AFE_CMD_SCAN_NOISE,							//Start Scan Noise.
	AFE_CMD_SCAN_SELF_TX,						//Start Scan Self TX.
	AFE_CMD_SCAN_SELF_RX,						//Start Scan Self RX.
	AFE_CMD_SCAN_MUTUAL,						//Start Scan Mutual.
} eAFE_CMD;

typedef enum
{
	AFE_FREQ_500_KHz,
	AFE_FREQ_400_KHz,
	AFE_FREQ_250_KHz,
	AFE_FREQ_100_KHz
} eAFE_FREQ;

//================================================================//
//Byte 2 - WRITE PARAMETER
#define SPI_WRITE_PRM_BYTE						0x02

typedef struct
{
	union
	{
		uint8_t u8_data[10];
		struct
		{
			eAFE_REQ u8_mode;
			eAFE_CMD u8_cmd;
			eAFE_FREQ u8_freq;
			uint8_t u8_txCnt;
			uint8_t u8_accCnt;

			uint8_t u8_isVref:1;
			uint8_t u8_isDiff:1;
		};
	};
} sAfeCmd_t;


typedef struct
{
	union
	{
		uint8_t u8_data[2];
		struct
		{
			uint8_t u8_isOk;
			eAFE_CMD u8_cmd;
		};
	};
} sAfeHeader_t;



/*****************************************************************
 * noise scan has no parameters
 *****************************************************************/

/*****************************************************************
 * self scan RX has no parameters:
 *****************************************************************/

/*****************************************************************
 * self scan TX has 1 parameters:
 * - number of TX 				<0~16>
 *****************************************************************/

/*****************************************************************
 * mutual scan has 4 parameters:
 * - VREF 						<0: For floating Ref Mode>
 * 								<1: For non floating Ref Mode>
 * - FREQ						<0: For 500 KHz>
 * 								<1: For 400 KHz>
 * 								<2: For 250 KHz>
 * 								<3: For 100 KHz>
 * - Number of TX 				<0~16>
 * - Number of Accmltn Cycle	<0~64>
 *****************************************************************/

//================================================================//




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
	int16_t*				i16p_Sentbuf;
	int16_t*				i16p_Rcvbuf;
	uint16_t 				u16_len;
} spiAFE_s;



uint8_t u8Spi_Slave_init(void);
uint8_t u8Spi_Gpio_Init(void);
uint8_t u8Spi_Slave_rcvOnly(int16_t *i16p_RcvBuff, uint16_t u16_len);
uint8_t u8Spi_Slave_sendOnly(int16_t *i16p_SendBuff, uint16_t u16_len);
uint8_t u8Spi_Slave_sendRcv(int16_t *i16p_Senddata, int16_t *i16p_Rcvdata, uint16_t length);
uint8_t u8Spi_Slave_run(void);

void SPI_Callback(eSPIop_t eOps);

#endif /* SRC_SPI_SLAVE_SPI_SLAVE_DRIVER_H_ */
