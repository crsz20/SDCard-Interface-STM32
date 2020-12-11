/*
 * stm32f466xx_spi_driver.h
 *
 *  Created on: Nov 21, 2020
 *      Author: cris1
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_


#include "stm32f446xx.h"





//Configuration Structure for SPIx Peripheral
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


//Handle Structure for SPIx Peripheral
typedef struct
{
	SPI_RegDef_t	*pSPIx;			//This holds the base address of SPIx peripherals
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;		//Stores the TX buffer address
	uint8_t			*pRxBuffer;		//Stores the RX buffer address
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;

//Application States
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2


//Possible SPI Application Events
#define SPI_EVENT_TX_CMPLT			0
#define SPI_EVENT_RX_CMPLT			1

#define SPI_EVENT_OVR_ERR			2
#define SPI_EVENT_CRC_ERR			3

//@SPI_DeviceMode
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0



//@SPI_BusConfig
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3



//@SPI_SclkSpeed
//Bits 5:3 BR[2:0]: Baud rate control
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7


//@SPI_DFF
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1


//@SPI_CPOL
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0


//@SPI_CPHA
#define SPI_CPHAL_HIGH		1
#define SPI_CPHA_LOW		0


//@SPI_SSM
#define SPI_SSM_EN			1
#define SPI_SSM_DI			0




//SPI related status flag definitions
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)



/************************************************************************************************************************
 * 											APIs supported by this driver
 * 							For more information about the APIs, check the function definitions
 ************************************************************************************************************************/

//Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


//Initialization & De-Initialization
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


//Data Send & Receive ("IT" refers to interrupt-based)
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


//IRQ Configuration & ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


//Other Peripheral Control APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


//Application callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
