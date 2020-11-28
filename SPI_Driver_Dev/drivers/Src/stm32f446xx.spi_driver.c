/*
 * stm32f446xx.spi_driver.c
 *
 *  Created on: Nov 21, 2020
 *      Author: cris1
 */


#include "stm32f446xx_spi_driver.h"




//Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}


	} else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}





//Initialization & De-Initialization
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Configure SPI_CR1 register

	uint32_t tempreg = 0;

	//Configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//Configure the bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Bi-directional mode should be cleared
		tempreg &= ~(1 << 15);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//Bi-directional mode should be set
		tempreg |= (1 << 15);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BI-directional mode should be cleared
		tempreg &= ~(1 << 15);

		//Set RXONLY bit
		tempreg |= (1 << 10);
	}


	//Configure the SPI serial clock speed (Baud Rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	//Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	//Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;


	pSPIHandle->pSPIx->CR1 = tempreg;
}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}

}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{

	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


//Data Send & Receive
//Blocking Call
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//Wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//Check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//Load the data into DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		} else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}



}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		} else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}

}


//IRQ Configuration & ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}







