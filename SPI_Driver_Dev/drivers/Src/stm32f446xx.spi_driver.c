/*
 * stm32f446xx.spi_driver.c
 *
 *  Created on: Nov 21, 2020
 *      Author: cris1
 */


#include "stm32f446xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


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
	while(Len > 0)
		{
			//Wait until RXNE is set
			while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			//Check the DFF bit in CR1
			if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//Load the data from DR to RxBuffer address
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			} else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}

		}

}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//Save the TX buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//Mark the SPI state as busy in transmission, so no other code can take over the same
		//SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR1 = ( 1 << SPI_CR2_TXEIE );

		//Data transmission handled by the ISR code

	}

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//Save the TX buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//Mark the SPI state as busy in transmission, so no other code can take over the same
		//SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR1 = ( 1 << SPI_CR2_RXNEIE );

		//Data transmission handled by the ISR code

	}

	return state;
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



void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
		{
			pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		} else
		{
			pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}

}


//IRQ Configuration & ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//Program ISER0 register
				*NVIC_ISER0 |= ( 1 << IRQNumber );

			}else if (IRQNumber > 31 && IRQNumber < 64)
			{
				//Program ISER1 register
				*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

			}else if (IRQNumber >= 64 && IRQNumber < 64)
			{
				//Program ISER2 register
				*NVIC_ISER2 |= ( 1 << (IRQNumber % 32) );

			}
		} else
		{
			if (IRQNumber <= 31)
			{
				//Program ICER0 register
				*NVIC_ICER0 |= ( 1 << IRQNumber );

			} else if (IRQNumber > 31 && IRQNumber < 64 )
			{
				//Program ICER1 register
				*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );

			}else if (IRQNumber >= 6 && IRQNumber < 96 )
			{
				//Program ICER2 register
				*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
			}

		}

}


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + ( iprx * 4)) |= ( IRQPriority << shift_amount );
}






void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	//Check why the Interrupt occurred. First check for TXE
	uint8_t temp1, temp2;

	//If TXE flag is set, temp1 = 1
	//If TXE flag is reset, temp1 = 0
	//If both variables are true, then this interrupt is caused by the setting of the TXE flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);

	}


	//Check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);

	}




	//Check for OVR flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		spi_ovr_interrupt_handle(pHandle);

	}



}


//Helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//Load the data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}


	if(! pSPIHandle->TxLen)
	{
		//If TxLen = 0, close SPI communication
		//Deactivate TXIE bit to prevent further interrupts from TXIE flag
		SPI_CloseTransmission(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}


}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Check the DFF bit in CR1
	if( pSPIHandle->pSPIx->CR1 & ( 1 << 11) )
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;

	} else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;

	}


	if(! pSPIHandle->RxLen)
	{
		//Reception is complete. Turn off the RXNEIE interrupt
		SPI_CloseReception(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}

}


static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Clear OVR flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->DR;

	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);



}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//Weak implementation --- the application may override this function
	//If the application doesn't implement this callback function, then this callback
	//function will be called
}



