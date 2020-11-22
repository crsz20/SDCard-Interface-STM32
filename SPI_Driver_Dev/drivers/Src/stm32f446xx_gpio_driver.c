/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Nov 15, 2020
 *      Author: cris1
 */


#include "stm32f446xx_gpio_driver.h"



//Peripheral Clock setup
/********************************************************************************
 * Function:		GPIO_PeriClockControl
 *
 * Description:		Enables/disables peripheral clock for the given GPIO port
 *
 * Inputs:			*pGPIOx - base address of the GPIO peripheral
 * 					EnorDi - ENABLE or DISABLE macros
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

	} else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}






//Initialization
/********************************************************************************
 * Function:
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp; //temp. register will at first hold the desired GPIO config setting, with the bits set at the desired register

	// Mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//Non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clear register
		pGPIOHandle->pGPIOx->MODER |= temp;   //Handle variable will get the base address and access the speed register. Set register

	} else
	{
		//Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Configure the Rising Trigger Selection Register
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//Configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configure the GPIO port selection in SYSCFG_EXTIR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		//Enable the EXTI interrupt delivery via Interrupt Mask Register
		EXTI->IMR |= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;

	//Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp = 0;

	//Pull up pull down
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//Output type - multiplying by 2 is only required for 2-bit registers
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//Alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;		//Get high or low alt function registers via 2-element array
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;		//Get the bits (4-bit registers)
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}


}



//De-Initialization
/********************************************************************************
 * Function:		GPIO_DeInit
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

		if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}

}


//Data read/write
/********************************************************************************
 * Function:		GPIO_ReadFromInputPin
 *
 * Description:
 *
 * Inputs:			value - 0 or 1
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)( (pGPIOx->IDR >> PinNumber) & 0x00000001 );

	return value;
}


/********************************************************************************
 * Function:		GPIO_ReadFromInputPort
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}


/********************************************************************************
 * Function:		GPIO_WriteToOutputPin
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin num
		pGPIOx->ODR |= ( 1 << PinNumber);
	} else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}


/********************************************************************************
 * Function:		GPIO_WriteToOutputPort
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/********************************************************************************
 * Function:		GPIO_ToggleOutputPin
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);
}


//IRQ config & ISR handling
/********************************************************************************
 * Function:		GPIO_IRQConfig
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


/********************************************************************************
 * Function:		GPIO_IRQPriorityConfig
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NUM_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + ( iprx * 4)) |= ( IRQPriority << shift_amount );

}



/********************************************************************************
 * Function:		GPIO_IRQHandling
 *
 * Description:
 *
 * Inputs:
 *
 * Return:
 *
 * Note:
 ********************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & ( 1 << PinNumber) )
	{
		//Clear
		EXTI->PR |= ( 1 << PinNumber);	//Use 1 to clear this time, per register details
	}
}









