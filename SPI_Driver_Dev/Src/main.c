 /*
 * 001led_toggle.c
 *
 *  Created on: Nov 16, 2020
 *      Author: cris1
 */


/*
 * PB14 = SPI2_MISO
 * PB15 = SPI2_MOSI
 * PB13 = SPI2_SCLK
 * PB12 = SPI2_NSS
 * ALT function mode: 5
 */



#include "stm32f446xx.h"
#include <string.h>


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;		//If driving another slave, it is best to activate the internal pull ups
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_15;
	GPIO_Init(&SPIPins);


	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GPIO_Init(&SPIPins);

}



void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;			//Generates SCLK of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}


int main(void)
{

	//Initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//Initializes the SPI2 peripheral parameters
	SPI2_Inits();

	//Makes NSS signal internally high and avoid MODF error
	//SPI_SSIConfig(SPI2, ENABLE);



	/*
	 * If SSOE = 1, then NSS output is enabled
	 * The NSS pin is automatically managed by the hardware
	 * (i.e. SPE = 1, so NSS will be pulled to low & vice versa)
	 */
	SPI_SSOEConfig(SPI2, ENABLE);


	//Enable SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//Send data
	char user_data[] = "Hello World!";
	SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));



	//Confirm that SPI is not busy
	while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

	//Disable SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);



	return 0;
}


