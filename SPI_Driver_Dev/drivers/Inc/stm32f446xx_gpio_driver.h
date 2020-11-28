/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Nov 15, 2020
 *      Author: cris1
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_


#include "stm32f446xx.h"


//Config settings for a GPIO pin
typedef struct
{
	uint8_t GPIO_PinNumber;					//Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;					//Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;					//Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;			//Possible values from @GPIO_PIN_PUPD_CONFIG
	uint8_t GPIO_PinOPType;					//Possible values from @GPIO_PIN_OUTPUT_TYPE
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;




//Handle structure for a GPIO pin
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				//A pointer that holds the base address of the GPIO PORT to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//This variable holds GPIO pin config settings

}GPIO_Handle_t;



//@GPIO_PIN_NUMBERS
#define GPIO_PIN_NUM_0			0
#define GPIO_PIN_NUM_1			1
#define GPIO_PIN_NUM_2			2
#define GPIO_PIN_NUM_3			3
#define GPIO_PIN_NUM_4			4
#define GPIO_PIN_NUM_5			5
#define GPIO_PIN_NUM_6			6
#define GPIO_PIN_NUM_7			7
#define GPIO_PIN_NUM_8			8
#define GPIO_PIN_NUM_9			9
#define GPIO_PIN_NUM_10			10
#define GPIO_PIN_NUM_11			11
#define GPIO_PIN_NUM_12			12
#define GPIO_PIN_NUM_13			13
#define GPIO_PIN_NUM_14			14
#define GPIO_PIN_NUM_15			15



//@GPIO_PIN_MODES
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4	//Input mode, falling edge (interrupt)
#define GPIO_MODE_IT_RT			5	//Input mode, rising edge (interrupt)
#define GPIO_MODE_IT_RFT		6	//Input mode, rising edge falling edge (interrupt)



//@GPIO_PIN_OUTPUT_TYPE
//GPIO pin possible output types
#define GPIO_OP_TYPE_PP			0	//Output mode, push pull
#define GPIO_OP_TYOE_OD			1	//Output mode, open drain



//@GPIO_PIN_SPEED
#define GPIOS_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3



//@GPIO_PIN_PUPD_CONFIG
//GPIO pin and pull up pull down config
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2



/************************************************************************************************************************
 * 											APIs supported by this driver
 * 							For more information about the APIs, check the function definitions
 ************************************************************************************************************************/

//Peripheral Clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


//Initialization & De-Initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


//Data read/write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


//IRQ Configuration & ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
















#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
