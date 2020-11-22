/*
 * stm32f446xx.h
 *
 *  Created on: Nov 15, 2020
 *      Author: cris1
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_




#include <stdint.h>

#define __vo volatile


/************************************START: Processor Specific Details******************************************/

//ARM Cortex M4 Processor NVIC ISERx Register addresses
#define NVIC_ISER0				( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1				( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2				( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3				( (__vo uint32_t*)0xE000E10C )


//ARM Cortex M4 Processor NVIC ICERx Register addresses
#define NVIC_ICER0				( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1				( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2				( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3				( (__vo uint32_t*)0XE000E18C )


//ARM Cortex M4 Processor NVIC ICERx Register addresses
#define NVIC_PR_BASEADDR		( (__vo uint32_t*)0xE000E400)


//ARM Cortex M4 Processor number of priority bits implemented in Priority Register
#define NUM_PR_BITS_IMPLEMENTED			4



/************************************START: MCU Specific Details******************************************/


//--------------Base addresses of Flash and SRAM memories-----------------

#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x20001C00U
#define ROM_BASEADDR					0x1FFF0000	//System memory, 112KB
#define SRAM							SRAM1_BASEADDR


//--------------APBx & AHBx Bus Peripheral base addresses-----------------

#define PERIPH_BASE						0x40000000U
#define APB1PERIPH_BASE					PERIPH_BASE
#define APB2PEIRH_BASE					0x40010000U

#define AHB1PERIPH_BASE					0x40020000U
#define AHB2PERIPH_BASE					0x50000000U


//--------------Base addresses of peripherals which are hanging on AHB1 bus--------------
//(base address + offset)

#define GPIOA_BASEADDR					(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASE + 0X0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASE + 0X0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASE + 0X0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASE + 0X1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASE + 0X1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASE + 0X1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASE + 0X1C00)
#define RCC_BASEADDR					(AHB1PERIPH_BASE + 0x3800)


//--------------Base addresses of peripherals which are hanging on APB1 bus--------------
//(base address + offset)

#define I2C1_BASEADDR					(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASE + 0x4800)

#define UART4_BASEADDR					(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASE + 0x5000)





//--------------Base addresses of peripherals which are hanging on APB2 bus--------------
//(base address + offset)

#define EXTI_BASEADDR					(APB2PEIRH_BASE + 0x3C00)
#define SYSCFG_BASEADDR					(APB2PEIRH_BASE + 0x3800)

#define SPI1_BASEADDR					(APB2PEIRH_BASE + 0x3000)

#define USART1_BASEADDR					(APB2PEIRH_BASE + 0x1000)
#define USART6_BASEADDR					(APB2PEIRH_BASE + 0x1400)





/************************************Peripheral Register Definition Structures******************************************/

//Peripheral Register Definition Structure for GPIO
typedef struct
{
	__vo uint32_t MODER;			//GPIO port mode register										Address offset: 0x00
	__vo uint32_t OTYPER;			//GPIO port output type register								Address offset: 0x04
	__vo uint32_t OSPEEDER;			//GPIO port output speed register								Address offset: 0x08
	__vo uint32_t PUPDR;			//GPIO port pull-up/pull-down register							Address offset: 0x0C
	__vo uint32_t IDR;				//GPIO port input data register									Address offset: 0x10
	__vo uint32_t ODR;				//GPIO port output data register								Address offset: 0x14
	__vo uint32_t BSRR;				//GPIO port bit set/reset register								Address offset: 0x18
	__vo uint32_t LCKR;				//GPIO port configuration lock register							Address offset: 0x1C
	__vo uint32_t AFR[2];			//AFR[0] : GPIO alternate function low register					Address offset: 0x20
				 	 	 	 	 	//AFR[1] : GPIO alternate function high register				Address offset: 0x24
}GPIO_RegDef_t;



//Peripheral Register Definition Structure for RCC
typedef struct
{
	__vo uint32_t CR;				//RCC clock control register									Address offset: 0x00
	__vo uint32_t PLLCFGR;			//RCC PLL configuration register								Address offset: 0x04
	__vo uint32_t CFGR;				//RCC clock configuration register								Address offset: 0x08
	__vo uint32_t CIR;				//RCC clock interrupt register									Address offset: 0x0C
	__vo uint32_t AHB1RSTR;			//RCC AHB1 peripheral reset register							Address offset: 0x10
	__vo uint32_t AHB2RSTR;			//RCC AHB2 peripheral reset register							Address offset: 0x14
	__vo uint32_t AHB3RSTR;			//RCC AHB3 peripheral reset register							Address offset: 0x18
	uint32_t 	  RESERVED0;		//Reserved, 0x1C
	__vo uint32_t APB1RSTR;			//RCC APB1 peripheral reset register							Address offset: 0x20
	__vo uint32_t APB2RSTR;			//RCC APB2 peripheral reset register							Address offset: 0x24
	uint32_t 	  RESERVED1[2];		//Reserved, 0x28
									//			0x2C
	__vo uint32_t AHB1ENR;			//RCC AHB1 peripheral clock enable register						Address offset: 0x30
	__vo uint32_t AHB2ENR;			//RCC AHB2 peripheral clock enable register						Address offset: 0x34
	__vo uint32_t AHB3ENR;			//RCC AHB3 peripheral clock enable register						Address offset: 0x38
	uint32_t 	  RESERVED2;		//Reserved, 0x38
	__vo uint32_t APB1ENR;			//RCC APB1 peripheral clock enable register						Address offset: 0x40
	__vo uint32_t APB2ENR;			//RCC APB2 peripheral clock enable register						Address offset: 0x44
	uint32_t 	  RESERVED3[2];		//Reserved, 0x48
									//			0x4C
	__vo uint32_t AHB1LPENR;		//RCC AHB1 peripheral clock enable in low power mode register	Address offset: 0x50
	__vo uint32_t AHB2LPENR;		//RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54
	__vo uint32_t AHB3LPENR;		//RCC AHB3 peripheral clock enable in low power mode register	Address offset: 0x58
	uint32_t	  RESERVED4;		//Reserved, 0x5C
	__vo uint32_t APB1LPENR;		//RCC APB1 peripheral clock enable in low power mode register	Address offset: 0x60
	__vo uint32_t APB2LPENR;		//RCC APB2 peripheral clock enabled in low power mode register	Address offset: 0x64
	uint32_t	  RESERVED5[2];		//Reserved, 0x68
									//			0x6C
	__vo uint32_t BDCR;				//RCC Backup domain control register							Address offset: 0x70
	__vo uint32_t CSR;				//RCC clock control & status register							Address offset: 0x74
	uint32_t	  RESERVED6[2];		//Reserved, 0x78
									//			0x7C
	__vo uint32_t SSCGR;			//RCC spread spectrum clock generation register					Address offset: 0x80
	__vo uint32_t PLLI2SCFGR;		//RCC PLLI2S configuration register								Address offset: 0x84
	__vo uint32_t PLLSAICFGR;		//RCC PLL configuration register								Address offset: 0x88
	__vo uint32_t DCKCFGR;			//RCC Dedicated Clock Configuration Register 					Address offset: 0x8C
	__vo uint32_t CKGATENR;			//RCC clocks gated enable register								Address offset: 0x90
	__vo uint32_t DCKCFGR2;			//RCC dedicated clocks configuration register 2					Address offset: 0x94

}RCC_RegDef_t;


//Peripheral Register Definition Structure for EXTI
typedef struct
{
	__vo uint32_t IMR;				//Interrupt mask register										Address offset: 0x00
	__vo uint32_t EMR;				//Event mask register											Address offset: 0x04
	__vo uint32_t RTSR;				//Rising trigger selection register								Address offset: 0x08
	__vo uint32_t FTSR;				//Falling trigger selection register							Address offset: 0x0C
	__vo uint32_t SWIER;			//Software interrupt event register								Address offset: 0x10
	__vo uint32_t PR;				//Pending register												Address offset: 0x14

}EXTI_RegDef_t;



//Peripheral Register Definition Structure for EXTI
typedef struct
{
	__vo uint32_t MEMRMP;			//SYSCFG memory remap register									Address offset: 0x00
	__vo uint32_t PMC;				//SYSCFG peripheral mode configuration register					Address offset: 0x04
	__vo uint32_t EXTICR[4];		//SYSCFG external interrupt configuration register 1			Address offset: 0x08
									//SYSCFG external interrupt configuration register 2			Address offset: 0x0C
									//SYSCFG external interrupt configuration register 3			Address offset: 0x10
									//SYSCFG external interrupt configuration register 4			Address offset: 0x14
	uint32_t RESERVED1[2];			//Reserved, 0x18
									//			0x1C
	__vo uint32_t CMPCR;			//Compensation cell control register							Address offset: 0x20
	uint32_t RESERVED2[2];			//Reserved, 0x24
									//			0x28
	__vo uint32_t CFGR;				//SYSCFG configuration register									Address offset: 0x2C

}SYSCFG_RegDef_t;



//TODO: Create struct for SPI



//Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t)
#define GPIOA					( (GPIO_RegDef_t*)GPIOA_BASEADDR )
#define GPIOB					( (GPIO_RegDef_t*)GPIOB_BASEADDR )
#define GPIOC					( (GPIO_RegDef_t*)GPIOC_BASEADDR )
#define GPIOD					( (GPIO_RegDef_t*)GPIOD_BASEADDR )
#define GPIOE					( (GPIO_RegDef_t*)GPIOE_BASEADDR )
#define GPIOF					( (GPIO_RegDef_t*)GPIOF_BASEADDR )
#define GPIOG					( (GPIO_RegDef_t*)GPIOG_BASEADDR )
#define GPIOH					( (GPIO_RegDef_t*)GPIOH_BASEADDR )


#define RCC						( (RCC_RegDef_t*)RCC_BASEADDR )


#define EXTI					( (EXTI_RegDef_t*)EXTI_BASEADDR )


#define SYSCFG					( (SYSCFG_RegDef_t*)SYSCFG_BASEADDR )


//Clock Enable Macros for GPIOx peripherals
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 0 ))		//bit position 0 made to 1
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 7 ))



//Clock Enable Macros for I2Cx peripherals
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 21 ))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 22 ))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 23 ))



//Clock Enable Macros for SPIx peripherals
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 12 ))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 14 ))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 15 ))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 13 ))


//Clock Enable Macros for USARTx & UARTx peripherals
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))

#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))

#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))


//Clock enable Macro for SYSCFG peripheral
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )


//TODO: To the same for the clock peripherals above, as well as creating DISABLE macros below-----------------



//Clock Disable Macros for GPIOx peripherals
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 7 ))


//Clock Disable Macros for I2C peripherals
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 21 ))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 22 ))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 23 ))



//Clock Disable Macros for USARTx & UARTx peripherals
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))

#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))

#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))




//Clock Disable Macro for SYSCFG peripheral
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))



//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)


#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0 :\
									  (x == GPIOB) ? 1 :\
									  (x == GPIOC) ? 2 :\
									  (x == GPIOD) ? 3 :\
									  (x == GPIOE) ? 4 :\
									  (x == GPIOF) ? 5 :\
									  (x == GPIOG) ? 6 :\
									  (x == GPIOH) ? 7 :0  )


//Interrupt Request Numbers of STM32F446xx MCU
#define IRQ_NUM_EXTI0			6
#define IRQ_NUM_EXTI1			7
#define IRQ_NUM_EXTI2			8
#define IRQ_NUM_EXTI3			9
#define IRQ_NUM_EXTI4			10
#define IRQ_NUM_EXTI9_5			23
#define IRQ_NUM_EXTI15_10		40



//Generic Macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET




#include "stm32f446xx_gpio_driver.h"


#endif /* INC_STM32F446XX_H_ */
