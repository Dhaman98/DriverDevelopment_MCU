/*
 * stm32f401xe.h
 *
 *  Created on: Feb 5, 2024
 *      Author: pldha
 */



#ifndef INC_STM32F401XE_H_
#define INC_STM32F401XE_H_


#include<stdint.h>
#include<stddef.h>


#define __vo volatile
#define __weak __attribute__((weak))



/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4


/*Memory base addresses*/
#define FLASH_BASEADDR   0x08000000UL  //flash memory (or) RAM base address
#define SRAM1_BASEADDR   0x20000000UL   // SRAM1 base address
#define ROM_BASEADDR     0x1FFF0000UL	//ROM (or) system memory base address
#define SRAM 			 SRAM1_BASEADDR  // SRAM base address


/***********************************PERIPHERALS SPECIFIC DETAILS*************************************************************************/
			/*AHBx APBx Peripheral base addresses*/

#define PERIPH_BASEADDR			0x40000000UL
#define APB1PERIPH_BASEADDR 	PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000UL
#define AHB1PERIPH_BASEADDR		0x40020000UL
#define AHB2PERIPH_BASEADDR		0x50000000UL


			/*Base addresses of peripherals which are hanging on AHB1 bus*/
#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1000)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1C00)
#define RCC_BASEADDR        (AHB1PERIPH_BASEADDR + 0X3800) // RCC register base address


			/*Base addresses of peripherals which are hanging on APB1 bus */
#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0X5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0X5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0X5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0X3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0X3C00)

#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0X4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000)


 	 	 	 /*Base addresses of peripherals which are hanging on APB1 bus */
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0X1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0X1400)

#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0X3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASEADDR + 0X3400)

#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0X3C00)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0X3800)

/********************PERIPHERAL REGISTER DEFINITION STRUCTURES***************** */

    			/*GPIO port registers definition*/
typedef struct {
	__vo uint32_t MODER;    //GPIO port mode register  				Address offset : 0x00
	__vo uint32_t OTYPER;	//GPIO port output type register		Address offset : 0x04
	__vo uint32_t OSPEEDR;	//GPIO port output speed register		Address offset : 0x08
	__vo uint32_t PUPDR;		//GPIO port pull-up/pull-down register	Address offset : 0x0c
	__vo uint32_t IDR;		//GPIO port input data register			Address offset : 0x10
	__vo uint32_t ODR;		//GPIO port output data register		Address offset : 0x14
	__vo uint32_t BSRR;		//GPIO port bit set/reset register		Address offset : 0x18
	__vo uint32_t LCKR;		//GPIO port configuration lock register	Address offset : 0x1c
	__vo uint32_t AFR[2];		//GPIO alternate function low register	Address offset : 0x20
								//GPIO alternate function high register	Address offset : 0x24
}GPIO_RegDef_t;  // GPIO port registers definition



				/*RCC registers definition*/
typedef struct {
	__vo uint32_t RCC_CR;				//RCC clock control register				Address offset: 0x00
	__vo uint32_t RCC_PLLCFGR;			//RCC PLL configuration register 			Address offset: 0x04
	__vo uint32_t CFGR;				//RCC clock configuration register 			Address offset: 0x08
	__vo uint32_t RCC_CIR;				//RCC clock interrupt register 				Address offset: 0x0C


	__vo uint32_t RCC_AHB1RSTR;			//RCC AHB1 peripheral reset register  		Address offset: 0x10
	__vo uint32_t RCC_AHB2RSTR;			//RCC AHB2 peripheral reset register  		Address offset: 0x14
	 uint32_t RCC_reserved1;			//RESREVED
	 uint32_t RCC_reserved2;			//RESERVED
	__vo uint32_t RCC_APB1RSTR;			//RCC APB1 peripheral reset register  		AdDress offset: 0x20
	__vo uint32_t RCC_APB2RSTR;			//RCC APB2 peripheral reset register		Address offset: 0x24
	 uint32_t RCC_reserved3;			//RESERVED
	 uint32_t RCC_reserved4;			//RESERVED
	__vo uint32_t RCC_AHB1ENR;			//RCC AHB1 peripheral clock enable register	Address offset: 0x30
	__vo uint32_t RCC_AHB2ENR;			//RCC AHB2 peripheral clock enable register Address offset: 0x34
	 uint32_t RCC_reserved5;			//RESERVED
	 uint32_t RCC_reserved6;			//RESERVED
	__vo uint32_t RCC_APB1ENR;	        //RCC APB1 peripheral clock enable register  Address offset: 0x40
	__vo uint32_t RCC_APB2ENR;	        //RCC APB2 peripheral clock enable register  Address offset: 0x44
	 uint32_t RCC_reserved7;	        //RESERVED
	 uint32_t RCC_reserved8;	        //RESERVED
	__vo uint32_t RCC_AHB1LPENR;	    //RCC AHB1 peripheral clock enable in low power mode register  Address offset: 0x50
	__vo uint32_t RCC_AHB2LPENR;	    //RCC AHB2 peripheral clock enable in low power mode register  Address offset: 0x54
	 uint32_t RCC_reserved9;	        //RESERVED
	 uint32_t RCC_reserved10;	        //RESERVED
	__vo uint32_t RCC_APB1LPENR;	    //RCC APB1 peripheral clock enable in low power mode register  Address offset: 0x60
	__vo uint32_t RCC_APB2LPENR;	    //RCC APB2 peripheral clock enable in low power mode register  Address offset: 0x64
	 uint32_t RCC_reserved11;	        //RESERVED
	 uint32_t RCC_reserved12;	        //RESERVED
	__vo uint32_t RCC_BDCR;	            //RCC Backup domain control register		Address offset: 0x70
	__vo uint32_t RCC_CSR;	            //RCC clock control & status register		Address offset: 0x74
	 uint32_t RCC_reserved13;	        //RESERVED
	 uint32_t RCC_reserved14;	        //RESERVED
	__vo uint32_t RCC_SSCGR;	        //RCC spread spectrum clock generation register   Address offset: 0x80
	__vo uint32_t RCC_PLLI2SCFGR;	    //RCC PLLI2S configuration register               Address offset: 0x84
	 uint32_t RCC_reserved15;	        //RESERVED
	__vo uint32_t RCC_DCKCFGR;	        //RCC Dedicated Clocks Configuration Register     Address offset: 0x8C

}RCC_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< ,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< ,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< ,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< ,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< ,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< ,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< ,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< ,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< ,     										Address offset: 0x20 */
} SPI_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;



/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
} SYSCFG_RegDef_t;



/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;



/**********************GPIO PERIPHERAL DEFINITIONS( PERIPHERAL BASE ADDRESSES TYPECASTED TO XXX_RegDef_T)******************/
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)

/**********************SPI Register DEFINITION( PERIPHERAL BASE ADDRESSES TYPECASTED TO XXX_RegDef_T) */
#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BASEADDR)

/**********************I2C Register DEFINITION( PERIPHERAL BASE ADDRESSES TYPECASTED TO XXX_RegDef_T) */\

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)


#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

/**********************RCC Register DEFINITION( RCC BASE ADDRESSES TYPECASTED TO XXX_RegDef_T) */
#define RCC  ((RCC_RegDef_t*)RCC_BASEADDR)

/**********************EXTI Register DEFINITION (EXTI base address typecasted to  )*************/
#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)


/*********************SYSCG Register Definition ************************/
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOH)?7:0)

    /*Clock Enable Macros for GPIOx Peripherals */
#define GPIOA_PCLK_EN()  (RCC->RCC_AHB1ENR |= (1<<0))  //enabling clock for GPIOA port
#define GPIOB_PCLK_EN()  (RCC->RCC_AHB1ENR |= (1<<1))  //enabling clock for GPIOB port
#define GPIOC_PCLK_EN()  (RCC->RCC_AHB1ENR |= (1<<2))  //enabling clock for GPIOB port
#define GPIOD_PCLK_EN()  (RCC->RCC_AHB1ENR |= (1<<3))  //enabling clock for GPIOB port
#define GPIOE_PCLK_EN()  (RCC->RCC_AHB1ENR |= (1<<4))  //enabling clock for GPIOB port
#define GPIOH_PCLK_EN()  (RCC->RCC_AHB1ENR |= (1<<7))  //enabling clock for GPIOB port


		/*Clock Enable Macros for I2C Peripherals */
#define I2C1_PCLK_EN()  (RCC->RCC_APB1ENR |= (1<<21))  //enabling clock for I2C1
#define I2C2_PCLK_EN()  (RCC->RCC_APB1ENR |= (1<<22))  //enabling clock for I2C2
#define I2C3_PCLK_EN()  (RCC->RCC_APB1ENR |= (1<<23))  //enabling clock for I2C3

		/*Clock Enable Macros for SPI Peripherals */
#define SPI1_PCLK_EN()  (RCC->RCC_APB2ENR |= (1<<12))  //enabling clock for SPI1
#define SPI2_PCLK_EN()  (RCC->RCC_APB1ENR |= (1<<14))  //enabling clock for SPI2
#define SPI3_PCLK_EN()  (RCC->RCC_APB1ENR |= (1<<15))  //enabling clock for SPI3
#define SPI4_PCLK_EN()  (RCC->RCC_APB2ENR |= (1<<13))  //enabling clock for SPI4


		/*Clock Enable Macros for USART Peripherals */
#define USART1_PCCK_EN()  (RCC->RCC_APB2ENR |= (1<<4))  //enabling clock for USART1
#define USART3_PCCK_EN()  (RCC->RCC_APB2ENR |= (1<<7))  //enabling clock for USART1
#define UART4_PCCK_EN()  (RCC->RCC_APB2ENR |= (1<<15))  //enabling clock for USART1


#define USART2_PCCK_EN()  (RCC->RCC_APB1ENR |= (1<<17))  //enabling clock for USART2
#define USART6_PCCK_EN()  (RCC->RCC_APB2ENR |= (1<<5))  //enabling clock for USART6

         /*Clock Enable Macros for SYSCFG Peripherals */
#define SYSCFG_PCLK_EN()  (RCC->RCC_APB2ENR |= (1<<14))  //enabling clock for SYSCFG



         /*Clock Disable Macros for GPIOx Peripherals */
#define GPIOA_PCLK_DI()  (RCC->RCC_AHB1ENR &= ~(1<<0))  //disabling clock for GPIOA port
#define GPIOB_PCLK_DI()  (RCC->RCC_AHB1ENR &= ~(1<<1))  //disabling clock for GPIOB port
#define GPIOC_PCLK_DI()  (RCC->RCC_AHB1ENR &= ~(1<<2))  //disabling clock for GPIOB port
#define GPIOD_PCLK_DI()  (RCC->RCC_AHB1ENR &= ~(1<<3))  //disabling clock for GPIOB port
#define GPIOE_PCLK_DI()  (RCC->RCC_AHB1ENR &= ~(1<<4))  //disabling clock for GPIOB port
#define GPIOH_PCLK_DI()  (RCC->RCC_AHB1ENR &= ~(1<<7))  //disabling clock for GPIOB port

         /*Clock Disable Macros for I2C Peripherals */
#define I2C1_PCLK_DI()  (RCC->RCC_APB1ENR &= ~(1<<21))  //disabling clock for I2C1
#define I2C2_PCLK_DI()  (RCC->RCC_APB1ENR &= ~(1<<22))  //disabling clock for I2C2
#define I2C3_PCLK_DI()  (RCC->RCC_APB1ENR &= ~(1<<23))  //disabling clock for I2C3

		/*Clock Disable Macros for SPI Peripherals */
#define SPI1_PCLK_DI()  (RCC->RCC_APB2ENR &= ~(1<<12))  //disabling clock for SPI1
#define SPI2_PCLK_DI()  (RCC->RCC_APB1ENR &= ~(1<<14))  //disabling clock for SPI2
#define SPI3_PCLK_DI()  (RCC->RCC_APB1ENR &= ~(1<<15))  //disabling clock for SPI3
#define SPI4_PCLK_DI()  (RCC->RCC_APB2ENR &= ~(1<<13))  //disabling clock for SPI4

		/*Clock Disable Macros for USART Peripherals */
#define USART1_PCLK_DI()  (RCC->RCC_APB2ENR &= ~(1<<4))  //disabling clock for USART1
#define USART2_PCLK_DI()  (RCC->RCC_APB1ENR &= ~(1<<17))  //disabling clock for USART2
#define USART6_PCLK_DI()  (RCC->RCC_APB2ENR &= ~(1<<5))  //disabling clock for USART6

         /*Clock Disable Macros for SYSCFG Peripherals */
#define SYSCFG_PCLK_DI()  (RCC->RCC_APB2ENR &= ~(1<<14))  //disabling clock for SYSCFG


/***************************************************************************************
 * **************Macros to reset GPIOx peripheral
 */
#define GPIOA_REG_RESET()    do{(RCC->RCC_AHB1RSTR|= (1<<0)); (RCC->RCC_AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	 do{(RCC->RCC_AHB1RSTR|= (1<<1)); (RCC->RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	 do{(RCC->RCC_AHB1RSTR|= (1<<2)); (RCC->RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	 do{(RCC->RCC_AHB1RSTR|= (1<<3)); (RCC->RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	 do{(RCC->RCC_AHB1RSTR|= (1<<4)); (RCC->RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()	 do{(RCC->RCC_AHB1RSTR|= (1<<7)); (RCC->RCC_AHB1RSTR &= ~(1<<7));}while(0)


/***************************************************************************************
 * **************Macros to reset SPIx peripheral
 */
#define SPI1_REG_RESET()	do{(RCC->RCC_APB2RSTR|=(1<<12)); (RCC->RCC_APB2RSTR&=~(1<<12));}while(0)
#define SPI2_REG_RESET()	do{(RCC->RCC_APB1RSTR|=(1<<14)); (RCC->RCC_APB2RSTR&=~(1<<14));}while(0)
#define SPI3_REG_RESET()	do{(RCC->RCC_APB1RSTR|=(1<<15)); (RCC->RCC_APB2RSTR&=~(1<<15));}while(0)
#define SPI4_REG_RESET()	do{(RCC->RCC_APB2RSTR|=(1<<13)); (RCC->RCC_APB2RSTR&=~(1<<13));}while(0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


//Generic macros
#define ENABLE			   1
#define	DISABLE			   0
#define SET 			   ENABLE
#define RESET 			   DISABLE
#define GPIO_PIN_SET 	   SET
#define GPIO_PIN_RESET 	   RESET
#define FLAG_RESET         RESET
#define FLAG_SET 		   SET


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8






/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS                    9



#include "stm32f401xe_gpio_driver.h"
#include "stm32f401xe_spi_driver.h"
#include "stm32f401xe_i2c_driver.h"
#include "stm32f401xe_rcc_driver.h"
#include "stm32f401xe_usart_driver.h"

#endif /* INC_STM32F401XE_H_ */


