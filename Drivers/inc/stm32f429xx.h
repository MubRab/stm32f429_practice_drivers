/*
 * stm32f429xx.h
 *
 *  Created on: Jan 31, 2022
 *      Author: MubRab
 *      MCU Specific Device Header File
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include <stdint.h>
#include <stddef.h>

/**
 * Miscellaneous macros
 */
#define ENABLE									1U
#define DISABLE									0U

/**
 * Selected ARM Cortex M4 NVIC Register Addresses
 */
#define NVIC_ISER0                              ((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1                              ((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2                              ((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3                              ((volatile uint32_t*) 0xE000E10C)

#define NVIC_ICER0                              ((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1                              ((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2                              ((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3                              ((volatile uint32_t*) 0xE000E18C)

#define NVIC_IPR_BASE_ADDR                      ((volatile uint32_t*) 0xE000E400) /**this would be a better way, instead of all 60**/

#define NUM_PR_BITS_IMPLEMENTED                 4 /**These are the LSBs that are ignored in the IPR registers**/

/**
 * Macros for base addresses of various memory
 * obtained from Datasheet- Ch 5 memory map
 */
#define FLASH_MEMORY_BASE_ADDR					0x08000000U	/**Flash memory base address*/
#define SYS_MEMORY_BASE_ADDR					0x1FFF0000U	/**ROM base address**/
#define SRAM1_BASE_ADDR							0x20000000U	/**SRAM base address**/
#define SRAM2_BASE_ADDR							0x2001C000U
#define SRAM3_BASE_ADDR							0x20020000U

/**
 * Macros for base addresses of buses
 * obtained from Datasheet- ch5 memory map
 */
#define APB1_BASE_ADDR							0x40000000U
#define APB2_BASE_ADDR							0x40010000U
#define AHB1_BASE_ADDR							0x40020000U
#define AHB2_BASE_ADDR							0x50000000U
#define AHB3_BASE_ADDR							0x60000000U

/**
 * Selected macros for APB1 Bus peripherals
 * obtained from Datasheet Ch 5 table 13
 */
#define SPI2_BASE_ADDR                          (APB1_BASE_ADDR + 0x3800U)
#define SPI3_BASE_ADDR                          (APB1_BASE_ADDR + 0x3C00U)
#define USART2_BASE_ADDR                        (APB1_BASE_ADDR + 0x4400U)
#define USART3_BASE_ADDR                        (APB1_BASE_ADDR + 0x4800U)
#define UART4_BASE_ADDR                         (APB1_BASE_ADDR + 0x4C00U)
#define UART5_BASE_ADDR                         (APB1_BASE_ADDR + 0x5000U)
#define UART7_BASE_ADDR                         (APB1_BASE_ADDR + 0x7800U)
#define UART8_BASE_ADDR                         (APB1_BASE_ADDR + 0x7C00U)
#define I2C1_BASE_ADDR                          (APB1_BASE_ADDR + 0x5400U)
#define I2C2_BASE_ADDR                          (APB1_BASE_ADDR + 0x5800U)
#define I2C3_BASE_ADDR                          (APB1_BASE_ADDR + 0x5C00U)

/**
 * Selected macros for APB2 Bus peripherals
 * obtained from Datasheet Ch 5 table 13
 */
#define USART1_BASE_ADDR                        (APB2_BASE_ADDR + 0x1000U)
#define USART6_BASE_ADDR                        (APB2_BASE_ADDR + 0x1400U)
#define SPI1_BASE_ADDR                          (APB2_BASE_ADDR + 0x3000U)
#define SPI4_BASE_ADDR                          (APB2_BASE_ADDR + 0x3400U)
#define SYSCFG_BASE_ADDRESS						(APB2_BASE_ADDR + 0x3800U)
#define EXTI_BASE_ADDRESS						(APB2_BASE_ADDR + 0x3C00U)
#define SPI5_BASE_ADDR                          (APB2_BASE_ADDR + 0x5000U)
#define SPI6_BASE_ADDR                          (APB2_BASE_ADDR + 0x5400U)

/**
 * Selected macros for AHB1 Bus peripherals
 * obtained from Datasheet Ch 5 table 13
 */
#define GPIOA_BASE_ADDR							(AHB1_BASE_ADDR + 0x0000U)
#define GPIOB_BASE_ADDR							(AHB1_BASE_ADDR + 0x0400U)
#define GPIOC_BASE_ADDR							(AHB1_BASE_ADDR + 0x0800U)
#define GPIOD_BASE_ADDR							(AHB1_BASE_ADDR + 0x0C00U)
#define GPIOE_BASE_ADDR							(AHB1_BASE_ADDR + 0x1000U)
#define GPIOF_BASE_ADDR							(AHB1_BASE_ADDR + 0x1400U)
#define GPIOG_BASE_ADDR							(AHB1_BASE_ADDR + 0x1800U)
#define GPIOH_BASE_ADDR							(AHB1_BASE_ADDR + 0x1C00U)
#define GPIOI_BASE_ADDR							(AHB1_BASE_ADDR + 0x2000U)
#define GPIOJ_BASE_ADDR							(AHB1_BASE_ADDR + 0x2400U)
#define GPIOK_BASE_ADDR							(AHB1_BASE_ADDR + 0x2800U)
#define RCC_BASE_ADDR							(AHB1_BASE_ADDR + 0x3800U)
/**
 * Selected macros for AHB2 Bus peripherals
 * obtained from Datasheet Ch 5 table 13
 */

/**
 * Vector Table IRQ Number (position) macros
 * see ch12 vector table in Reference manual
 */
/**GPIO**/
#define EXTI0_IRQ_NUM                               6
#define EXTI1_IRQ_NUM                               7
#define EXTI2_IRQ_NUM                               8
#define EXTI3_IRQ_NUM                               9
#define EXTI4_IRQ_NUM                               10
#define EXTI9_5_IRQ_NUM                             23
#define EXTI15_10_IRQ_NUM                           40

/**SPI**/
#define SPI1_IRQ_NUM                                35
#define SPI2_IRQ_NUM                                36
#define SPI3_IRQ_NUM                                51
#define SPI4_IRQ_NUM                                84
#define SPI5_IRQ_NUM                                85
#define SPI6_IRQ_NUM                                86

/*I2C*/
#define I2C1_EV_IRQ_NUM                             31
#define I2C1_ER_IRQ_NUM                             32
#define I2C2_EV_IRQ_NUM                             33
#define I2C2_ER_IRQ_NUM                             34
#define I2C3_EV_IRQ_NUM                             72
#define I2C3_ER_IRQ_NUM                             73

/**
 * Vector Table IRQ Priority macros
 * see ch12 vector table in Reference manual
 */
#define EXTI0_IRQ_P                                 13
#define EXTI1_IRQ_P                                 14
#define EXTI2_IRQ_P                                 15
#define EXTI3_IRQ_P                                 16
#define EXTI4_IRQ_P                                 17
#define EXTI9_5_IRQ_P                               30
#define EXTI15_10_IRQ_P                             47

/**SPI**/
#define SPI1_IRQ_P                                  43
#define SPI2_IRQ_P                                  42
#define SPI3_IRQ_P                                  58
#define SPI4_IRQ_P                                  91
#define SPI5_IRQ_P                                  92
#define SPI6_IRQ_P                                  93

/****************************** Peripheral Registers ****************************/

/**
 * RCC registers struct
 * obtained from reference manual Ch. 6- RCC register map
 */
typedef struct
{
	/**uint32_t is 4 bytes and each offset of the register is 4 bytes**/
    volatile uint32_t CR;/****/
    volatile uint32_t PLLCFGR;/****/
    volatile uint32_t CFGR;/****/
    volatile uint32_t CIR;/****/
    volatile uint32_t AHB1RSTR;/****/
    volatile uint32_t AHB2RSTR;/****/
    volatile uint32_t AHB3RSTR;/****/
    uint32_t RESERVED;/****/
    volatile uint32_t APB1RSTR;/****/
    volatile uint32_t APB2RSTR;/****/
    uint32_t RESERVED1;/****/
    uint32_t RESERVED2;/****/
    volatile uint32_t AHB1ENR;/****/
    volatile uint32_t AHB2ENR;/****/
    volatile uint32_t AHB3ENR;/****/
    uint32_t RESERVED3;/****/
    volatile uint32_t APB1ENR;/****/
    volatile uint32_t APB2ENR;/****/
    uint32_t RESERVED4;/****/
    uint32_t RESERVED5;/****/
    volatile uint32_t AHB1LPENR;/****/
    volatile uint32_t AHB2LPENR;/****/
    volatile uint32_t AHB3LPENR;/****/
    uint32_t RESERVED6;/****/
    volatile uint32_t APB1LPENR;/****/
    volatile uint32_t APB2LPENR;/****/
    uint32_t RESERVED7;/****/
    uint32_t RESERVED8;/****/
    volatile uint32_t BDCR;/****/
    volatile uint32_t CSR;/****/
    uint32_t RESERVED9;/****/
    uint32_t RESERVED10;/****/
    volatile uint32_t SSCGR;/****/
    volatile uint32_t PLLI2SCFGR;/****/
    volatile uint32_t PLLSAICFGR;/****/
    volatile uint32_t DCKCFGR;/****/
} RCC_Registers_t;

#define RCC										((RCC_Registers_t*) RCC_BASE_ADDR)

/**
 * GPIO registers struct
 * obtained from reference manual Ch. 8- GPIO register map
 */
typedef struct
{
	/**uint32_t is 4 bytes and each offset of the register is 4 bytes**/
    volatile uint32_t MODER;/****/
    volatile uint32_t OTYPER;/****/
    volatile uint32_t OSPEEDR;/****/
    volatile uint32_t PUPDR;/****/
    volatile uint32_t IDR;/****/
    volatile uint32_t ODR;/****/
    volatile uint32_t BSRR;/****/
    volatile uint32_t LCKR;/****/
    volatile uint32_t AFRL;/****/
    volatile uint32_t AFRH;/****/
} GPIO_Registers_t;

/**
 * Type-casting each GPIO port base address to the registers struct
 */
#define GPIOA                                   ((GPIO_Registers_t*) GPIOA_BASE_ADDR)
#define GPIOB                                   ((GPIO_Registers_t*) GPIOB_BASE_ADDR)
#define GPIOC                                   ((GPIO_Registers_t*) GPIOC_BASE_ADDR)
#define GPIOD                                   ((GPIO_Registers_t*) GPIOD_BASE_ADDR)
#define GPIOE                                   ((GPIO_Registers_t*) GPIOE_BASE_ADDR)
#define GPIOF                                   ((GPIO_Registers_t*) GPIOF_BASE_ADDR)
#define GPIOG                                   ((GPIO_Registers_t*) GPIOG_BASE_ADDR)
#define GPIOH                                   ((GPIO_Registers_t*) GPIOH_BASE_ADDR)
#define GPIOI                                   ((GPIO_Registers_t*) GPIOI_BASE_ADDR)
#define GPIOJ                                   ((GPIO_Registers_t*) GPIOJ_BASE_ADDR)
#define GPIOK                                   ((GPIO_Registers_t*) GPIOK_BASE_ADDR)

/**
 * SYSCFG registers struct
 * obtained from reference manual Ch. 9
 */
typedef struct
{
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    uint32_t RESERVED;
    uint32_t RESERVED1;
    volatile uint32_t CMPCR;
} SYSCFG_Registers_t;

#define SYSCFG                                  ((SYSCFG_Registers_t*) SYSCFG_BASE_ADDRESS)


/**
 * EXTI registers struct
 * obtained from reference manual Ch. 12- EXTI register map
 */
typedef struct
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RSTR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_Registers_t;

#define EXTI                                    ((EXTI_Registers_t*) EXTI_BASE_ADDRESS)

/**
 * I2C registers struct
 * Ch 27 - Register Map
 */
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_Registers_t;

#define I2C1                                    ((I2C_Registers_t*) I2C1_BASE_ADDR)
#define I2C2                                    ((I2C_Registers_t*) I2C2_BASE_ADDR)
#define I2C3                                    ((I2C_Registers_t*) I2C3_BASE_ADDR)

/**
 * SPI registers struct
 * Ch 28 - Register Map
 */
typedef struct
{
    volatile uint32_t CR1;      /****/
    volatile uint32_t CR2;      /****/
    volatile uint32_t SR;       /****/
    volatile uint32_t DR;       /****/
    volatile uint32_t CRCPR;    /****/
    volatile uint32_t RXCPR;    /****/
    volatile uint32_t TXCRCR;   /****/
    volatile uint32_t IS2CFGR;  /****/
    volatile uint32_t IS2SPR;   /****/
} SPI_Registers_t;

#define SPI1                                    ((SPI_Registers_t*) SPI1_BASE_ADDR)
#define SPI2                                    ((SPI_Registers_t*) SPI2_BASE_ADDR)
#define SPI3                                    ((SPI_Registers_t*) SPI3_BASE_ADDR)
#define SPI4                                    ((SPI_Registers_t*) SPI4_BASE_ADDR)
#define SPI5                                    ((SPI_Registers_t*) SPI5_BASE_ADDR)
#define SPI6                                    ((SPI_Registers_t*) SPI6_BASE_ADDR)

/**
 * USART registers struct
 * see ch 30.6 in reference manual
 */
typedef struct
{
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_Registers_t;

#define USART1                                  ((USART_Registers_t*) USART1_BASE_ADDR)
#define USART2                                  ((USART_Registers_t*) USART2_BASE_ADDR)
#define USART3                                  ((USART_Registers_t*) USART3_BASE_ADDR)
#define UART4                                   ((USART_Registers_t*) UART4_BASE_ADDR)
#define UART5                                   ((USART_Registers_t*) UART5_BASE_ADDR)
#define USART6                                  ((USART_Registers_t*) USART6_BASE_ADDR)
#define UART7                                   ((USART_Registers_t*) UART7_BASE_ADDR)
#define UART8                                   ((USART_Registers_t*) UART8_BASE_ADDR)

/*********************************Function Macros******************************/
/**
 * RCC function macros to enable clocks
 */
#define GPIOA_CLK_EN()							(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()							(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()							(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()							(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()							(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()							(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()							(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()							(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN()							(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_CLK_EN()							(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_CLK_EN()							(RCC->AHB1ENR |= (1 << 10))

/**SPI**/
#define SPI1_CLK_EN()                           (RCC->APB2ENR |= (1<<12))
#define SPI2_CLK_EN()                           (RCC->APB1ENR |= (1<<14))
#define SPI3_CLK_EN()                           (RCC->APB1ENR |= (1<<15))
#define SPI4_CLK_EN()                           (RCC->APB2ENR |= (1<<13))
#define SPI5_CLK_EN()                           (RCC->APB2ENR |= (1<<20))
#define SPI6_CLK_EN()                           (RCC->APB2ENR |= (1<<21))

/*I2C*/
#define I2C1_CLK_EN()                           (RCC->APB1ENR |= (1<<21))
#define I2C2_CLK_EN()                           (RCC->APB1ENR |= (1<<22))
#define I2C3_CLK_EN()                           (RCC->APB1ENR |= (1<<23))

/*USART*/
#define USART1_CLK_EN()                         (RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_EN()                         (RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN()                         (RCC->APB1ENR |= (1 << 18))
#define UART4_CLK_EN()                          (RCC->APB1ENR |= (1 << 19))
#define UART5_CLK_EN()                          (RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_EN()                         (RCC->APB2ENR |= (1 << 5))
#define UART7_CLK_EN()                          (RCC->APB1ENR |= (1 << 30))
#define UART8_CLK_EN()                          (RCC->APB1ENR |= (1 << 31))

#define SYSCFG_CLK_EN()                         (RCC->APB2ENR |= (1 << 14))
/**
 * RCC function macros to disable clocks
 */
#define GPIOA_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_CLK_DI()							(RCC->AHB1ENR &= ~(1 << 10))

/**SPI**/
#define SPI1_CLK_DI()                           (RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLK_DI()                           (RCC->APB1ENR &= ~(1<<14))
#define SPI3_CLK_DI()                           (RCC->APB1ENR &= ~(1<<15))
#define SPI4_CLK_DI()                           (RCC->APB2ENR &= ~(1<<13))
#define SPI5_CLK_DI()                           (RCC->APB2ENR &= ~(1<<20))
#define SPI6_CLK_DI()                           (RCC->APB2ENR &= ~(1<<21))

/*I2C*/
#define I2C1_CLK_DI()                           (RCC->APB1ENR &= ~(1<<21))
#define I2C2_CLK_DI()                           (RCC->APB1ENR &= ~(1<<22))
#define I2C3_CLK_DI()                           (RCC->APB1ENR &= ~(1<<23))

/*USART*/
#define USART1_CLK_DI()                         (RCC->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DI()                         (RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DI()                         (RCC->APB1ENR &= ~(1 << 18))
#define UART4_CLK_DI()                          (RCC->APB1ENR &= ~(1 << 19))
#define UART5_CLK_DI()                          (RCC->APB1ENR &= ~(1 << 20))
#define USART6_CLK_DI()                         (RCC->APB2ENR &= ~(1 << 5))
#define UART7_CLK_DI()                          (RCC->APB1ENR &= ~(1 << 30))
#define UART8_CLK_DI()                          (RCC->APB1ENR &= ~(1 << 31))

#define SYSCFG_CLK_DI()                         (RCC->APB2ENR &= (1 << 14))
/**
 * RCC function macros to reset peripherals
 */
#define GPIOA_RESET()							do{RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0);}while(0)
#define GPIOB_RESET()							do{RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1);}while(0)
#define GPIOC_RESET()							do{RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2);}while(0)
#define GPIOD_RESET()							do{RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3);}while(0)
#define GPIOE_RESET()							do{RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4);}while(0)
#define GPIOF_RESET()							do{RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5);}while(0)
#define GPIOG_RESET()							do{RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6);}while(0)
#define GPIOH_RESET()							do{RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7);}while(0)
#define GPIOI_RESET()							do{RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8);}while(0)
#define GPIOJ_RESET()							do{RCC->AHB1RSTR |= (1 << 9); RCC->AHB1RSTR &= ~(1 << 9);}while(0)
#define GPIOK_RESET()							do{RCC->AHB1RSTR |= (1 << 10); RCC->AHB1RSTR &= ~(1 << 10);}while(0)

/*SPI*/
#define SPI1_RESET()                            do{RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12);}while(0)
#define SPI2_RESET()                            do{RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14);}while(0)
#define SPI3_RESET()                            do{RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15);}while(0)
#define SPI4_RESET()                            do{RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13);}while(0)
#define SPI5_RESET()                            do{RCC->APB2RSTR |= (1 << 20); RCC->APB2RSTR &= ~(1 << 20);}while(0)
#define SPI6_RESET()                            do{RCC->APB2RSTR |= (1 << 21); RCC->APB2RSTR &= ~(1 << 21);}while(0)

/*I2C*/
#define I2C1_RESET()                            do{RCC->APB1RSTR |= (1 << 21); RCC->APB1RSTR &= ~(1 << 21);}while(0)
#define I2C2_RESET()                            do{RCC->APB1RSTR |= (1 << 22); RCC->APB1RSTR &= ~(1 << 22);}while(0)
#define I2C3_RESET()                            do{RCC->APB1RSTR |= (1 << 23); RCC->APB1RSTR &= ~(1 << 23);}while(0)

/*USART*/
#define USART1_RESET()                          do{RCC->APB2RSTR |= (1 << 4); RCC->APB2RSTR &= ~(1 << 4);}while(0)
#define USART2_RESET()                          do{RCC->APB1RSTR |= (1 << 17); RCC->APB1RSTR &= ~(1 << 17);}while(0)
#define USART3_RESET()                          do{RCC->APB1RSTR |= (1 << 18); RCC->APB1RSTR &= ~(1 << 18);}while(0)
#define UART4_RESET()                           do{RCC->APB1RSTR |= (1 << 19); RCC->APB1RSTR &= ~(1 << 19);}while(0)
#define UART5_RESET()                           do{RCC->APB1RSTR |= (1 << 20); RCC->APB1RSTR &= ~(1 << 20);}while(0)
#define USART6_RESET()                          do{RCC->APB2RSTR |= (1 << 5); RCC->APB2RSTR &= ~(1 << 5);}while(0)
#define UART7_RESET()                           do{RCC->APB1RSTR |= (1 << 30); RCC->APB1RSTR &= ~(1 << 30);}while(0)
#define UART8_RESET()                           do{RCC->APB1RSTR |= (1 << 31); RCC->APB1RSTR &= ~(1 << 31);}while(0)


#define SYSCFG_RESET()                          do{RCC->APB2RSTR |= (1 << 14); RCC->APB2RSTR &= ~(1 << 14);}while(0)


#endif /* INC_STM32F429XX_H_ */
