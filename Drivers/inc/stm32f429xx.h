/*
 * stm32f429xx.h
 *
 *  Created on: Jan 31, 2022
 *      Author: MubRab
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include <stdint.h>

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

#define NVIC_ICER0                              ((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1                              ((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2                              ((volatile uint32_t*) 0xE000E188)

#define NVIC_IPR_BASE_ADDR                      ((volatile uint32_t*) 0xE000E400) /**this would be a better way, instead of all 60**/

#define NUM_PR_BITS_IMPLEMENTED                 4

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

/**
 * Selected macros for APB2 Bus peripherals
 * obtained from Datasheet Ch 5 table 13
 */
#define SYSCFG_BASE_ADDRESS						(APB2_BASE_ADDR + 0x3800U)
#define EXTI_BASE_ADDRESS						(APB2_BASE_ADDR + 0x3C00U)

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
#define EXTI0_IRQ_NUM                               6
#define EXTI1_IRQ_NUM                               7
#define EXTI2_IRQ_NUM                               8
#define EXTI3_IRQ_NUM                               9
#define EXTI4_IRQ_NUM                               10
#define EXTI9_5_IRQ_NUM                             23
#define EXTI15_10_IRQ_NUM                           40

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
 * Type-casting each GPIO port base address to the registers struct
 */
#define GPIOA									((GPIO_Registers_t*) GPIOA_BASE_ADDR)
#define GPIOB									((GPIO_Registers_t*) GPIOB_BASE_ADDR)
#define GPIOC									((GPIO_Registers_t*) GPIOC_BASE_ADDR)
#define GPIOD									((GPIO_Registers_t*) GPIOD_BASE_ADDR)
#define GPIOE									((GPIO_Registers_t*) GPIOE_BASE_ADDR)
#define GPIOF									((GPIO_Registers_t*) GPIOF_BASE_ADDR)
#define GPIOG									((GPIO_Registers_t*) GPIOG_BASE_ADDR)
#define GPIOH									((GPIO_Registers_t*) GPIOH_BASE_ADDR)
#define GPIOI									((GPIO_Registers_t*) GPIOI_BASE_ADDR)
#define GPIOJ									((GPIO_Registers_t*) GPIOJ_BASE_ADDR)
#define GPIOK									((GPIO_Registers_t*) GPIOK_BASE_ADDR)

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

#define SYSCFG_RESET()                          do{RCC->APB2RSTR |= (1 << 14); RCC->APB2RSTR &= ~(1 << 14);}while(0)
#endif /* INC_STM32F429XX_H_ */
