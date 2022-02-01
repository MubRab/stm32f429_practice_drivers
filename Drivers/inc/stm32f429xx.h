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
#define ENABLE									1
#define DISABLE									0

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
    volatile uint32_t RESERVED;/****/
    volatile uint32_t APB1RSTR;/****/
    volatile uint32_t APB2RSTR;/****/
    volatile uint32_t RESERVED1;/****/
    volatile uint32_t RESERVED2;/****/
    volatile uint32_t AHB1ENR;/****/
    volatile uint32_t AHB2ENR;/****/
    volatile uint32_t AHB3ENR;/****/
    volatile uint32_t RESERVED3;/****/
    volatile uint32_t APB1ENR;/****/
    volatile uint32_t APB2ENR;/****/
    volatile uint32_t RESERVED4;/****/
    volatile uint32_t RESERVED5;/****/
    volatile uint32_t AHB1LPENR;/****/
    volatile uint32_t AHB2LPENR;/****/
    volatile uint32_t AHB3LPENR;/****/
    volatile uint32_t RESERVED6;/****/
    volatile uint32_t APB1LPENR;/****/
    volatile uint32_t APB2LPENR;/****/
    volatile uint32_t RESERVED7;/****/
    volatile uint32_t RESERVED8;/****/
    volatile uint32_t BDCR;/****/
    volatile uint32_t CSR;/****/
    volatile uint32_t RESERVED9;/****/
    volatile uint32_t RESERVED10;/****/
    volatile uint32_t SSCGR;/****/
    volatile uint32_t PLLI2SCFGR;/****/
    volatile uint32_t PLLSAICFGR;/****/
    volatile uint32_t DCKCFGR;/****/
} RCC_Registers_t;

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
#endif /* INC_STM32F429XX_H_ */
