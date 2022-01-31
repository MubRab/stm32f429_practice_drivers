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
#define EXTI_BASE_ADDRESS						(APB2_BASE_ADDR + 0x3800U)
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

#endif /* INC_STM32F429XX_H_ */
