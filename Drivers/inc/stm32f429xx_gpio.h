/*
 * stm32f429xx_gpio.h
 *
 *  Created on: 01 Feb 2022
 *      Author:
 */

#ifndef INC_STM32F429XX_GPIO_H_
#define INC_STM32F429XX_GPIO_H_

#include "stm32f429xx.h"

/**
 * GPIO pin macros to be used by user when configuring the pins
 */
/**
 * Pin numbers
 */
#define GPIO_PIN_0					(uint32_t) 0
#define GPIO_PIN_1					(uint32_t) 1
#define GPIO_PIN_2					(uint32_t) 2
#define GPIO_PIN_3					(uint32_t) 3
#define GPIO_PIN_4					(uint32_t) 4
#define GPIO_PIN_5					(uint32_t) 5
#define GPIO_PIN_6					(uint32_t) 6
#define GPIO_PIN_7					(uint32_t) 7
#define GPIO_PIN_8					(uint32_t) 8
#define GPIO_PIN_9					(uint32_t) 9
#define GPIO_PIN_10					(uint32_t) 10
#define GPIO_PIN_11					(uint32_t) 11
#define GPIO_PIN_12					(uint32_t) 12
#define GPIO_PIN_13					(uint32_t) 13
#define GPIO_PIN_14					(uint32_t) 14
#define GPIO_PIN_15					(uint32_t) 15

/**
 * Pin Modes
 */
#define GPIO_PIN_MODE_IN			(uint32_t) 0
#define GPIO_PIN_MODE_OUT			(uint32_t) 1
#define GPIO_PIN_MODE_ALT_FUNC		(uint32_t) 2
#define GPIO_PIN_MODE_ANALOG		(uint32_t) 3
/**
 * Interrupt pin modes
 */
#define GPIO_PIN_MODE_IT_FALLING    (uint32_t) 4
#define GPIO_PIN_MODE_IT_RISING     (uint32_t) 5
#define GPIO_PIN_MODE_IT_BOTH       (uint32_t) 6

/**
 * Pin output type
 */
#define GPIO_PIN_OUT_PP             (uint32_t) 0
#define GPIO_PIN_OUT_OPEN_DRAIN     (uint32_t) 1
/**
 * Pin speeds
 */
#define GPIO_PIN_SPEED_LOW			(uint32_t) 0
#define GPIO_PIN_SPEED_MED			(uint32_t) 1
#define GPIO_PIN_SPEED_HIGH			(uint32_t) 3
#define GPIO_PIN_SPEED_VHIGH		(uint32_t) 4

/**
 * Pin pull-up or pull-down
 */
#define GPIO_PIN_NOPUPD				(uint32_t) 0
#define GPIO_PIN_PULL_UP			(uint32_t) 1
#define GPIO_PIN_PULL_DOWN			(uint32_t) 2

/**
 * Struct for configuring each pin of the selected GPIO port
 */
typedef struct
{
    uint32_t pin_number;/****/
    uint32_t pin_mode;/****/
    uint32_t pin_speed;/****/
    uint32_t pin_out_type;/****/
    uint32_t pin_pupd;/****/
    uint32_t pin_alt_func;/****/
} GPIO_Pin_Config_t;

/**
 * Struct for handling each gpio pin
 */
typedef struct
{
	GPIO_Registers_t *pGPIOx;
	GPIO_Pin_Config_t *pGPIOx_Pin_Config;
} GPIO_Handle_t;

/**
 * API function prototypes
 */
/**
 * Initializing and Reseting
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);/****/
void GPIO_Reset(GPIO_Registers_t *pGPIOx);/****/
//static void GPIO_Clk(GPIO_Registers_t *pGPIOx, uint8_t en);/****/
/**
 * Reading and writing
 */
uint8_t GPIO_ReadPin(GPIO_Registers_t *pGPIOx, uint8_t pinNumber);/****/
uint16_t GPIO_ReadPort(GPIO_Registers_t *pGPIOx);/****/
void GPIO_WritePin(GPIO_Registers_t *pGPIOx, uint8_t pinNumber, uint8_t value);/****/
void GPIO_WritePort(GPIO_Registers_t *pGPIOx, uint16_t value);/****/
void GPIO_TogglePin(GPIO_Registers_t *pGPIOx, uint8_t pinNumber);/****/
/**
 * Interrupts
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en);/****/
//static void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);/****/
void GPIO_IRQHandler(uint8_t pinNumber);/****/
#endif /* INC_STM32F429XX_GPIO_H_ */
