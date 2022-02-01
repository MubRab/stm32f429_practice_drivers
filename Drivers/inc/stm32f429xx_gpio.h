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
#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

/**
 * Pin Modes
 */
#define GPIO_PIN_MODE_IN			0
#define GPIO_PIN_MODE_OUT			1
#define GPIO_PIN_MODE_ALT_FUNC		2
#define GPIO_PIN_MODE_ANALOG		3

/**
 * Pin output type
 */
#define GPIO_PIN_OUT_PP             0
#define GPIO_PIN_OUT_OPEN_DRAIN     1
/**
 * Pin speeds
 */
#define GPIO_PIN_SPEED_LOW			0
#define GPIO_PIN_SPEED_MED			1
#define GPIO_PIN_SPEED_HIGH			3
#define GPIO_PIN_SPEED_VHIGH		4

/**
 * Pin pull-up or pull-down
 */
#define GPIO_PIN_NOPUPD				0
#define GPIO_PIN_PULL_UP			1
#define GPIO_PIN_PULL_DOWN			2

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
void GPIO_Clk(GPIO_Registers_t *pGPIOx, uint8_t en);/****/
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
void GPIO_IRQHandler(uint8_t pinNumber);/****/
#endif /* INC_STM32F429XX_GPIO_H_ */
