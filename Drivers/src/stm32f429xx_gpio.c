/*
 * stm32f429xx_gpio.c
 *
 *  Created on: 01 Feb 2022
 *      Author:
 */
#include "stm32f429xx_gpio.h"

/**
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    GPIO_Clk(pGPIOHandle->pGPIOx, ENABLE);

    uint32_t pin_num = pGPIOHandle->pGPIOx_Pin_Config->pin_number;

    if (pGPIOHandle->pGPIOx_Pin_Config->pin_mode <= GPIO_PIN_MODE_ANALOG)
    {
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pin_num));
        pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->pGPIOx_Pin_Config->pin_mode) << (2 * pin_num);
    }
    else /**interrupt mode**/
    {
        SYSCFG_CLK_EN();

        EXTI->IMR |= 1 << pin_num;

        if (pGPIOHandle->pGPIOx_Pin_Config->pin_mode == GPIO_PIN_MODE_IT_FALLING)
        {
            EXTI->FTSR |= 1 << pin_num;
            EXTI->RTSR &= ~(1 << pin_num);
        }
        else if (pGPIOHandle->pGPIOx_Pin_Config->pin_mode == GPIO_PIN_MODE_IT_RISING)
        {
            EXTI->RTSR |= 1 << pin_num;
            EXTI->FTSR &= ~(1 << pin_num);
        }
        else if (pGPIOHandle->pGPIOx_Pin_Config->pin_mode == GPIO_PIN_MODE_IT_BOTH)
        {
            EXTI->FTSR |= 1 << pin_num;
            EXTI->RTSR |= 1 << pin_num;
        }

        //configure GPIO port in SYSCFG_EXTICR
        uint32_t reg_num = (uint32_t) pin_num / 4;
        if (pGPIOHandle->pGPIOx == GPIOA)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x0 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOB)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x01 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOC)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x2 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOD)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x3 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOE)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x4 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOF)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x5 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOG)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x6 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOH)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x7 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOI)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x8 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOJ)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x9 << (4 * (pin_num % 4));
        }
        else if (pGPIOHandle->pGPIOx == GPIOK)
        {
            SYSCFG->EXTICR[reg_num] &= ~(0xF << (4 * (pin_num % 4)));
            SYSCFG->EXTICR[reg_num] |= 0x10 << (4 * (pin_num % 4));
        }
    }

    pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pin_num);
    pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->pGPIOx_Pin_Config->pin_out_type) << pin_num;

    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pin_num));
    pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->pGPIOx_Pin_Config->pin_speed << (2 * pin_num));

    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pin_num));
    pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->pGPIOx_Pin_Config->pin_pupd << (2 * pin_num));

    if (pin_num <= GPIO_PIN_7)
    {
        pGPIOHandle->pGPIOx->AFRL &= ~(0xF << (4 * pin_num));
        pGPIOHandle->pGPIOx->AFRL |= pGPIOHandle->pGPIOx_Pin_Config->pin_alt_func << (4 * pin_num);
    }
    else if (pin_num >= GPIO_PIN_8 || pin_num <= GPIO_PIN_15)
    {
        pGPIOHandle->pGPIOx->AFRH &= ~(0xF << (4 * (pin_num % 8)));
        pGPIOHandle->pGPIOx->AFRH |= pGPIOHandle->pGPIOx_Pin_Config->pin_alt_func << (4 * (pin_num % 8));
    }
}

/**
 *
 */
void GPIO_Reset(GPIO_Registers_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_RESET();
    }
    else if (pGPIOx == GPIOJ)
    {
        GPIOJ_RESET();
    }
    else if (pGPIOx == GPIOK)
    {
        GPIOK_RESET();
    }
}

/**
 * is automatically called during init
 */
void GPIO_Clk(GPIO_Registers_t *pGPIOx, uint8_t en)
{
    if (en == ENABLE) {
        if (pGPIOx == GPIOA)
        {
            GPIOA_CLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_CLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_CLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_CLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_CLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_CLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_CLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_CLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_CLK_EN();
        }
        else if (pGPIOx == GPIOJ)
        {
            GPIOJ_CLK_EN();
        }
        else if (pGPIOx == GPIOK)
        {
            GPIOK_CLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_CLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_CLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_CLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_CLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_CLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_CLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_CLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_CLK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_CLK_DI();
        }
        else if (pGPIOx == GPIOJ)
        {
            GPIOJ_CLK_DI();
        }
        else if (pGPIOx == GPIOK)
        {
            GPIOK_CLK_DI();
        }
    }
}

/**
 *
 */
uint8_t GPIO_ReadPin(GPIO_Registers_t *pGPIOx, uint8_t pinNumber)
{
    return (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);
}

/**
 *
 */
uint16_t GPIO_ReadPort(GPIO_Registers_t *pGPIOx)
{
    return (uint16_t) (pGPIOx->IDR);
}

/**
 *
 */
void GPIO_WritePin(GPIO_Registers_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
    pGPIOx->ODR |= (value << pinNumber);
}

/**
 *
 */
void GPIO_WritePort(GPIO_Registers_t *pGPIOx, uint16_t value)
{
    pGPIOx->ODR = value;
}

/**
 *
 */
void GPIO_TogglePin(GPIO_Registers_t *pGPIOx, uint8_t pinNumber)
{
    pGPIOx->ODR ^= (1 << pinNumber);
}

/**
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en)
{

}

/**
 *
 */
void GPIO_IRQHandler(uint8_t pinNumber)
{

}
