/*
 * stm32f429xx_i2c.c
 *
 *  Created on: 08 Feb 2022
 *      Author:
 */
#include "stm32f429xx_i2c.h"

static void I2C_Clk(I2C_Registers_t *pI2Cx, uint8_t en);
static void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    I2C_Clk(pI2CHandle->pI2Cx, ENABLE);

}


void I2C_Reset(I2C_Registers_t *pI2Cx)
{
    if (pI2Cx == I2C1)
    {
        I2C1_RESET();
    }
    else if (pI2Cx == I2C2)
    {
        I2C2_RESET();
    }
    else if (pI2Cx == I2C3)
    {
        I2C3_RESET();
    }

}

uint8_t I2C_Control(I2C_Registers_t *pI2Cx, uint8_t EN)
{
    if (EN == ENABLE)
    {
        pI2Cx->CR1 |= (1 << 0);
        return 1;
    }
    else
    {
        if (pI2Cx->SR2 & (1 << 1))
        {
            return 0;
        }
        pI2Cx->CR1 &= ~(1 << 0);
        return 1;
    }
}

static void I2C_Clk(I2C_Registers_t *pI2Cx, uint8_t en)
{
    if (en == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_CLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_CLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_CLK_EN();
        }
    }
    else
    {
        if (pI2Cx == I2C1)
        {
            I2C1_CLK_DI();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_CLK_DI();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_CLK_DI();
        }
    }
}
