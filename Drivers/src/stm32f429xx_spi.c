/*
 * stm32f429xx_spi.c
 *
 *  Created on: 03 Feb 2022
 *      Author:
 */
#include "stm32f429xx_spi.h"

static void SPI_Clk(SPI_Registers_t *pSPIx, uint8_t en);
//static void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/**
 *TODO: instead of shifting by a literal number, rather use macros for the offset
 *TODO: e.g. pSPIHandle->pSPIx->CR1 &= ~(1 << 15);                      BAD
 *TODO:   -> pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDI_MODE);    BETTER
 *TODO:   define macros in MCU specific header file
 */

/**
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    SPI_Clk(pSPIHandle->pSPIx, ENABLE);

    pSPIHandle->pSPIx->CR1 = 0;

    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_Config.Device_Mode << 2);

    if (pSPIHandle->SPI_Config.Bus_Config == SPI_BUS_FULL_DUPLEX)
    {
        pSPIHandle->pSPIx->CR1 &= ~(1 << 15);
    }
    else if (pSPIHandle->SPI_Config.Bus_Config == SPI_BUS_HALF_DUPLEX)
    {
        pSPIHandle->pSPIx->CR1 |= (1 << 15);
    }
    else if (pSPIHandle->SPI_Config.Bus_Config == SPI_BUS_RXONLY)
    {
        pSPIHandle->pSPIx->CR1 &= ~(1 << 15);
        pSPIHandle->pSPIx->CR1 |= (1 << 10);
    }

    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_Config.DFF << 11);

    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_Config.CPHA << 0);

    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_Config.CPOL << 1);

    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_Config.SSM << 9);

    if (pSPIHandle->SPI_Config.SSM == SPI_SSM_EN)
    {
        if (pSPIHandle->SPI_Config.SSI == SPI_SSI_DI)
        {
            pSPIHandle->pSPIx->CR1 &=  ~(1 << 8);
        }
        else if (pSPIHandle->SPI_Config.SSI == SPI_SSI_EN)
        {
            pSPIHandle->pSPIx->CR1 |=  (1 << 8);
        }
    }

    pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPI_Config.CLK_Speed << 3);
}

/**
 *TODO
 */
void SPI_Reset(SPI_Registers_t *pSPIx)
{

}

/**
 *
 */
static void SPI_Clk(SPI_Registers_t *pSPIx, uint8_t en)
{
    if (en == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_CLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_CLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_CLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_CLK_EN();
        }
        else if (pSPIx == SPI5)
        {
            SPI5_CLK_EN();
        }
        else if (pSPIx == SPI6)
        {
            SPI6_CLK_EN();
        }

    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_CLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_CLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_CLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_CLK_DI();
        }
        else if (pSPIx == SPI5)
        {
            SPI5_CLK_DI();
        }
        else if (pSPIx == SPI6)
        {
            SPI6_CLK_DI();
        }

    }
}

/**
 *
 */
void SPI_Control(SPI_Registers_t *pSPIx, uint8_t EN)
{
    if (EN == ENABLE)
    {
        pSPIx->CR1 |= (1 << 6);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << 6);
    }
}

/**
 *pData is an array of length size consisting of one byte of data per index
 *
 *polling based sending of data
 */
void SPI_SendData(SPI_Registers_t *pSPIx, uint8_t *pData, uint32_t size)
{
    while (size > 0)
    {
        while (!(pSPIx->SR & (1 << 1)));

        if (pSPIx->CR1 & (1 << 11)) /**16-bit**/
        {
            pSPIx->DR = *((uint16_t*) pData);
            size -= 2;
            pData += 2;
        }
        else /**8-bit**/
        {
            pSPIx->DR = *pData;
            --size;
            ++pData;
        }

    }

}

/**
 *
 */
void SPI_ReceiveData(SPI_Registers_t *pSPIx, uint8_t *pRxBuffer, uint32_t size)
{

}

/**
 *
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en)
{

}

/**
 *
 */
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle)
{

}

