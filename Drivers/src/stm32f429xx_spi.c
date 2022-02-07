/*
 * stm32f429xx_spi.c
 *
 *  Created on: 03 Feb 2022
 *      Author:
 */
#include "stm32f429xx_spi.h"

static void SPI_Clk(SPI_Registers_t *pSPIx, uint8_t en);
static void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
static void SPI_IRQHandler_Tx(SPI_Handle_t *pSPIHandle);
static void SPI_IRQHandler_Rx(SPI_Handle_t *pSPIHandle);
static void SPI_IRQHandler_Error(SPI_Handle_t *pSPIHandle);

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
uint8_t SPI_Control(SPI_Registers_t *pSPIx, uint8_t EN)
{
    if (EN == ENABLE)
    {
        pSPIx->CR1 |= (1 << 6);
        return 1;
    }
    else
    {
        if (pSPIx->SR & (1 << 7)) /**Checks if SPI communication is busy*/
            return 0;

        pSPIx->CR1 &= ~(1 << 6);
        return 1;
    }
}

/**
 *pData is an array of length size consisting of one byte of data per index
 *
 *polling based sending of data
 */
void SPI_SendData(SPI_Registers_t *pSPIx, uint8_t *pTxData, uint32_t size)
{
    while (size > 0)
    {
        while (!(pSPIx->SR & (1 << 1)));

        if (pSPIx->CR1 & (1 << 11)) /**16-bit**/
        {
            pSPIx->DR = *((uint16_t*) pTxData);
            size -= 2;
            pTxData += 2;
        }
        else /**8-bit**/
        {
            pSPIx->DR = *pTxData;
            --size;
            ++pTxData;
        }

    }

}

/**
 *
 */
void SPI_ReceiveData(SPI_Registers_t *pSPIx, uint8_t *pRxData, uint32_t size)
{
    while (size > 0)
    {
        while (!pSPIx->SR & (1 << 0));

        if (pSPIx->CR1 & (1 << 11))
        {
            (*(uint16_t*) pRxData) = pSPIx->DR;
            size -= 2;
            pRxData += 2;
        }
        else
        {
            *pRxData = pSPIx->DR;
            --size;
            ++pRxData;
        }
    }

}
/**
 * Send data in interrupt mode
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxData, uint32_t size)
{
    if (pSPIHandle->TxState != SPI_TX_BUSY)
    {
        pSPIHandle->pTxBuffer = pTxData;
        pSPIHandle->TxSize = size;

        pSPIHandle->TxState = SPI_TX_BUSY;

        pSPIHandle->pSPIx->CR2 |= (1 << 7);
    }

    return pSPIHandle->TxState;
}

/**
 * Receive data in interrupt mode
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxData, uint32_t size)
{
    if (pSPIHandle->RxState != SPI_RX_BUSY)
    {
        pSPIHandle->pRxBuffer = pRxData;
        pSPIHandle->RxSize = size;

        pSPIHandle->RxState = SPI_RX_BUSY;

        pSPIHandle->pSPIx->CR2 |= (1 << 6);
    }

    return pSPIHandle->RxState;

}

/**
 *
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en)
{
    if (en == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 31 || IRQNumber <= 63)
        {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        SPI_IRQPriorityConfig(IRQNumber, IRQPriority);
    }
    else
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 31 || IRQNumber <= 63)
        {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
    }
}

/**
 *
 */
static void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t prix = (IRQNumber % 4) * 8;
    /**
     * (prix + (8 - NUM_PR_BITS_IMPLEMENTED) is necessary since the 4 LSB in the PRIx register is
     * not implemented and all writes will be ignored
     * NUM_PR_BITS_IMPLEMENTED is MCU specific. STM32 uses 4-bits
     */
    *(NVIC_IPR_BASE_ADDR + iprx) |= IRQPriority << (prix + (8 - NUM_PR_BITS_IMPLEMENTED));
}

/**
 *
 */
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle)
{
    /*TX*/
    if ((pSPIHandle->pSPIx->SR & (1 << 1)) && (pSPIHandle->pSPIx->CR2 & (1 << 7)))
    {
        SPI_IRQHandler_Tx(pSPIHandle);
    }

    /*RX*/
    if ((pSPIHandle->pSPIx->SR & (1 << 0)) && (pSPIHandle->pSPIx->CR2 & (1 << 6)))
    {
        SPI_IRQHandler_Rx(pSPIHandle);
    }

    /*Error for Overrun only*/
    if ((pSPIHandle->pSPIx->SR & (1 << 1)) && (pSPIHandle->pSPIx->CR2 & (1 << 5)))
    {
        SPI_IRQHandler_Error(pSPIHandle);
    }

}

/**
 *
 */
static void SPI_IRQHandler_Tx(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->pSPIx->CR1 & (1 << 11)) /**16-bit**/
    {
        pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
        pSPIHandle->TxSize -= 2;
        pSPIHandle->pTxBuffer += 2;
    }
    else /**8-bit**/
    {
        pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
        --pSPIHandle->TxSize;
        ++pSPIHandle->pTxBuffer;
    }

    if (pSPIHandle->TxSize <= 0)
    {
        SPI_CloseTx(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
    }
}

/**
 *
 */
static void SPI_IRQHandler_Rx(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->pSPIx->CR1 & (1 << 11))
    {
        (*(uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxSize -= 2;
        pSPIHandle->pRxBuffer += 2;
    }
    else
    {
        *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        --pSPIHandle->RxSize;
        ++pSPIHandle->pRxBuffer;
    }


    if (pSPIHandle->RxSize <= 0)
    {
        SPI_CloseRx(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
    }
}

/**
 *
 */
static void SPI_IRQHandler_Error(SPI_Handle_t *pSPIHandle)
{
    if (pSPIHandle->TxState == SPI_TX_BUSY)
    {
        uint8_t temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
        (void) temp;
    }

    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVERRUN_ERROR);
}

void SPI_ClearOVRFlag(SPI_Registers_t *pSPIx)
{
    uint8_t temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void) temp;
}

void SPI_CloseTx(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << 7);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxSize = 0;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseRx(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << 6);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxSize = 0;
    pSPIHandle->RxState = SPI_READY;
}

/**
 * weak function, can be overriden by user
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event)
{

}

