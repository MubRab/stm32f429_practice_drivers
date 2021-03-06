/*
 * stm32f429xx_usart.c
 *
 *  Created on: 16 Feb 2022
 *      Author:
 */
#include "stm32f429xx_usart.h"
#include "stm32f429xx_rcc.h"

static void USART_Clk(USART_Registers_t *pUSARTx, uint8_t en);
static void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

void USART_Init(USART_Handle_t *pUSARTHandle)
{
    USART_Clk(pUSARTHandle->pUSARTx, ENABLE);

    if (pUSARTHandle->USARTConfig.Mode == USART_MODE_ONLY_RX)
    {
        pUSARTHandle->pUSARTx->CR1 |= (1 << 2);
    }
    else if (pUSARTHandle->USARTConfig.Mode == USART_MODE_ONLY_TX)
    {
        pUSARTHandle->pUSARTx->CR1 |= (1 << 3);
    }
    else if (pUSARTHandle->USARTConfig.Mode == USART_MODE_TXRX)
    {
        pUSARTHandle->pUSARTx->CR1 |= (1 << 2);
        pUSARTHandle->pUSARTx->CR1 |= (1 << 3);
    }

    uint32_t usart_div;
    if (pUSARTHandle->pUSARTx == USART1 || pUSARTHandle->pUSARTx == USART6)
    {
    usart_div = (uint32_t) ((GetPCLKAPB1() /
            (8 * (2 - pUSARTHandle->USARTConfig.OversamplingMode) * pUSARTHandle->USARTConfig.BaudRate)) * 100);
    }
    else
    {
        usart_div = (uint32_t) ((GetPCLKAPB2() /
                    (8 * (2 - pUSARTHandle->USARTConfig.OversamplingMode) * pUSARTHandle->USARTConfig.BaudRate)) * 100);
    }

    uint16_t div_mantissa = (uint16_t) (usart_div/100);
    uint8_t div_fraction = (uint8_t) (((usart_div - (div_mantissa * 100) * (8 * (2 - pUSARTHandle->USARTConfig.OversamplingMode))) + 50)/100);

    if (pUSARTHandle->USARTConfig.OversamplingMode == USART_OVER_8)
    {
        pUSARTHandle->pUSARTx->CR1 |= (1 << 15);
        div_fraction = (uint8_t) (div_fraction & (0x7));
    }
    else
    {
        div_fraction = (uint8_t) (div_fraction & (0xF));
    }

    pUSARTHandle->pUSARTx->BRR |= ((div_mantissa << 4) | (div_fraction));

    pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USARTConfig.StopBits << 12);

    pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->USARTConfig.WordLength << 12);

    if (pUSARTHandle->USARTConfig.WordLength == USART_PARITY_EN_EVEN)
    {
        pUSARTHandle->pUSARTx->CR1 |= (1 << 10);
    }
    else if (pUSARTHandle->USARTConfig.WordLength == USART_PARITY_EN_ODD)
    {
        pUSARTHandle->pUSARTx->CR1 |= (1 << 10);
        pUSARTHandle->pUSARTx->CR1 |= (1 << 9);
    }

    if (pUSARTHandle->USARTConfig.HWFlowCtrl == USART_HW_FLOW_CTRL_CTS)
    {
        pUSARTHandle->pUSARTx->CR3 |= (1 << 9);
    }
    else if (pUSARTHandle->USARTConfig.HWFlowCtrl == USART_HW_FLOW_CTRL_RTS)
    {
        pUSARTHandle->pUSARTx->CR3 |= (1 << 8);
    }
    if (pUSARTHandle->USARTConfig.HWFlowCtrl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        pUSARTHandle->pUSARTx->CR3 |= (1 << 8);
        pUSARTHandle->pUSARTx->CR3 |= (1 << 9);
    }

}

void USART_Reset(USART_Registers_t *pUSARTx)
{
    if (pUSARTx == USART1)
    {
        USART1_RESET();
    }
    else if (pUSARTx == USART2)
    {
        USART2_RESET();
    }
    else if (pUSARTx == USART3)
    {
        USART3_RESET();
    }
    else if (pUSARTx == UART4)
    {
        UART4_RESET();
    }
    else if (pUSARTx == UART5)
    {
        UART5_RESET();
    }
    else if (pUSARTx == USART6)
    {
        USART6_RESET();
    }
    else if (pUSARTx == UART7)
    {
        UART7_RESET();
    }
    else if (pUSARTx == UART8)
    {
        UART8_RESET();
    }
}

void USART_Clk(USART_Registers_t *pUSARTx, uint8_t en)
{
    if (en == ENABLE)
    {
        if (pUSARTx == USART1)
        {
            USART1_CLK_EN();
        }
        else if (pUSARTx == USART2)
        {
            USART2_CLK_EN();
        }
        else if (pUSARTx == USART3)
        {
            USART3_CLK_EN();
        }
        else if (pUSARTx == UART4)
        {
            UART4_CLK_EN();
        }
        else if (pUSARTx == UART5)
        {
            UART5_CLK_EN();
        }
        else if (pUSARTx == USART6)
        {
            USART6_CLK_EN();
        }
        else if (pUSARTx == UART7)
        {
            UART7_CLK_EN();
        }
        else if (pUSARTx == UART8)
        {
            UART8_CLK_EN();
        }
    }
    else
    {
        if (pUSARTx == USART1)
        {
            USART1_CLK_DI();
        }
        else if (pUSARTx == USART2)
        {
            USART2_CLK_DI();
        }
        else if (pUSARTx == USART3)
        {
            USART3_CLK_DI();
        }
        else if (pUSARTx == UART4)
        {
            UART4_CLK_DI();
        }
        else if (pUSARTx == UART5)
        {
            UART5_CLK_DI();
        }
        else if (pUSARTx == USART6)
        {
            USART6_CLK_DI();
        }
        else if (pUSARTx == UART7)
        {
            UART7_CLK_DI();
        }
        else if (pUSARTx == UART8)
        {
            UART8_CLK_DI();
        }
    }
}

uint8_t USART_Control(USART_Registers_t *pUSART, uint8_t EN)
{
    if (EN == ENABLE)
    {
        pUSART->CR1 |= (1 << 13);
        return 1;
    }
    else
    {
        if (pUSART->SR & (1 << 6))
        {
            pUSART->CR1 &= ~(1 << 13);
            return 1;
        }
        return 0;
    }
}

void USART_SendData(USART_Registers_t *pUSARTx,uint8_t *pTxData, uint32_t size)
{
    for (uint32_t i = 0; i < size; ++i)
    {
        while (!(pUSARTx->SR & (1 << 7)));

        if (pUSARTx->CR1 & (1 << 12))
        {
            /*9-bits*/
            pUSARTx->DR = (*((uint16_t*)pTxData) & 0x01FF); /*Only use the first 9-bits, mask the rest*/

            if (pUSARTx->CR1 & (1 << 10))
            {
                /*parity enabled*/
                ++pTxData;
            }
            else
            {
                pTxData += 2;
            }
        }
        else
        {
            /*8-bits*/
            pUSARTx->DR = (*(pTxData) & 0xFF);
            ++pTxData;
        }
    }

    while (!(pUSARTx->SR & (1 << 6)));
}

void USART_ReceiveData(USART_Registers_t *pUSARTx, uint8_t *pRxData, uint32_t size)
{
    for (uint32_t i = 0; i < size; ++i)
    {
        while (!(pUSARTx->SR & (1 << 5)));

        if (pUSARTx->CR1 & (1 << 12))
        {
            /*9-bits*/

            if (pUSARTx->CR1 & (1 << 10))
            {
                /*parity enabled*/
                *pRxData = (uint8_t)(pUSARTx->DR & 0xFF);
                ++pRxData;
            }
            else
            {
                *((uint16_t*)pRxData) = (uint16_t)(pUSARTx->DR & 0x01FF);
                pRxData += 2;
            }
        }
        else
        {
            /*8-bits*/

            if (pUSARTx->CR1 & (1 << 10))
            {
                /*parity enabled*/
                *pRxData = (uint8_t)(pUSARTx->DR & 0x7F);

            }
            else
            {
                *pRxData = (uint8_t)pUSARTx->DR;
            }

            ++pRxData;
        }
    }
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxData, uint32_t size)
{
    uint8_t state = pUSARTHandle->TxState;

    if (state != USART_STATE_BUSY_TX)
    {
        pUSARTHandle->TxSize = size;
        pUSARTHandle->pTxData = pTxData;
        pUSARTHandle->TxState = USART_STATE_BUSY_TX;

        pUSARTHandle->pUSARTx->CR1 |= (1 << 7);
        pUSARTHandle->pUSARTx->CR1 |= (1 << 6);
    }

    return state;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxData, uint32_t size)
{
    uint8_t state = pUSARTHandle->RxState;

    if (state != USART_STATE_BUSY_RX)
    {
        pUSARTHandle->RxSize = size;
        pUSARTHandle->pRxData = pRxData;
        pUSARTHandle->RxState = USART_STATE_BUSY_RX;

        pUSARTHandle->pUSARTx->CR1 |= (1 << 5);
    }

    return state;
}

void USART_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en)
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
        else if(IRQNumber >= 64 && IRQNumber < 96 )
        {
            *NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
        }
        USART_IRQPriorityConfig(IRQNumber, IRQPriority);
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
        else if(IRQNumber >= 6 && IRQNumber < 96 )
        {
            *NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
        }
    }
}

static void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
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

/*
 *
 */
void USART_IRQHandler(USART_Handle_t *pUSARTHandle)
{
    /*Transmission complete*/
    if ((pUSARTHandle->pUSARTx->SR & (1 << 6)) && (pUSARTHandle->pUSARTx->CR1 & (1 << 6)))
    {
        if (pUSARTHandle->TxState == USART_STATE_BUSY_TX && pUSARTHandle->TxSize == 0)
        {
            pUSARTHandle->pUSARTx->SR &= ~(1 << 6);
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << 6);
            pUSARTHandle->TxState = USART_STATE_READY;
            pUSARTHandle->pTxData = NULL;
            pUSARTHandle->TxSize = 0;
            USART_ApplicationEventCallback(pUSARTHandle, USART_EV_TX_COMPLETE);
        }
    }

    /*TXE Flag*/
    if ((pUSARTHandle->pUSARTx->SR & (1 << 7)) && (pUSARTHandle->pUSARTx->CR1 & (1 << 7)))
    {
        if (pUSARTHandle->TxState == USART_STATE_BUSY_TX && pUSARTHandle->TxSize > 0)
        {
            if (pUSARTHandle->USARTConfig.WordLength == USART_WORDLENGTH_9BITS &&
                    pUSARTHandle->USARTConfig.ParityBit == USART_PARITY_DISABLE)
            {
                pUSARTHandle->pUSARTx->DR = (uint16_t)((*(uint16_t*)pUSARTHandle->pTxData) & 0x01FF);
                (pUSARTHandle->pTxData) += 2;
                (pUSARTHandle->TxSize) -= 2;
            }
            else if (pUSARTHandle->USARTConfig.WordLength == USART_WORDLENGTH_9BITS &&
                        pUSARTHandle->USARTConfig.ParityBit != USART_PARITY_DISABLE)
            {
                pUSARTHandle->pUSARTx->DR = (uint16_t)((*(uint16_t*)pUSARTHandle->pTxData) & 0x01FF);
                ++(pUSARTHandle->pTxData);
                --(pUSARTHandle->TxSize);
            }
            else /*Wordlength = 8-bit*/
            {
                pUSARTHandle->pUSARTx->DR = (uint8_t)((*(uint8_t*)pUSARTHandle->pTxData) & 0xFF);
                ++(pUSARTHandle->pTxData);
                --(pUSARTHandle->TxSize);
            }
        }
        else if (pUSARTHandle->TxState == USART_STATE_BUSY_TX && pUSARTHandle->TxSize == 0)
        {
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << 7);
            /*the rest is handled by the TC handler*/
        }
    }

    /*RXNE Flag*/
    if ((pUSARTHandle->pUSARTx->SR & (1 << 5)) && (pUSARTHandle->pUSARTx->CR1 & (1 << 5)))
    {
        if (pUSARTHandle->RxState == USART_STATE_BUSY_RX && pUSARTHandle->RxSize > 0)
        {
            if (pUSARTHandle->USARTConfig.WordLength == USART_WORDLENGTH_9BITS &&
                    pUSARTHandle->USARTConfig.ParityBit == USART_PARITY_DISABLE)
            {
                *((uint16_t*)pUSARTHandle->pRxData) = (uint16_t)(pUSARTHandle->pUSARTx->DR & 0x01FF);
                (pUSARTHandle->pRxData) += 2;
                (pUSARTHandle->RxSize) -= 2;
            }
            else if ((pUSARTHandle->USARTConfig.WordLength == USART_WORDLENGTH_9BITS &&
                        pUSARTHandle->USARTConfig.ParityBit != USART_PARITY_DISABLE) ||
                    (pUSARTHandle->USARTConfig.WordLength == USART_WORDLENGTH_8BITS &&
                        pUSARTHandle->USARTConfig.ParityBit == USART_PARITY_DISABLE))
            {
                *(pUSARTHandle->pRxData) = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0xFF);
                ++(pUSARTHandle->pRxData);
                --(pUSARTHandle->RxSize);
            }
            else
            {
                *(pUSARTHandle->pRxData) = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0x7F);
                ++(pUSARTHandle->pRxData);
                --(pUSARTHandle->RxSize);
            }
        }
        else if (pUSARTHandle->RxState == USART_STATE_BUSY_RX && pUSARTHandle->RxSize == 0)
        {
            pUSARTHandle->pUSARTx->CR1 &= ~(1 << 5);
            pUSARTHandle->RxState = USART_STATE_READY;
            USART_ApplicationEventCallback(pUSARTHandle, USART_EV_RX_COMPLETE);
        }
    }

    /*CTS Flag*/
    if ((pUSARTHandle->pUSARTx->SR & (1 << 9)) && (pUSARTHandle->pUSARTx->CR3 & (1 << 9)) && (pUSARTHandle->pUSARTx->CR3 & (1 << 10)))
    {
        pUSARTHandle->pUSARTx->SR &= ~(1 << 9);
        USART_ApplicationEventCallback(pUSARTHandle, USART_EV_CTS);
    }

    /*IDLE Flag*/
    if ((pUSARTHandle->pUSARTx->SR & (1 << 4)) && (pUSARTHandle->pUSARTx->CR1 & (1 << 4)))
    {
        uint8_t tempRead = pUSARTHandle->pUSARTx->DR;
        (void)tempRead;
        USART_ApplicationEventCallback(pUSARTHandle, USART_EV_IDLE);
    }

    /*ORE Flag*/
    if ((pUSARTHandle->pUSARTx->SR & (1 << 3)) && (pUSARTHandle->pUSARTx->CR1 & (1 << 5)))
    {
        uint8_t tempRead = pUSARTHandle->pUSARTx->DR;
        (void)tempRead;
        USART_ApplicationEventCallback(pUSARTHandle, USART_EV_ORE);
    }

    /*other error flags not added, since we are not working with multi-buffer communication*/
}
