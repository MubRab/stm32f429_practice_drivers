/*
 * stm32f429xx_i2c.c
 *
 *  Created on: 08 Feb 2022
 *      Author:
 */
#include "stm32f429xx_i2c.h"

static void I2C_Clk(I2C_Registers_t *pI2Cx, uint8_t en);
static void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

static uint32_t GetPCLK1(void)
{
    uint8_t clk_type = ((RCC->CFGR >> 2) & 0x3);
    uint32_t clk_speed = 16000000;/*default value*/

    /*Identifying clk source*/
    if (clk_type == 0)
    {
        clk_speed = 16000000;
    }
    else if (clk_type == 1)
    {
        clk_speed = 8000000;
    }
    else
    {
        /*PLLCLK source. Not needed*/
    }

    /*Pre-scaler*/
    uint16_t prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
    uint8_t idx = ((RCC->CFGR >> 4) & 0xF);
    if (idx >= 8)
    {
        clk_speed /= prescaler[idx-8];
    }

    uint16_t apb1_prescaler[4] = {2, 4, 8, 16};
    idx = ((RCC->CFGR >> 4) & 0x7);
    if (idx >= 4)
    {
        clk_speed /= apb1_prescaler[idx-4];
    }

    return clk_speed;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    I2C_Clk(pI2CHandle->pI2Cx, ENABLE);

//    pI2CHandle->pI2Cx->CR1 |= (pI2CHandle->I2CConfig.ACKControl << 10);

    uint8_t clk_speed = (uint8_t) GetPCLK1();
    pI2CHandle->pI2Cx->CR2 |= (clk_speed/1000000U);

    pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2CConfig.DeviceAddress << 1);
    pI2CHandle->pI2Cx->OAR1 |= (1 << 14);

    uint32_t ccr, t_rise, t_pclk, trise_reg = 0U;
    if (pI2CHandle->I2CConfig.SCLSpeed <= I2C_SCL_FREQ_SM)
    {
        pI2CHandle->pI2Cx->CCR |= (I2C_MODE_SM << 15);

        t_rise = (1/pI2CHandle->I2CConfig.SCLSpeed) / 2;
        t_pclk = (1/clk_speed);
        ccr = t_rise / t_pclk;

        trise_reg = (uint8_t) (((1000/1000000000) / t_pclk) + 1);
        pI2CHandle->pI2Cx->TRISE |= trise_reg;
    }
    else if (pI2CHandle->I2CConfig.SCLSpeed <= I2C_SCL_FREQ_FM && pI2CHandle->I2CConfig.FMDutyCycle == 0)
    {
        pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2CConfig.FMDutyCycle << 14);
        pI2CHandle->pI2Cx->CCR |= (I2C_MODE_FM << 15);

        t_rise = (1 / pI2CHandle->I2CConfig.SCLSpeed) / 3;
        t_pclk = (1 / clk_speed);
        ccr = t_rise / t_pclk;

        trise_reg = (uint8_t) (((300/1000000000U) / t_pclk) + 1);
        pI2CHandle->pI2Cx->TRISE |= trise_reg;
    }
    else if (pI2CHandle->I2CConfig.SCLSpeed <= I2C_SCL_FREQ_FM && pI2CHandle->I2CConfig.FMDutyCycle == 1)
    {
        pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2CConfig.FMDutyCycle << 14);
        pI2CHandle->pI2Cx->CCR |= (I2C_MODE_FM << 15);

        t_rise = (1 / pI2CHandle->I2CConfig.SCLSpeed) / 25;
        t_pclk = (1 / clk_speed);
        ccr = t_rise / (9 * t_pclk);

        trise_reg = (uint8_t) (((300/1000000000U) / t_pclk) + 1);
        pI2CHandle->pI2Cx->TRISE |= trise_reg;
    }
    pI2CHandle->pI2Cx->CCR |= ccr;

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

void I2C_ACKControl(I2C_Registers_t *pI2Cx, uint8_t EN)
{
    if (EN == ENABLE)
    {
        pI2Cx->CR1 |= (1 << 10);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << 10);
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

/**
 * see Fig 243-Transfer Sequence Diagram, in Reference Manual
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t size, uint8_t slaveAddr, uint8_t repeatedStart)
{
    /*1. Start condition*/
    pI2CHandle->pI2Cx->CR1 |= (1 << 8);

    /*2. EV5: wait until SB is cleared in SR1*/
    while (!pI2CHandle->pI2Cx->SR1 & (1 << 0));

    /*3. Send address*/
    pI2CHandle->pI2Cx->DR &= ~(1 << 0);
    pI2CHandle->pI2Cx->DR |= (slaveAddr << 1);

    /*4. EV6: wait until ADDR is cleared. Read SR1 then SR2*/
    while (!(pI2CHandle->pI2Cx->SR1 & (1 << 1)));
    uint32_t read = pI2CHandle->pI2Cx->SR1;
    read = pI2CHandle->pI2Cx->SR2;
    (void) read;

    /*5. Write data in DR1*/
    while (size <= 0)
    {
        while (!(pI2CHandle->pI2Cx->SR1 & (1 << 7)));
        pI2CHandle->pI2Cx->DR = *pData;
        --size;
        ++pData;
    }

    /*7. EV8_2: TxE and BTF flags cleared + Stop Condition*/
    while (!(pI2CHandle->pI2Cx->SR1 & (1 << 7)));
    while (!(pI2CHandle->pI2Cx->SR1 & (1 << 2)));

    if (repeatedStart == DISABLE)
    {
        pI2CHandle->pI2Cx->CR1 |= (1 << 9);
    }
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t size, uint8_t slaveAddr, uint8_t repeatedStart)
{
    /*1. Start condition*/
    pI2CHandle->pI2Cx->CR1 |= (1 << 8);
    /*2. Confirm SB flag*/
    while (!pI2CHandle->pI2Cx->SR1 & (1 << 0));
    /*3. Send Address*/
    pI2CHandle->pI2Cx->DR |= (1 << 0);
    pI2CHandle->pI2Cx->DR |= (slaveAddr << 1);
    /*4. Check ADDR flag*/
    while (!(pI2CHandle->pI2Cx->SR1 & (1 << 1)));

    /*5a. Reading only one byte*/
    if (size == 1)
    {
        /*Disable ACK*/
//        pI2CHandle->pI2Cx->CR1 &= ~(1 << 10);
        I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
        /*Clear ADDR flag*/
        while (!(pI2CHandle->pI2Cx->SR1 & (1 << 1)));
        uint32_t read = pI2CHandle->pI2Cx->SR1;
        read = pI2CHandle->pI2Cx->SR2;
        (void) read;
        /*Check RXNE flag*/
        while (!(pI2CHandle->pI2Cx->SR1 & (1 << 6)));
        /*STOP condition*/
        if (repeatedStart == DISABLE)
        {
            pI2CHandle->pI2Cx->CR1 |= (1 << 9);
        }
        /*Read data*/
        *pData = pI2CHandle->pI2Cx->DR;

        return;
    }

    /*5b. More than one byte*/
    if (size > 1)
    {
        /*clear addr flag*/
        while (!(pI2CHandle->pI2Cx->SR1 & (1 << 1)));
        uint32_t read = pI2CHandle->pI2Cx->SR1;
        read = pI2CHandle->pI2Cx->SR2;
        (void) read;

        /*read data until len becomes zero*/
        for (uint32_t i = size; i > 0; --i)
        {
            /*wait for RXNE*/
            while (!(pI2CHandle->pI2Cx->SR1 & (1 << 6)));

            /*check if last 2 bytes remaining*/
            if (i == 2)
            {
                /*clear ACK*/
//                pI2CHandle->pI2Cx->CR1 &= ~(1 << 10);
                I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
                /*STOP condition*/
                if (repeatedStart == DISABLE)
                {
                    pI2CHandle->pI2Cx->CR1 |= (1 << 9);
                }
            }

            /*Read data*/
            *pData = pI2CHandle->pI2Cx->DR;
            /*inc data pointer address*/
            ++pData;
        }
    }

    //re-enable ACK
    if (pI2CHandle->I2CConfig.ACKControl == I2C_ACK_EN)
    {
//        pI2CHandle->pI2Cx->CR1 |= (1 << 10);
        I2C_ACKControl(pI2CHandle->pI2Cx, ENABLE);
    }

}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t size, uint8_t slaveAddr, uint8_t repeatedStart)
{
    if (pI2CHandle->TxRxState == I2C_STATE_READY)
    {
        pI2CHandle->pTxData = pData;
        pI2CHandle->TxSize = size;
        pI2CHandle->SlaveAddress = slaveAddr;
        pI2CHandle->RepeatedStart = repeatedStart;
        pI2CHandle->TxRxState = I2C_STATE_BUSY_TX;

        /*start condition*/
        pI2CHandle->pI2Cx->CR1 |= (1 << 8);
        /*enable interrupts*/
        pI2CHandle->pI2Cx->CR2 |= (1 << 10);
        pI2CHandle->pI2Cx->CR2 |= (1 << 9);
        pI2CHandle->pI2Cx->CR2 |= (1 << 8);

        return I2C_STATE_READY;
    }

    return pI2CHandle->TxRxState;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t size, uint8_t slaveAddr, uint8_t repeatedStart)
{
    if (pI2CHandle->TxRxState == I2C_STATE_READY)
    {
        pI2CHandle->pRxData = pData;
        pI2CHandle->RxSize = size;
        pI2CHandle->RxSize_ = size;
        pI2CHandle->SlaveAddress = slaveAddr;
        pI2CHandle->RepeatedStart = repeatedStart;
        pI2CHandle->TxRxState = I2C_STATE_BUSY_RX;

        /*start condition*/
        pI2CHandle->pI2Cx->CR1 |= (1 << 8);
        /*enable interrupts*/
        pI2CHandle->pI2Cx->CR2 |= (1 << 10);
        pI2CHandle->pI2Cx->CR2 |= (1 << 9);
        pI2CHandle->pI2Cx->CR2 |= (1 << 8);

        return I2C_STATE_READY;
    }

    return pI2CHandle->TxRxState;
}

void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en)
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
        I2C_IRQPriorityConfig(IRQNumber, IRQPriority);
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

static void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
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
 * TODO: Too many nested-ifs, reduce by using functions
 */
void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle)
{
    if ((pI2CHandle->pI2Cx->CR2 & (1 << 9)) && (pI2CHandle->pI2Cx->SR1 & (1 << 0))) /*SB Event*/
    {
        /*send the address*/
        if (pI2CHandle->TxRxState == I2C_STATE_BUSY_RX)
        {
            pI2CHandle->pI2Cx->DR |= (1 << 0);
            pI2CHandle->pI2Cx->DR |= (pI2CHandle->SlaveAddress << 1);
        }
        else if (pI2CHandle->TxRxState == I2C_STATE_BUSY_TX)
        {
            pI2CHandle->pI2Cx->DR &= ~(1 << 0);
            pI2CHandle->pI2Cx->DR |= (pI2CHandle->SlaveAddress << 1);
        }
    }

    if ((pI2CHandle->pI2Cx->CR2 & (1 << 9)) && (pI2CHandle->pI2Cx->SR1 & (1 << 1))) /*ADDR event*/
    {
        /*clear the ADDR flag*/
        if ((pI2CHandle->TxRxState == I2C_STATE_BUSY_RX) && (pI2CHandle->pI2Cx->SR2 & (1 << 0)) && (pI2CHandle->RxSize == 1))
        {
            I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
        }
        uint32_t read = pI2CHandle->pI2Cx->SR1;
        read = pI2CHandle->pI2Cx->SR2;
        (void) read;
    }

    if ((pI2CHandle->pI2Cx->CR2 & (1 << 9)) && (pI2CHandle->pI2Cx->SR1 & (1 << 2))) /*BTF event*/
    {
        if (pI2CHandle->TxRxState == I2C_STATE_BUSY_TX && pI2CHandle->TxSize <= 0)
        {
            if (pI2CHandle->pI2Cx->SR1 & (1 << 7))
            {
                if (pI2CHandle->RepeatedStart == DISABLE)
                {
                    pI2CHandle->pI2Cx->CR1 |= (1 << 9);
                }
                pI2CHandle->pI2Cx->CR2 &= ~(1 << 9);
                pI2CHandle->pI2Cx->CR2 &= ~(1 << 10);
                pI2CHandle->pTxData = NULL;
                pI2CHandle->TxSize = 0;
                pI2CHandle->TxRxState = I2C_STATE_READY;
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_COMPLETE);
            }
            /*RX is not required to be closed*/
        }
    }

    if ((pI2CHandle->pI2Cx->CR2 & (1 << 9)) && (pI2CHandle->pI2Cx->SR1 & (1 << 4))) /*STOPF event*/
    {
        pI2CHandle->pI2Cx->CR1 |= 0x0000;
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    if ((pI2CHandle->pI2Cx->CR2 & (1 << 9)) && (pI2CHandle->pI2Cx->CR2 & (1 << 10)) && (pI2CHandle->pI2Cx->SR1 & (1 << 6)) && (pI2CHandle->pI2Cx->SR2 & (1 << 0))) /*RxNE event*/
    {
        if (pI2CHandle->TxRxState == I2C_STATE_BUSY_RX)
        {
            if (pI2CHandle->RxSize == 1)
            {
                *pI2CHandle->pRxData = pI2CHandle->pI2Cx->DR;
                --(pI2CHandle->RxSize);

            }
            else if (pI2CHandle->RxSize > 1)
            {
                /*check if last 2 bytes remaining*/
                if (pI2CHandle->RxSize == 2)
                {
                    I2C_ACKControl(pI2CHandle->pI2Cx, DISABLE);
                }

                *pI2CHandle->pRxData = pI2CHandle->pI2Cx->DR;
                --(pI2CHandle->RxSize);
                ++(pI2CHandle->pRxData);
            }
            else if (pI2CHandle->RxSize == 0)
            {
                if (pI2CHandle->RepeatedStart == DISABLE)
                {
                    pI2CHandle->pI2Cx->CR1 |= (1 << 9);
                }
                pI2CHandle->pI2Cx->CR2 &= ~(1 << 9);
                pI2CHandle->pI2Cx->CR2 &= ~(1 << 10);
                pI2CHandle->pRxData = NULL;
                pI2CHandle->RxSize = 0;
                pI2CHandle->RxSize_ = 0;
                pI2CHandle->TxRxState = I2C_STATE_READY;
                if (pI2CHandle->I2CConfig.ACKControl == I2C_ACK_EN)
                {
                    I2C_ACKControl(pI2CHandle->pI2Cx, ENABLE);
                }
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
            }
        }
    }

    if ((pI2CHandle->pI2Cx->CR2 & (1 << 9)) && (pI2CHandle->pI2Cx->CR2 & (1 << 10)) && (pI2CHandle->pI2Cx->SR1 & (1 << 7)) && (pI2CHandle->pI2Cx->SR2 & (1 << 0))) /*TxE event*/
    {
        if ((pI2CHandle->TxRxState == I2C_STATE_BUSY_TX) && (pI2CHandle->TxSize > 0))
        {
            pI2CHandle->pI2Cx->DR = *pI2CHandle->pTxData;
            --(pI2CHandle->TxSize);
            ++(pI2CHandle->pTxData);
        }
    }
}

/**
 *
 */
void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle)
{
    /*Bus error*/
    if ((pI2CHandle->pI2Cx->CR2 & (1 << 8)) && (pI2CHandle->pI2Cx->SR1 & (1 << 8)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << 8);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_BERR);
    }

    /*ARLO error*/
    if ((pI2CHandle->pI2Cx->CR2 & (1 << 8)) && (pI2CHandle->pI2Cx->SR1 & (1 << 9)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << 9);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_ARLO);
    }

    /*ACK fail error*/
    if ((pI2CHandle->pI2Cx->CR2 & (1 << 8)) && (pI2CHandle->pI2Cx->SR1 & (1 << 10)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << 10);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_AF);
    }

    /*OVR error*/
    if ((pI2CHandle->pI2Cx->CR2 & (1 << 8)) && (pI2CHandle->pI2Cx->SR1 & (1 << 11)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << 11);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_OVR);
    }

    /*Time-out error*/
    if ((pI2CHandle->pI2Cx->CR2 & (1 << 8)) && (pI2CHandle->pI2Cx->SR1 & (1 << 14)))
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << 14);
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ER_TIMEOUT);
    }


}
