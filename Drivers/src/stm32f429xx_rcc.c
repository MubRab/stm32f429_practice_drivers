/*
 * stm32f429xx_rcc.c
 *
 *  Created on: 17 Feb 2022
 *      Author:
 */
#include "stm32f429xx_rcc.h"

uint32_t GetPCLKAPB1(void)
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
    idx = ((RCC->CFGR >> 10) & 0x7);
    if (idx >= 4)
    {
        clk_speed /= apb1_prescaler[idx-4];
    }

    return clk_speed;
}

uint32_t GetPCLKAPB2(void)
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

    uint16_t apb2_prescaler[4] = {2, 4, 8, 16};
    idx = ((RCC->CFGR >> 13) & 0x7);
    if (idx >= 4)
    {
        clk_speed /= apb2_prescaler[idx-4];
    }

    return clk_speed;
}
