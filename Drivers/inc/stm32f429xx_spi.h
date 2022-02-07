/*
 * stm32f429xx_spi.h
 *
 *  Created on: 03 Feb 2022
 *      Author:
 */

#ifndef INC_STM32F429XX_SPI_H_
#define INC_STM32F429XX_SPI_H_

#include "stm32f429xx.h"

/**
 * MACROS for configuring SPI_Config_t struct
 */
#define SPI_MODE_SLAVE                  0
#define SPI_MODE_MASTER                 1

#define SPI_BUS_FULL_DUPLEX             0
#define SPI_BUS_HALF_DUPLEX             1
#define SPI_BUS_TXONLY                  2
#define SPI_BUS_RXONLY                  3

#define SPI_DFF_8_BIT                   0
#define SPI_DFF_16_BIT                  1

#define SPI_CPHA_LOW                    0
#define SPI_CPHA_HIGH                   1

#define SPI_CPOL_LOW                    0
#define SPI_CPOL_HIGH                   1

#define SPI_SSM_DI                      0
#define SPI_SSM_EN                      1

#define SPI_SSI_DI                      0
#define SPI_SSI_EN                      1

#define SPI_SPEED_CLK_DIV2              0
#define SPI_SPEED_CLK_DIV4              2
#define SPI_SPEED_CLK_DIV8              3
#define SPI_SPEED_CLK_DIV16             4
#define SPI_SPEED_CLK_DIV32             5
#define SPI_SPEED_CLK_DIV64             6
#define SPI_SPEED_CLK_DIV128            7
#define SPI_SPEED_CLK_DIV256            8

#define SPI_READY                       0
#define SPI_RX_BUSY                     1
#define SPI_TX_BUSY                     2

#define SPI_EVENT_TX_COMPLETE           1
#define SPI_EVENT_RX_COMPLETE           2
#define SPI_EVENT_OVERRUN_ERROR         3



typedef struct
{
    uint32_t Device_Mode;
    uint32_t Bus_Config;
    uint32_t DFF;
    uint32_t CPHA;
    uint32_t CPOL;
    uint32_t SSM;
    uint32_t SSI;
    uint32_t CLK_Speed;
} SPI_Config_t;

typedef struct
{
    SPI_Registers_t *pSPIx;
    SPI_Config_t SPI_Config;
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint32_t TxSize;
    uint32_t RxSize;
    uint8_t TxState;
    uint8_t RxState;
} SPI_Handle_t;

/*******************API Function Prototypes***********************************/
void SPI_Init(SPI_Handle_t *pSPIHandle);/****/
void SPI_Reset(SPI_Registers_t *pSPIx);/****/
uint8_t SPI_Control(SPI_Registers_t *pSPIx, uint8_t EN);


void SPI_SendData(SPI_Registers_t *pSPIx, uint8_t *pTxData, uint32_t size);
void SPI_ReceiveData(SPI_Registers_t *pSPIx, uint8_t *pRxData, uint32_t size);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxData, uint32_t size);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxData, uint32_t size);

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en);/****/
//static void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);/****/
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);/****/

void SPI_ClearOVRFlag(SPI_Registers_t *pSPIx);
void SPI_CloseTx(SPI_Handle_t *pSPIHandle);
void SPI_CloseRx(SPI_Handle_t *pSPIHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event);


#endif /* INC_STM32F429XX_SPI_H_ */
