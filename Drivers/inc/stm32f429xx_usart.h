/*
 * stm32f429xx_usart.h
 *
 *  Created on: 16 Feb 2022
 *      Author:
 */

#ifndef INC_STM32F429XX_USART_H_
#define INC_STM32F429XX_USART_H_

#include "stm32f429xx.h"


#define USART_MODE_ONLY_TX                  0
#define USART_MODE_ONLY_RX                  1
#define USART_MODE_TXRX                     2

#define USART_STD_BAUD_1200                 1200
#define USART_STD_BAUD_2400                 2400
#define USART_STD_BAUD_9600                 9600
#define USART_STD_BAUD_19200                19200
#define USART_STD_BAUD_38400                38400
#define USART_STD_BAUD_57600                57600
#define USART_STD_BAUD_115200               115200
#define USART_STD_BAUD_230400               230400
#define USART_STD_BAUD_460800               460800
#define USART_STD_BAUD_921600               921600
#define USART_STD_BAUD_2M                   2000000
#define SUART_STD_BAUD_3M                   3000000

#define USART_PARITY_DISABLE                0
#define USART_PARITY_EN_EVEN                1
#define USART_PARITY_EN_ODD                 2

#define USART_WORDLENGTH_8BITS              0
#define USART_WORDLENGTH_9BITS              1

#define USART_STOPBITS_1                    0
#define USART_STOPBITS_0_5                  1
#define USART_STOPBITS_2                    2
#define USART_STOPBITS_1_5                  3

#define USART_HW_FLOW_CTRL_NONE             0
#define USART_HW_FLOW_CTRL_CTS              1
#define USART_HW_FLOW_CTRL_RTS              2
#define USART_HW_FLOW_CTRL_CTS_RTS          3

typedef struct
{
    uint8_t Mode;
    uint16_t BaudRate;
    uint8_t StopBits;
    uint8_t WordLength;
    uint8_t ParityBit;
    uint8_t HWFlowCtrl;
} USART_Config_t;

typedef struct
{
    USART_Registers_t *pUSARTx;
    USART_Config_t USARTConfig;

} USART_Handle_t;

/*
 * API
 */
void USART_Init(USART_Handle_t *pUSARTHandle);/****/
void USART_Reset(USART_Registers_t *pUSARTx);/****/
uint8_t USART_Control(USART_Registers_t *pUSART, uint8_t EN);

void USART_SendData(USART_Registers_t *pUSARTx,uint8_t *pTxData, uint32_t size);
void USART_ReceiveData(USART_Registers_t *pUSARTx, uint8_t *pRxData, uint32_t size);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxData, uint32_t size);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxData, uint32_t size);

void USART_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en);/****/
void USART_IRQHandler(USART_Handle_t *pUSARTHandle);

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event);

#endif /* INC_STM32F429XX_USART_H_ */
