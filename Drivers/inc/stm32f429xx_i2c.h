/*
 * stm32f429xx_i2c.h
 *
 *  Created on: 08 Feb 2022
 *      Author:
 */

#ifndef INC_STM32F429XX_I2C_H_
#define INC_STM32F429XX_I2C_H_

#include "stm32f429xx.h"

/*
 * Macros
 */
#define I2C_MODE_SM                         0
#define I2C_MODE_FM                         1

#define I2C_ACK_EN                          1
#define I2C_ACK_DI                          0

#define I2C_FM_DUTY_2                       0
#define I2C_FM_DUTY_16_9                    1

#define I2C_SCL_FREQ_SM                     100000
#define I2C_SCL_FREQ_FM                     400000

#define I2C_STATE_READY                     0
#define I2C_STATE_BUSY_RX                   1
#define I2C_STATE_BUSY_TX                   2

#define I2C_EV_TX_COMPLETE                  0
#define I2C_EV_RX_COMPLETE                  1
#define I2C_EV_STOP                         2

typedef struct
{
    uint32_t SCLSpeed;
    uint8_t DeviceAddress;
    uint8_t ACKControl;
    uint16_t FMDutyCycle;
} I2C_Config_t;

typedef struct
{
    I2C_Registers_t *pI2Cx;
    I2C_Config_t I2CConfig;
    uint8_t *pTxData;
    uint8_t *pRxData;
    uint32_t TxSize;
    uint32_t RxSize;
    uint8_t TxRxState;  /*Only one state, since I2C is half-duplex only*/
    uint8_t SlaveAddress;
    uint32_t RxSize_;
    uint8_t RepeatedStart;
} I2C_Handle_t;

/**
 * API Prototypes
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);/****/
void I2C_Reset(I2C_Registers_t *pI2Cx);/****/
uint8_t I2C_Control(I2C_Registers_t *pI2Cx, uint8_t EN);
void I2C_ACKControl(I2C_Registers_t *pI2Cx, uint8_t EN);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t size, uint8_t slaveAddr, uint8_t repeatedStart);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t size, uint8_t slaveAddr, uint8_t repeatedStart);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t size, uint8_t slaveAddr, uint8_t repeatedStart);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pData, uint32_t size, uint8_t slaveAddr, uint8_t repeatedStart);

void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t en);/****/
void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event);

#endif /* INC_STM32F429XX_I2C_H_ */
