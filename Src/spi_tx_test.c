/*
 * spi_tx_test.c
 *
 *  Created on: 04 Feb 2022
 *      Author:
 */
#include <stdlib.h>
#include "stm32f429xx.h"
#include "stm32f429xx_gpio.h"
#include "stm32f429xx_spi.h"

int main(void)
{
    /**
     * Setting up GPIOs for SPI transmitting only
     */
    GPIO_Handle_t *spi_pins = (GPIO_Handle_t*) malloc(sizeof(GPIO_Handle_t));
    spi_pins->pGPIOx = GPIOB;
    spi_pins->pGPIOx_Pin_Config->pin_mode = GPIO_PIN_MODE_ALT_FUNC;
    spi_pins->pGPIOx_Pin_Config->pin_speed = GPIO_PIN_SPEED_HIGH;
    spi_pins->pGPIOx_Pin_Config->pin_out_type = GPIO_PIN_OUT_PP;
    spi_pins->pGPIOx_Pin_Config->pin_pupd = GPIO_PIN_NOPUPD;
    spi_pins->pGPIOx_Pin_Config->pin_alt_func = GPIO_PIN_AF5;

    /**SCLK**/
    spi_pins->pGPIOx_Pin_Config->pin_number = GPIO_PIN_10;
    GPIO_Init(spi_pins);

    /**MOSI**/
    spi_pins->pGPIOx_Pin_Config->pin_number = GPIO_PIN_15;
    GPIO_Init(spi_pins);

    free(spi_pins);

    /**
     * Setting up SPI2
     */
    SPI_Handle_t *spi2 = (SPI_Handle_t*) malloc(sizeof(SPI_Handle_t));
    spi2->pSPIx = SPI2;
    spi2->SPI_Config.Device_Mode = SPI_MODE_MASTER;
    spi2->SPI_Config.Bus_Config = SPI_BUS_FULL_DUPLEX;
    spi2->SPI_Config.DFF = SPI_DFF_8_BIT;
    spi2->SPI_Config.CPHA = SPI_CPHA_LOW;
    spi2->SPI_Config.CPOL = SPI_CPOL_LOW;
    spi2->SPI_Config.SSM = SPI_SSM_EN;
    spi2->SPI_Config.SSI = SPI_SSI_EN;
    spi2->SPI_Config.CLK_Speed = SPI_SPEED_CLK_DIV2;

    SPI_Init(spi2);
    free(spi2);

    /**
     * Sending data
     */
    char data[] = "Hello World";
    uint32_t len = sizeof(data)/sizeof(data[0]);
    /**first enable the peripheral**/
    SPI_Control(SPI2, ENABLE);
    SPI_SendData(SPI2, (uint8_t*) data, len);
    SPI_Control(SPI2, DISABLE);
    while(1);



    return 1;
}

