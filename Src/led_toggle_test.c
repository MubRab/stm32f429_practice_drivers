#include "stm32f429xx.h"
#include "stm32f429xx_gpio.h"

#define delay()                 for(int i = 0; i < 500000; ++i)

int main(void)
{
    GPIO_Handle_t *pGPIOHandleLED = malloc(sizeof(GPIO_Handle_t));
    pGPIOHandleLED->pGPIOx = GPIOG;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_number = GPIO_PIN_13;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_mode = GPIO_PIN_MODE_OUT;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_out_type = GPIO_PIN_OUT_PP;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_speed = GPIO_PIN_SPEED_HIGH;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_pupd = GPIO_PIN_NOPUPD;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_alt_func = 0x0;

    GPIO_Init(pGPIOHandleLED);

    while (1)
    {
        GPIO_TogglePin(GPIOG, GPIO_PIN_13);
        delay();
    }

    return 0;
}
