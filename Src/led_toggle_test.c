#include <stdlib.h>
#include "stm32f429xx.h"
#include "stm32f429xx_gpio.h"

#define delay()                 for(int i = 0; i < 500000/2; ++i)

int main(void)
{
    GPIO_Handle_t *pGPIOHandleLED = (GPIO_Handle_t*) malloc(sizeof(GPIO_Handle_t));
    pGPIOHandleLED->pGPIOx = GPIOG;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_number = GPIO_PIN_13;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_mode = GPIO_PIN_MODE_OUT;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_out_type = GPIO_PIN_OUT_PP;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_speed = GPIO_PIN_SPEED_HIGH;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_pupd = GPIO_PIN_NOPUPD;
    pGPIOHandleLED->pGPIOx_Pin_Config->pin_alt_func = 0x0;

    GPIO_Init(pGPIOHandleLED);

    GPIO_Handle_t *pGPIOHandleBtn = (GPIO_Handle_t*) malloc(sizeof(GPIO_Handle_t));
    pGPIOHandleBtn->pGPIOx = GPIOA;
    pGPIOHandleBtn->pGPIOx_Pin_Config->pin_number = GPIO_PIN_0;
    pGPIOHandleBtn->pGPIOx_Pin_Config->pin_mode = GPIO_PIN_MODE_IN;
    pGPIOHandleBtn->pGPIOx_Pin_Config->pin_out_type = 0x0;
    pGPIOHandleBtn->pGPIOx_Pin_Config->pin_speed = 0x0;
    pGPIOHandleBtn->pGPIOx_Pin_Config->pin_pupd = 0x0;
    pGPIOHandleBtn->pGPIOx_Pin_Config->pin_alt_func = 0x0;

    GPIO_Init(pGPIOHandleBtn);


    while (1)
    {
        if (GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)/**In this case, the button will be high when the button is pressed**/
        {
            GPIO_TogglePin(GPIOG, GPIO_PIN_13);
            delay();/**Delay for software de-bouncing for button**/
        }
    }

//    free(pGPIOHandleLED);
//    free(pGPIOHandleBtn);

    return 0;
}
