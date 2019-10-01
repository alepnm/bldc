#ifndef NUCLEO_H_INCLUDED
#define NUCLEO_H_INCLUDED

#include "stm32f3xx_ll_gpio.h"

#define LED2_ON()       LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13);
#define LED2_OFF()      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
#define LED2_TOGGLE()   LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_13);

#endif /* NUCLEO_H_INCLUDED */
