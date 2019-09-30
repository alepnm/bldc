#ifndef NUCLEO_H_INCLUDED
#define NUCLEO_H_INCLUDED

#include "usart.h"

#define LED2_ON()       LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13, GPIO_PIN_RESET);
#define LED2_OFF()      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13, GPIO_PIN_SET);
#define LED2_TOGGLE()   LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_13);

#endif /* NUCLEO_H_INCLUDED */
