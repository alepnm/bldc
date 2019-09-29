#include "main.h"
#include "bldc.h"


uint8_t BLDC_SystemInit(void){

    /* SysTick start */
    SysTick_Config(SystemCoreClock/1000);


    /* TIM1 - Output timer */
    LL_TIM_EnableIT_COM(TIM1);
    LL_TIM_EnableIT_BRK(TIM1);

    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);

    /* TIM2 - interfacing timer */
    LL_TIM_EnableIT_COM(TIM2);
    LL_TIM_EnableCounter(TIM2);

    /* UART init */
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_DisableIT_TC(USART1);
    LL_USART_DisableIT_TXE(USART1);

    return 1;
}


