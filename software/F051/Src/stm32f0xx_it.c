/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static __INLINE void BLDCMotorPrepareCommutation(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  timestamp++;
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */

  if( LL_TIM_IsActiveFlag_COM(TIM1) ){
    LL_TIM_ClearFlag_COM(TIM1);
  }

  if(LL_TIM_IsActiveFlag_BRK(TIM1)){
    LL_TIM_ClearFlag_BRK(TIM1);
  }
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if(LL_TIM_IsActiveFlag_CC1(TIM2)){

    LL_TIM_ClearFlag_CC1(TIM2);

  }else if(LL_TIM_IsActiveFlag_CC2(TIM2)){

    LL_TIM_ClearFlag_CC2(TIM2);

    BLDCMotorPrepareCommutation();

  }else{;}


  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

static __INLINE void BLDCMotorPrepareCommutation(void)
{

  static uint16_t lasthallpos = 0;
  uint16_t hallpos = ((LL_GPIO_ReadInputPort(GPIOA) & 0x0700) >> 8);

  if (hallpos == lasthallpos) return;

  lasthallpos = hallpos;

  uint8_t BH1 = BLDC_BRIDGE_STATE_VORWARD[hallpos][0];
  uint8_t BL1 = BLDC_BRIDGE_STATE_VORWARD[hallpos][1];

  uint8_t BH2 = BLDC_BRIDGE_STATE_VORWARD[hallpos][2];
  uint8_t BL2 = BLDC_BRIDGE_STATE_VORWARD[hallpos][3];

  uint8_t BH3 = BLDC_BRIDGE_STATE_VORWARD[hallpos][4];
  uint8_t BL3 = BLDC_BRIDGE_STATE_VORWARD[hallpos][5];


//  if (BH1) {
//
//    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
//    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
//    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
//
//  } else {
//
//    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
//
//    if (BL1){
//
//      TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
//      TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
//
//    } else {
//
//      TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
//    }
//
//  }



}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
