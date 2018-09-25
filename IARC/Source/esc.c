/**
 *     esc.c:  Defines functionality for using Electronic Speed Controllers (ESCs)
 *             for the quadcopter.
 *
 *     This driver relies on STM32 HAL for functionality. Support for other STM32 libraries
 *     (i.e. LL or SPL) is unimplemented.
 *
 *     HOW TO USE THIS DRIVER:
 *     See esc.h
 *
 *     ================================================================================
 *
 *     Author:               Georges Troulis
 *     Email:                gtroulis@ucsd.edu
 *     Driver Version:       0.1.0
 *     Last Revision Date:   09/24/2018
 *
 *     ================================================================================
 *
 *     Changelog:
 */

#include "esc.h"
#include "tim.h"

typedef struct {
  TIM_TypeDef* tim;
  uint32_t timCh;
  void (*SetCompareChannelPeriod)(TIM_TypeDef *TIMx, uint32_t CompareValue);
} esc_t;

#define ESC1_TIM  TIM2
#define ESC2_TIM  TIM2
#define ESC3_TIM  TIM2
#define ESC4_TIM  TIM2

#define ESC1_CH   LL_TIM_CHANNEL_CH1
#define ESC2_CH   LL_TIM_CHANNEL_CH2
#define ESC3_CH   LL_TIM_CHANNEL_CH4
#define ESC4_CH   LL_TIM_CHANNEL_CH3

// Function Pointers to set the CCR (Capture Compare Reg) value of each channel
// Done using function pointers to parameterize setting the TIM channel CCR
// since it is the function name that differs per channel
#define ESC1_SET_CCR_FUNC   (LL_TIM_OC_SetCompareCH1)
#define ESC2_SET_CCR_FUNC   (LL_TIM_OC_SetCompareCH2)
#define ESC3_SET_CCR_FUNC   (LL_TIM_OC_SetCompareCH4)
#define ESC4_SET_CCR_FUNC   (LL_TIM_OC_SetCompareCH3)


esc_t motors[4] = {
    {ESC1_TIM, ESC1_CH, ESC1_SET_CCR_FUNC},
    {ESC2_TIM, ESC2_CH, ESC2_SET_CCR_FUNC},
    {ESC3_TIM, ESC3_CH, ESC3_SET_CCR_FUNC},
    {ESC4_TIM, ESC4_CH, ESC4_SET_CCR_FUNC}
};

void ESC_Init() {

  for (int i = 0; i < 4; i++) {
    ESC_Stop(i);
  }

  for (int i = 0; i < 4; i++) {
    LL_TIM_EnableCounter(motors[i].tim);
  }
}

void ESC_Stop(uint32_t escInd) {
  esc_t* esc = &motors[escInd];
  ESC_SetSpeedRaw(escInd, 0);
  //LL_TIM_CC_DisableChannel(esc->tim, esc->timCh);
}

void ESC_SetSpeed(uint32_t escInd, float speed) {
  if (speed < 0)
    return;

  // When speed == 0.0, period == 1 ms
  // When speed == 1.0, period == 2 ms
  // Anything in between scales linearly
  uint32_t period = speed * (ESC_TIM_PERIOD_2MS - ESC_TIM_PERIOD_1MS)
                      + ESC_TIM_PERIOD_1MS;
  ESC_SetSpeedRaw(escInd, period);
}

void ESC_SetSpeedRaw(uint32_t escInd, uint32_t pwmPeriod) {
  esc_t* esc = &motors[escInd];
  LL_TIM_CC_EnableChannel(esc->tim, esc->timCh);
  (*esc->SetCompareChannelPeriod)(esc->tim, pwmPeriod);
}
