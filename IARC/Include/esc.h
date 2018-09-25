/**
 *     esc.h:  Defines a software interface for using the Electronic Speed Controllers (ESC)
 *
 *     This driver relies on STM32 HAL for functionality. Support for other STM32 libraries
 *     (i.e. LL or SPL) is unimplemented.
 *
 *     HOW TO USE THIS DRIVER:
 *     (1) In STM32CubeMX, configure HAL to have 4 PWM channels from Timers (ESCs don't
 *         need to share the same timer). Configure each PWM channel as follows:
 *           * ChannelX: PWM Generation CHx
 *           * Mode: PWM mode 1
 *           * CH Polarity: High
 *     (2) In STM32CubeMX, ensure all Timers used have the same frequency (APB1 and APB2
 *          are the same frequency), and then modify the prescaler below in esc.h
 *          appropriately to scale that frequency to 1 MHz
 *     (3) In esc.c, modify the ESCx_TIM macros for each ESC to match the Timer
 *         used for each ESC channel
 *     (4) In esc.c, modify the ESCx_CH macros for each ESC to match the LL_TIM channel
 *         used for each ESC (independent of timer used)
 *     (5) Modify the ESCx_SET_CCR_FUNC macros to contain the pointer to the function
 *         that is used to change the CCR (Capture/Compare Register) of each ESC channel.
 *
 *         ex: if ESC3 uses Channel 2 of TIMx (timer is irrelevant), modify
 *             ESC3_SET_CCR_FUNC to be (LL_TIM_OC_SetCompareCH2)
 *
 *    ================================================================================
 *
 *     Author:               Georges Troulis
 *     Email:                gtroulis@ucsd.edu
 *     Driver Version:       0.1.1
 *     Last Revision Date:   09/25/2018
 *
 *    ================================================================================
 *
 *    Changelog:
 */

#ifndef INCLUDE_ESC_H
#define INCLUDE_ESC_H

#include "tim.h"

/*------------------------------------------------------------*/
//  ESC Timer config options, used to set the PWM period
//  and duty cycle
/*------------------------------------------------------------*/
#define ESC_TIM_CNT_END      20000
#define ESC_TIM_PERIOD_1MS   1000
#define ESC_TIM_PERIOD_2MS   2000

// (2) Modify this prescaler to scale the APB Timer Clock (see
//     datasheet for which APB is responsible for timers) down
//     to 1 MHz
#define ESC_PSC_84MHz_1MHz   83

/*------------------------------------------------------------*/
//  ESC Handler Macros. Use these to access desired ESCs
//  when calling interface functions
/*------------------------------------------------------------*/
#define ESC1 0
#define ESC2 1
#define ESC3 2
#define ESC4 3

void ESC_Init();
void ESC_Stop(uint32_t escVect);
void ESC_SetSpeed(uint32_t escVect, float speed);
void ESC_SetPWMPulseWidth(uint32_t escVect, uint32_t pwmPeriod);

#endif
