/**
 *     esc.h:  Defines a software interface for using the Electronic Speed Controllers (ESC)
 *
 *     This driver relies on STM32 HAL for functionality. Support for other STM32 libraries
 *     (i.e. LL or SPL) is unimplemented.
 *
 *     HOW TO USE THIS DRIVER:
 *     (1) Configure HAL to use at least one timer such that 4 PWM channels exist.
 *         Ensure the following settings for the timer(s) used:
 *         * TODO
 *     (2)
 *
 *    ================================================================================
 *
 *     Author:               Georges Troulis
 *     Email:                gtroulis@ucsd.edu
 *     Driver Version:       0.1.0
 *     Last Revision Date:   09/24/2018
 *
 *    ================================================================================
 *
 *    Changelog:
 */

#ifndef INCLUDE_ESC_H
#define INCLUDE_ESC_H

#include "tim.h"

#define ESC1 0
#define ESC2 1
#define ESC3 2
#define ESC4 3

void ESC_Init();
void ESC_Stop(uint32_t escVect);
void ESC_SetSpeed(uint32_t escVect, float speed);
void ESC_SetSpeedRaw(uint32_t escVect, uint32_t pwmPeriod);

#endif
