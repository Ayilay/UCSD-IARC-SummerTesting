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
 *     Driver Version:       0.2.0
 *     Last Revision Date:   09/25/2018
 *
 *     ================================================================================
 *
 *     Changelog:
 *      0.2.0:    Modified to use HAL instead of LL
 */

#include "esc.h"
#include "tim.h"

/*------------------------------------------------------------*/
//  (3) Modify the below macros to match the TIM peripheral
//      used for each ESC channel
/*------------------------------------------------------------*/
#define ESC1_TIMHANDLE  &htim2
#define ESC2_TIMHANDLE  &htim2
#define ESC3_TIMHANDLE  &htim2
#define ESC4_TIMHANDLE  &htim2

/*------------------------------------------------------------*/
//  (4) Modify the below macros to match the channel of the
//      TIM peripheral used for each ESC channel
/*------------------------------------------------------------*/
#define ESC1_CH   TIM_CHANNEL_1
#define ESC2_CH   TIM_CHANNEL_2
#define ESC3_CH   TIM_CHANNEL_3
#define ESC4_CH   TIM_CHANNEL_4

/*------------------------------------------------------------*/
//  Internal data structure which contains information about
//  an ESC channel
/*------------------------------------------------------------*/
typedef struct {
  TIM_HandleTypeDef* timHandle;
  uint32_t timCh;
} esc_t;

/*------------------------------------------------------------*/
//  Internal array that holds all ESC channels
/*------------------------------------------------------------*/
esc_t motors[4] = {
    {ESC1_TIMHANDLE, ESC1_CH},
    {ESC2_TIMHANDLE, ESC2_CH},
    {ESC3_TIMHANDLE, ESC3_CH},
    {ESC4_TIMHANDLE, ESC4_CH}
};

/*------------------------------------------------------------*/
//  Interface Functions
/*------------------------------------------------------------*/

/**
 *  Initialize the ESC channels with appropriate prescaler and
 *  autoreload values, and disable their outputs.
 */
void ESC_Init() {

  // Configure the timer of each ESC with the appropriate prescaler
  // and counter period, then disable the output channel
  for (int i = 0; i < 4; i++) {
    __HAL_TIM_SET_PRESCALER(motors[i].timHandle, ESC_PSC_84MHz_1MHz);
    __HAL_TIM_SET_AUTORELOAD(motors[i].timHandle, ESC_TIM_CNT_END);

    ESC_Stop(i);
  }

  // Enable the Timer counters for each channel (No output
  // will be generated yet because channels are disabled)
  for (int i = 0; i < 4; i++) {
    __HAL_TIM_ENABLE(motors[i].timHandle);
  }
}

/**
 *  Disables an ESC channel from outputting a pulse.
 *
 *  After this function is called, the esc chosen will output
 *  a 0V output signal, effectively disabling the ESC.
 *
 *  escInd:  Identifies which ESC to stop. Options for this
 *           argument are the following:
 *           - ESC1
 *           - ESC2
 *           - ESC3
 *           - ESC4
 */
void ESC_Stop(uint32_t escInd) {
  ESC_SetPWMPulseWidth(escInd, 0);
}

/**
 *  Sets the speed of an ESC channel.
 *
 *  After this function is called, the esc chosen will output
 *  a 20ms period PWM signal whose positive pulse varies from
 *  1 ms (0% power) to 2 ms (100% power).
 *
 *  escInd:  Identifies which ESC to stop. Options for this
 *           argument are the following:
 *           - ESC1
 *           - ESC2
 *           - ESC3
 *           - ESC4
 *  speed:  A floating point value representing the percentage
 *          of power to give to the esc, in the range [0.0, 1.0]
 */
void ESC_SetSpeed(uint32_t escInd, float speed) {
  if (speed < 0)
    return;

  // When speed == 0.0, period == 1 ms
  // When speed == 1.0, period == 2 ms
  // Anything in between scales linearly
  uint32_t period = speed * (ESC_TIM_PERIOD_2MS - ESC_TIM_PERIOD_1MS)
                      + ESC_TIM_PERIOD_1MS;
  ESC_SetPWMPulseWidth(escInd, period);
}

/**
 *  Sets the positive pulse width for the PWM signal
 *  going to an ESC channel. Use this for low level control only,
 *  otherwise prefer to use the ESC_SetSpeed() function.
 *
 *  After this function is called, the esc chosen will output
 *  a 20ms period PWM signal whose positive pulse is
 *  'pwmPeriodMicros' microseconds long.
 *
 *  Note that if 'pwmPeriodMicros' is longer than 20 ms, the
 *  output will be always high.
 *
 *  escInd:  Identifies which ESC to stop. Options for this
 *           argument are the following:
 *           - ESC1
 *           - ESC2
 *           - ESC3
 *           - ESC4
 *  pwmPeriodMicros: The period of the positive pulse of the PWM signal
 *                   in microseconds
 */
void ESC_SetPWMPulseWidth(uint32_t escInd, uint32_t pwmPeriodMicros) {
  esc_t* esc = &motors[escInd];

  // Enable the Capture Compare channel in case it was disabled
  // Bypass HAL for minimum overhead
  TIM_CCxChannelCmd(esc->timHandle->Instance, esc->timCh, TIM_CCx_ENABLE);

  // Set the pulse width of the ESC channel
  __HAL_TIM_SET_COMPARE(esc->timHandle, esc->timCh, pwmPeriodMicros);
}
