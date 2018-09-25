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
 *     Driver Version:       0.1.1
 *     Last Revision Date:   09/25/2018
 *
 *     ================================================================================
 *
 *     Changelog:
 */

#include "esc.h"
#include "tim.h"

/*------------------------------------------------------------*/
//  (3) Modify the below macros to match the TIM peripheral
//      used for each ESC channel
/*------------------------------------------------------------*/
#define ESC1_TIM  TIM2
#define ESC2_TIM  TIM2
#define ESC3_TIM  TIM2
#define ESC4_TIM  TIM2

/*------------------------------------------------------------*/
//  (4) Modify the below macros to match the channel of the
//      TIM peripheral used for each ESC channel
/*------------------------------------------------------------*/
#define ESC1_CH   LL_TIM_CHANNEL_CH1
#define ESC2_CH   LL_TIM_CHANNEL_CH2
#define ESC3_CH   LL_TIM_CHANNEL_CH4
#define ESC4_CH   LL_TIM_CHANNEL_CH3

/*------------------------------------------------------------*/
//  (5) Modify the below macros to match the LL function name
//      of the LL function that modifies the CCR (Capture-
//      Compare Register) of the appropriate channel
/*------------------------------------------------------------*/

#define ESC1_SET_CCR_FUNC   (LL_TIM_OC_SetCompareCH1)
#define ESC2_SET_CCR_FUNC   (LL_TIM_OC_SetCompareCH2)
#define ESC3_SET_CCR_FUNC   (LL_TIM_OC_SetCompareCH4)
#define ESC4_SET_CCR_FUNC   (LL_TIM_OC_SetCompareCH3)


/*------------------------------------------------------------*/
//  Internal data structure which contains information about
//  an ESC channel
/*------------------------------------------------------------*/
typedef struct {
  TIM_TypeDef* tim;
  uint32_t timCh;
  void (*SetCompareChannelPeriod)(TIM_TypeDef *TIMx, uint32_t CompareValue);
} esc_t;

/*------------------------------------------------------------*/
//  Internal array that holds all ESC channels
/*------------------------------------------------------------*/
esc_t motors[4] = {
    {ESC1_TIM, ESC1_CH, ESC1_SET_CCR_FUNC},
    {ESC2_TIM, ESC2_CH, ESC2_SET_CCR_FUNC},
    {ESC3_TIM, ESC3_CH, ESC3_SET_CCR_FUNC},
    {ESC4_TIM, ESC4_CH, ESC4_SET_CCR_FUNC}
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
    LL_TIM_SetPrescaler(motors[i].tim, ESC_PSC_84MHz_1MHz);
    LL_TIM_SetAutoReload(motors[i].tim, ESC_TIM_CNT_END);

    ESC_Stop(i);
  }

  // Enable the Timer counters for each channel (No output
  // will be generated yet because channels are disabled)
  for (int i = 0; i < 4; i++) {
    LL_TIM_EnableCounter(motors[i].tim);
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
  LL_TIM_CC_EnableChannel(esc->tim, esc->timCh);

  // Invoke the appropriate "SetCompareChannelPeriod" function to set
  // the pulse width of the given ESC channel. Each channel has
  // a different function, thus the function pointers of the functions
  // were used to parameterize setting the pulse width on the fly
  (*esc->SetCompareChannelPeriod)(esc->tim, pwmPeriodMicros);
}
