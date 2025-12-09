/**
 * @file GPIO_Utilities.h
 *
 * @brief   a function purely meant to appease the Power (ULP) Advice
 *
 */

#ifndef __INC_GPIO_UTILITIES_H__
#define __INC_GPIO_UTILITIES_H__

#include "msp.h"

/**
 * @brief Initialize unused GPIO ports to reduce power consumption
 *
 * This function configures all unused GPIO pins as outputs with low state to
 *  eliminate floating inputs and reduce current consumption. This satisfies
 *  the MSP432 ULP (Ultra-Low Power) Advisor requirements.
 *
 * @details Initializes ALL ports to GPIO mode with outputs low by default.
 *      It also appears that these registers are assigned directly instead of
 *      friendly-modified. This suffices because this will be the default
 *      setting until otherwise replaced by bitmasked values by their
 *      respective drivers.
 */
void Init_Unused_Ports(void);


/**
 * @brief set up all receive and send UART interrupts
 */
void Set_All_Interrupts_1(void);

/**
 * @brief set up all receive and send UART interrupts
 * @details difference 1 & 2 is that it holds and releases the whole set of 
 *  EUSCI_Ax blocks while performing changes. In my experience, these steps are
 *  not necessary
 */
void Set_All_Interrupts_2(void);


#endif /* __INC_GPIO_UTILITIES_H__ */
