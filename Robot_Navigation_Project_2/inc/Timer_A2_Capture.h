/**
 * @file Timer_A2_Capture.h
 * @brief Header file for the Timer_A2_Capture driver.
 *
 * This file contains the function definitions for configuring Timer A2 in Capture mode.
 * Timer_A2_Capture is used to detect the edges of a square wave signal.
 *
 * The following pin is used as an input capture pin:
 *  - P5.6 (PM_TA2.1)
 *
 * @author Aaron Nanas
 */

#ifndef INC_TIMER_A2_CAPTURE_H_
#define INC_TIMER_A2_CAPTURE_H_

#include <stdint.h>
#include "msp.h"

// Declare pointer to the user-defined function
void (*Timer_A2_Capture_Task)(uint16_t time);

/**
 * @brief Initialize Timer A2 for Capture mode with interrupts enabled.
 *
 * This function initializes Timer A3 in Capture mode to request interrupts
 * on the rising edge of both P5.6 (PM_TA2.1). The associated interrupt
 * service routine acknowledges the interrupt, clears it, and then
 * calls a user-defined function.
 *
 * @param task:     Pointer to a user-defined function that is called when the rising edge of
 *                  P5.6 is detected. The parameter is a 16-bit up-counter value that
 *                  indicates when the rising edge of P5.6 occurred (in units of 83.3 ns)
 *
 * @return None
 */
void Timer_A2_Capture_Init(void(*task)(uint16_t time));

#endif /* INC_TIMER_A2_CAPTURE_H_ */
