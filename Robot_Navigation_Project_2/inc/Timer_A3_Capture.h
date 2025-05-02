/**
 * @file Timer_A3_Capture.h
 * @brief Header file for the Timer_A3_Capture driver.
 *
 * This file contains the function definitions for configuring Timer A3 in Capture mode.
 * with interrupts enabled. External signals are connected to P10.4 and P10.5.
 * The interrupts are triggered when the rising edges of P10.4 and P10.5 are detected.
 *
 * Timer A3 is used as a base driver for the Tachometer driver.
 *
 * @author Aaron Nanas
 */

#ifndef INC_TIMER_A3_CAPTURE_H_
#define INC_TIMER_A3_CAPTURE_H_

#include <stdint.h>
#include "msp.h"

// Declare pointer to the user-defined function
void (*Timer_A3_Capture_Task_0)(uint16_t time);

// Declare pointer to the user-defined function
void (*Timer_A3_Capture_Task_1)(uint16_t time);

/**
 * @brief Initialize Timer A3 for Capture mode with interrupts enabled.
 *
 * This function initializes Timer A3 in Capture mode to request interrupts
 * on the rising edge of both P10.4 (PM_TA3.0, Right Encoder A) and P10.5 (PM_TA3.1, Left Encoder A).
 * The associated interrupt service routine acknowledges the interrupt, clears it,
 * and then calls a user-defined function.
 *
 * @param task0:    Pointer to a user-function that is called when the rising edge of
 *                  P10.4 is detected. The parameter is a 16-bit up-counter value that
 *                  indicates when the rising edge of P10.4 occurred (in units of 83.3 ns)
 *
 * @param task1:    Pointer to a user-function that is called when the rising edge of
 *                  P10.5 is detected. The parameter is a 16-bit up-counter value that
 *                  indicates when the rising edge of P10.5 occurred (in units of 83.3 ns)
 *
 * @return None
 */
void Timer_A3_Capture_Init(void(*task0)(uint16_t time), void(*task1)(uint16_t time));

#endif /* INC_TIMER_A3_CAPTURE_H_ */
