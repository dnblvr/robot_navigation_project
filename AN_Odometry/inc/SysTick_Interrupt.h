/**
 * @file SysTick_Interrupt.h
 * @brief Header file for the SysTick_Interrupt driver.
 *
 * This file contains the function definitions for the SysTick_Interrupt driver.
 * It uses the SysTick timer to perform interrupt requests at the specified period.
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_SYSTICK_INTERRUPT_H_
#define INC_SYSTICK_INTERRUPT_H_

#include <stdint.h>
#include "msp.h"

// The toggle rate for SysTick_Interrupt in ms
#define SYSTICK_INT_TOGGLE_RATE_MS 500

// The number of clock cycles between SysTick interrupts
// The frequency of the clock used is 48 MHz, so each
// clock cycle would last about 20.83 ns
// Therefore, using a value of 48,000 would result in the
// time interval for a SysTick interrupt of 1 ms
#define SYSTICK_INT_NUM_CLK_CYCLES 48000

// The priority level of the SysTick interrupt
#define SYSTICK_INT_PRIORITY 2

/**
 * @brief Initializes the SysTick timer with periodic interrupts.
 *
 * This function configures the SysTick timer to generate periodic interrupts. The interrupts will occur at a specified
 * interval determined by the 'clock_cycles' parameter, which is provided in terms of the bus clock period. The function
 * also allows setting the priority level of the SysTick interrupt handler using the 'priority' parameter.
 *
 * @param clock_cycles The number of clock cycles between SysTick interrupts.
 *                     The valid range for 'clock_cycles' is from 1 to 2^24 - 1.
 *                     The actual time between interrupts will depend on the system clock frequency and the 'clock_cycles' value.
 * @param priority     The priority level of the SysTick interrupt. Valid values range from 0 (highest priority) to 7 (lowest priority).
 *
 * @note Before calling this function, ensure that the SysTick interrupt handler (SysTick_Handler) is defined and handles the interrupts.
 *       The interrupt handler should be implemented separately to perform the desired operations on each SysTick interrupt.
 *
 * @note The 'clock_cycles' value should be chosen carefully to prevent the SysTick interrupt from occurring too frequently,
 *       causing an overload on the system. Additionally, setting a very small 'clock_cycles' value may result in frequent interrupts,
 *       which could lead to timing issues and affect the system's performance.
 *
 * @note The SysTick timer will be enabled and configured with the provided parameters during the function execution.
 *
 * @return None
 */
void SysTick_Interrupt_Init(uint32_t clock_cycles, uint32_t priority);

#endif /* INC_SYSTICK_INTERRUPT_H_ */
