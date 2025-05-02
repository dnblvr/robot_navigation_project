/**
 * @file Timer_A2_Interrupt.h
 * @brief Header file for the Timer_A2_Interrupt driver.
 *
 * This file contains the function definitions for the Timer_A2_Interrupt driver.
 * It uses the Timer_A2 timer to generate periodic interrupts at a specified rate.
 *
 * @note The interrupt rate has been set to 1 kHz for the Periodic Interrupts lab.
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_TIMER_A2_INTERRUPT_H_
#define INC_TIMER_A2_INTERRUPT_H_

#include <stdint.h>
#include "msp.h"

// Periodic interrupt rate of 1 kHz for the Periodic Interrupts lab
#define TIMER_A2_INT_CCR0_VALUE 12000

void (*Timer_A2_Task)(void);

/**
 * @brief Initialize Timer A2 for periodic interrupt generation.
 *
 * This function initializes Timer A2 to generate periodic interrupts with a user-defined task function
 * at a specified period. It configures the timer's clock source, prescale value, and interrupt settings.
 * The user-defined task function will be called each time the interrupt is triggered.
 *
 * @param task A pointer to the user-defined task function to be executed upon each interrupt.
 *
 * @param period The period for generating interrupts, in timer ticks.
 *
 * @note The Timer_A2_Interrupt_Init function should be called before using Timer A2 interrupts.
 *
 * @return None
 */
void Timer_A2_Interrupt_Init(void(*task)(void), uint16_t period);

/**
 * @brief Stop Timer A2 and disable its associated interrupt.
 *
 * This function halts Timer A2 by clearing its control bits and disables the corresponding interrupt in the NVIC.
 *
 * @note The Timer_A2_Interrupt_Init function should be called again to restart Timer A2 and its interrupt.
 *
 * @return None
 */
void Timer_A2_Stop(void);

#endif /* INC_TIMER_A2_INTERRUPT_H_ */
