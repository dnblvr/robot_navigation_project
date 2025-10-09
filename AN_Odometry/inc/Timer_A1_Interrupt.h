/**
 * @file Timer_A1_Interrupt.h
 * @brief Header file for the Timer_A1_Interrupt driver.
 *
 * This file contains the function definitions for the Timer_A1_Interrupt driver.
 * It uses the Timer_A1 timer to generate periodic interrupts at a specified rate.
 *
 * @note The interrupt rate has been set to 1 kHz for the Periodic Interrupts lab.
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_TIMER_A1_INTERRUPT_H_
#define INC_TIMER_A1_INTERRUPT_H_

#include <stdint.h>
#include <stdio.h>
#include "msp.h"


// Periodic interrupt rate of 2 kHz for the Tachometer Lab
#define TIMER_A1_INT_CCR0_VALUE 30000


// Declare pointer to the user-defined functions and their respective skip-timer amounts
void (*Timer_A1_Task_0)(void);
//void (*Timer_A1_Task_1)(void);
//void (*Timer_A1_Task_2)(void);

//static uint16_t skip_amount_0 = 0;
//static uint16_t skip_amount_1 = 0;
//static uint16_t skip_amount_2 = 0;

/**
 * @brief Initialize Timer A1 for periodic interrupt generation.
 *
 * This function initializes Timer A1 to generate periodic interrupts with a user-defined task function
 * at a specified period. It configures the timer's clock source, prescale value, and interrupt settings.
 * The user-defined task function will be called each time the interrupt is triggered.
 *
 * @param[in] task A pointer to the user-defined task function to be executed upon each interrupt.
 *
 * @param[in] period The period for generating interrupts, in timer ticks.
 *
 * @note The Timer_A1_Interrupt_Init function should be called before using Timer A1 interrupts.
 *
 * @return None
 */
void Timer_A1_Interrupt_Init(
        void(*task_0)(void),

        uint16_t period);

/**
 * @brief Stop Timer A1 and disable its associated interrupt.
 *
 * This function halts Timer A1 by clearing its control bits and disables the corresponding interrupt in the NVIC.
 *
 * @note The Timer_A1_Interrupt_Init function should be called again to restart Timer A1 and its interrupt.
 *
 * @return None
 */
void Timer_A1_Stop(void);

#endif /* INC_TIMER_A1_INTERRUPT_H_ */
