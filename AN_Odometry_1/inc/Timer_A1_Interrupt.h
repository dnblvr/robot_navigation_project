/**
 * @file Timer_A1_Interrupt.h
 * @brief Header file for the Timer_A1_Interrupt driver.
 *
 * This file contains the function definitions for the Timer_A1_Interrupt driver.
 * It uses the Timer_A1 timer to generate periodic interrupts at a specified rate.
 *
 * @note The interrupt rate has been set to 10 Hz for this application.
 *
 * @author Aaron Nanas, Gian Fajardo
 *
 */

#ifndef INC_TIMER_A1_INTERRUPT_H_
#define INC_TIMER_A1_INTERRUPT_H_

#include <stdint.h>

#include "msp.h"


/**
 * @brief   timer_A1 counter reset value
 *
 * @details when it resets to this value, then one `tick` has passed
 */
#define TIMER_A1_CCR0_VALUE  60000


/**
 * @brief pointer to the user-defined interrupt-driven function
 */
void (*Timer_A1_Task)(void);


/**
 * @brief Initialize Timer A1 for periodic interrupt generation.
 *
 * This function initializes Timer A1 to generate periodic interrupts with a user-defined task function
 * at a specified period. It configures the timer's clock source, prescale value, and interrupt settings.
 * The user-defined task function will be called each time the interrupt is triggered.
 *
 * @param[in]   task A pointer to the user-defined task function to be executed upon each interrupt.
 *
 * @param[in]   period The period for generating interrupts, in timer ticks.
 *
 * @note        The Timer_A1_Interrupt_Init function should be called before using Timer A1 interrupts.
 *
 *
 * @details     Under this timer configuration, the interrupt triggers
 *      at the time declared at CCR[0].
 *
 * With SMCLK as a clock (at 12 MHz) and a total clock division of:
 *  - ID * EX0 = 4*5 = 20, and
 *  - a CCR[0] count value of 60,000:
 *
 *      - 12E6 counts per sec / (20 * 60,000 counts) = 10 Hz
 *      - this should achieve a tick period of `0.1 seconds per tick`.
 *
 * @return None
 */
void Timer_A1_Interrupt_Init(
        void   (*task_function)(void),

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

/**
 * @brief disable interrupt enable, while keeping the interrupt generation intact
 */
void Timer_A1_Ignore(void);


/**
 * @brief enable interrupt enable, recognizing the interrupt generation
 */
void Timer_A1_Acknowledge(void);



#endif /* INC_TIMER_A1_INTERRUPT_H_ */
