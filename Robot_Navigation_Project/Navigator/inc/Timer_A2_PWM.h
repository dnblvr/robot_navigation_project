/**
 * @file Timer_A2_PWM.h
 * @brief Header file for the Timer_A2_PWM driver.
 *
 * This file contains the function definitions for configuring Timer A2 in Pulse Width Modulation (PWM) mode.
 * It provides functions for initializing the PWM timer, updating duty cycles for PWM signals, and controlling
 * the PWM output on specific pins.
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_TIMER_A2_PWM_H_
#define INC_TIMER_A2_PWM_H_

#include <stdint.h>
#include "msp.h"

// Periodic interrupt rate of 1 kHz for the Periodic Interrupts lab
#define TIMER_A1_INT_CCR0_VALUE 12000

// Constant used to define the period of the PWM signal generated by Timer A2
// General formula: Period = (2*period_constant) / (12 MHz / Prescale Value)
// In this case: Period = (2*60000) / (12 MHz / 2) = 20 ms
// Note: SMCLK frequency is being divided by 2 in the Timer_A2_PWM_Init function
#define TIMER_A2_PERIOD_CONSTANT 60000

/**
 * @brief Initialize Timer A2 in PWM mode.
 *
 * This function initializes Timer A2 to generate PWM signals on specific pins. It configures the timer's clock source,
 * prescale value, and PWM mode settings.
 *
 * @param period The PWM period constant.
 *
 * @param duty_cycle_1 The duty cycle for the PWM signal, P5.6 (PM_TA2.1, CCR[1])
 *
 * @param duty_cycle_2 The duty cycle for the PWM signal, P5.7 (PM_TA2.2, CCR[2])
 *
 * @return None
 */
void Timer_A2_PWM_Init(uint16_t period_constant, uint16_t duty_cycle_1, uint16_t duty_cycle_2);

/**
 * @brief Update the Timer A2 duty cycle for the PWM signal, P5.6 (PM_TA2.1, CCR[1])
 *
 * @param duty_cycle_1 The new duty cycle for the PWM signal, P5.6 (PM_TA2.1, CCR[1])
 *
 * @return None
 */
void Timer_A2_Update_Duty_Cycle_1(uint16_t duty_cycle_1);

/**
 * @brief Update the Timer A2 duty cycle for the PWM signal, P5.7 (PM_TA2.2, CCR[2])
 *
 * @param duty_cycle_2 The new duty cycle for the PWM signal, P5.7 (PM_TA2.2, CCR[2])
 *
 * @return None
 */
void Timer_A2_Update_Duty_Cycle_2(uint16_t duty_cycle_2);

#endif /* INC_TIMER_A2_PWM_H_ */
