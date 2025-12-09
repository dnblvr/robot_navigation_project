/**
 * @file Timer_A2_PWM.h
 * @brief Source code for the Timer_A2_PWM driver.
 *
 * This file contains the function definitions for configuring Timer A2 in Pulse Width Modulation (PWM) mode.
 * It provides functions for initializing the PWM timer, updating duty cycles for PWM signals, and controlling
 * the PWM output on specific pins.
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/Timer_A2_PWM.h"

void Timer_A2_PWM_Init(uint16_t period_constant, uint16_t duty_cycle_1, uint16_t duty_cycle_2)
{
    // Return immediately if either duty cycle values are greater than
    // or equal to the given period_constant
    if (duty_cycle_1 >= period_constant) return;
    if (duty_cycle_2 >= period_constant) return;

    // Configure pins P5.6 (PM_TA2.1) and P5.7 (PM_TA2.2) to use peripheral function mode
    // by setting Bits 6 and 7 of the SEL0 register for P5
    // and clearing Bits 6 and 7 of the SEL1 register for P5
    P5->SEL0 |= 0xC0;
    P5->SEL1 &= ~0xC0;

    // Configure pins P5.6 and P5.7 as output GPIO pins to drive the PWM signals
    // Set Bits 6 and 7 of the DIR register for P5
    P5->DIR |= 0xC0;

    // Set the Timer A2 Capture/Compare register to the specified period_constant
    // CCR[0] is primarily used as the "period" register
    // General formula: Period = (2*period_constant) / (12 MHz / Prescale Value)
    // In this case: Period = (2*60000) / (12 MHz / 2) = 20 ms
    // Assign the value of period_constant to the CCR[0] register
    TIMER_A2->CCR[0] = period_constant;

    // Configure the Timer A2 expansion register to divide the clock frequency by 1
    // Clear all bits of the EX0 register
    TIMER_A2->EX0 = 0x0000;

    // Configure the output mode as Toggle / Reset for CCR[1]
    // Set the bits of the OUTMOD field of the CCTL[1] register to 010b
    TIMER_A2->CCTL[1] |= 0x0040;

    // Assign the value of duty_cycle_1 to the CCR[1] register
    // Duty Cycle %: duty_cycle_1 / period_constant
    TIMER_A2->CCR[1] = duty_cycle_1;

    // Configure the output mode as Toggle / Reset for CCR[2]
    // Set the bits of the OUTMOD field of the CCTL[2] register to 010b
    TIMER_A2->CCTL[2] |= 0x0040;

    // Assign the value of duty_cycle_1 to the CCR[2] register
    // Duty Cycle %: duty_cycle_2 / period_constant
    TIMER_A2->CCR[2] = duty_cycle_2;

    // Modify the following bits in the CTL register
    // Select SMCLK = 12 MHz as timer clock source
    // Set ID = 1 (Divide timer clock by 2)
    // Set MC = 3 (Up/Down Mode)
    TIMER_A2->CTL |= 0x0270;
}

void Timer_A2_Update_Duty_Cycle_1(uint16_t duty_cycle_1)
{
    // Immediately return if duty cycle is greater than the given period
    if (duty_cycle_1 >= TIMER_A2->CCR[0]) return;

    // Otherwise, update the duty cycle
    TIMER_A2->CCR[1] = duty_cycle_1;
}

void Timer_A2_Update_Duty_Cycle_2(uint16_t duty_cycle_2)
{
    // Immediately return if duty cycle is greater than the given period
    if (duty_cycle_2 >= TIMER_A2->CCR[0]) return;

    // Otherwise, update the duty cycle
    TIMER_A2->CCR[2] = duty_cycle_2;
}
