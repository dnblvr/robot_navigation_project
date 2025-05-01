/**
 * @file Timer_A0_PWM.c
 * @brief Source code for the Timer_A0_PWM driver.
 *
 * This file contains the function definitions for configuring Timer A0 in Pulse Width Modulation (PWM) mode.
 * It provides functions for initializing the PWM timer, updating duty cycles for PWM signals, and controlling
 * the PWM output on specific pins.
 *
 * @note The Motor driver uses Timer_A0_PWM as a base driver.
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/Timer_A0_PWM.h"

void Timer_A0_PWM_Init(uint16_t period_constant, uint16_t duty_cycle_1, uint16_t duty_cycle_2)
{
    // Return immediately if either duty cycle values are greater than
    // or equal to the given period_constant
    if (duty_cycle_1 >= period_constant) return;
    if (duty_cycle_2 >= period_constant) return;

    // Configure pins P2.6 (PM_TA0.3) and P2.7 (PM_TA0.4) to use peripheral function mode
    // by setting Bits 6 and 7 of the SEL0 register for P2
    // and clearing Bits 6 and 7 of the SEL1 register for P2
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;

    // Configure pins P2.6 and P2.7 as output GPIO pins to drive the PWM signals
    // Set Bits 6 and 7 of the DIR register for P2
    P2->DIR |= 0xC0;

    // Set the Timer A0 Capture/Compare register to the specified period_constant
    // CCR[0] is primarily used as the "period" register
    // General formula: Period = (2*period_constant) / (12 MHz / Prescale Value)
    // In this case: Period = (2*15000) / (12 MHz / 8) = 20 ms
    // Assign the value of period_constant to the CCR[0] register
    TIMER_A0->CCR[0] = period_constant;

    // Configure the Timer A0 expansion register to divide the clock frequency by 1
    // Clear all bits of the EX0 register
    TIMER_A0->EX0 = 0x0000;

    // Configure the output mode as Toggle / Reset for CCR[3]
    // Set the bits of the OUTMOD field of the CCTL[3] register to 010b
    TIMER_A0->CCTL[3] |= 0x0040;

    // Assign the value of duty_cycle_1 to the CCR[3] register
    // Duty Cycle %: duty_cycle_1 / period_constant
    TIMER_A0->CCR[3] = duty_cycle_1;

    // Configure the output mode as Toggle / Reset for CCR[4]
    // Set the bits of the OUTMOD field of the CCTL[4] register to 010b
    TIMER_A0->CCTL[4] |= 0x0040;

    // Assign the value of duty_cycle_1 to the CCR[4] register
    // Duty Cycle %: duty_cycle_2 / period_constant
    TIMER_A0->CCR[4] = duty_cycle_2;

    // Modify the following bits in the CTL register
    // Select SMCLK = 12 MHz as timer clock source
    // Set ID = 3 (Divide timer clock by 8)
    // Set MC = 3 (Up/Down Mode)
    TIMER_A0->CTL |= 0x02F0;
}

void Timer_A0_Update_Duty_Cycle_1(uint16_t duty_cycle_1)
{
    // Immediately return if duty cycle is greater than the given period
    if (duty_cycle_1 >= TIMER_A0->CCR[0]) return;

    // Otherwise, update the duty cycle
    TIMER_A0->CCR[3] = duty_cycle_1;
}

void Timer_A0_Update_Duty_Cycle_2(uint16_t duty_cycle_2)
{
    // Immediately return if duty cycle is greater than the given period
    if (duty_cycle_2 >= TIMER_A0->CCR[0]) return;

    // Otherwise, update the duty cycle
    TIMER_A0->CCR[4] = duty_cycle_2;
}
