/**
 * @file Timer_A2_Capture.c
 * @brief Source code for the Timer_A2_Capture driver.
 *
 * This file contains the function definitions for configuring Timer A2 in Capture mode.
 * Timer_A2_Capture is used to detect the edges of a square wave signal.
 *
 * The following pin is used as an input capture pin:
 *  - P5.6 (PM_TA2.1)
 *
 * @author Aaron Nanas
 */

#include "../inc/Timer_A2_Capture.h"

void Timer_A2_Capture_Init(void(*task)(uint16_t time))
{
    // Store the user-defined task function for use during interrupt handling
    Timer_A2_Capture_Task = task;

    // Configure pin P5.6 (PM_TA2.1) to use peripheral function mode
    // by setting Bit 6 of the SEL0 register for P5
    // and clearing Bit 6 of the SEL1 register for P5
    P5->SEL0 |=  0x40;
    P5->SEL1 &= ~0x40;

    // Configure pin P5.6 as an input pin by
    // clearing Bit 6 of the DIR register for P5
    P5->DIR &= ~0x40;

    // Halt Timer A2 by clearing the MC bits in the CTL register
    TIMER_A2->CTL &= ~0x0030;

    // In the CTL register, set the TASSEL bits
    // Choose SMCLK as timer clock source (TASSEL = 10b)
    // Choose prescale value of 1 (ID = 0)
    TIMER_A2->CTL |= 0x0200;

    // Modify the following bits in the CCTL[1] register
    // Capture on Rising Edge (Bit 15-14 = 01)
    // Synchronous Capture Source (Bit 11 = 1)
    // Capture Mode (Bit 8 = 1)
    // Capture / Compare Interrupt Enable (Bit 4 = 1)
    // Clear Capture / Compare Interrupt Flag (Bit 0 = 0)
    TIMER_A2->CCTL[1] |= 0x4910;

    // Divide the SMCLK frequency by 1 by clearing the
    // TAIDEX bits of the EX0 register
    TIMER_A2->EX0 = 0x0000;

    // Set interrupt priority level to 3 using the IPR3 register of NVIC
    // Timer A2_N has an IRQ number of 13
    NVIC->IP[3] = (NVIC->IP[3] & 0xFFFF00FF) | 0x00006000;

    // Enable Interrupt 13 in NVIC by setting Bit 13 of the ISER register
    NVIC->ISER[0] |= 0x00002000;

    // Set the TACLR bit and enable Timer A2 in continuous mode using the
    // MC bits in the CTL register
    TIMER_A2->CTL |= 0x0024;
}

void TA2_N_IRQHandler(void)
{
    // Acknowledge Capture/Compare interrupt and clear it
    TIMER_A2->CCTL[1] &= ~0x0001;

    // Execute the user-defined task and pass the timer value
    (*Timer_A2_Capture_Task)(TIMER_A2->CCR[1]);
}
