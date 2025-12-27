/**
 * @file Timer_A1_Interrupt.c
 * @brief Source code for the Timer_A1_Interrupt driver.
 *
 * This file contains the function definitions for the Timer_A1_Interrupt driver.
 * It uses the Timer_A1 timer to generate periodic interrupts at a specified rate.
 *
 * @note The interrupt rate has been set to 1 kHz for the Periodic Interrupts lab.
 *
 * @author Aaron Nanas, Gian Fajardo
 *
 */

#include "../inc/Timer_A1_Interrupt.h"


void Timer_A1_Interrupt_Init(
        void(*task_function)(void),

        uint16_t period)
{

    // Store the user-defined task function for use during interrupt handling
    Timer_A1_Task	= task_function;


    // Halt Timer A1 by clearing the MC bits in the CTL register
    TIMER_A1->CTL  &= ~0x0030;


    // In the CTL register, set the TASSEL and ID bits:
    //  - [x] Choose SMCLK as timer clock source (TASSEL = 10b)
    //  - [ ] Choose prescale value of 4 (ID = 2); 2 << 6
    //  - [x] Choose prescale value of 8 (ID = 3); 3 << 6
    TIMER_A1->CTL  |=  0x0200;
    TIMER_A1->CTL  |=  0x0080;
//    TIMER_A1->CTL  |=  0x00C0;


    // Enable interrupt request of the
    // corresponding Capture/Compare interrupt flag
    // in the CCTL[0] register using the CCIE bit
    TIMER_A1->CCTL[0]  |= 0x0010;


    // Store the period in the CCR0 register
    // Note: Timer starts counting from 0
    TIMER_A1->CCR[0]    = (period - 1);


    // Divide the SMCLK frequency by 5 by setting the
    // TAIDEX bits of the EX0 register
    TIMER_A1->EX0  &= ~0xFFFF;
    TIMER_A1->EX0  |=  0x0004;


    // Set interrupt priority level to n using the IPR2 register of NVIC
    // Timer A1 has an IRQ number of 10
    NVIC->IP[2]     = (NVIC->IP[2] & 0xFF00FFFF) | 0x00000000; // 0
//    NVIC->IP[2]     = (NVIC->IP[2] & 0xFF00FFFF) | 0x00200000; // 1
//    NVIC->IP[2]     = (NVIC->IP[2] & 0xFF00FFFF) | 0x00400000; // 2
//    NVIC->IP[2]     = (NVIC->IP[2] & 0xFF00FFFF) | 0x00600000; // 3


    // Enable Interrupt 10 in NVIC by setting Bit 10 of the ISER register
    NVIC->ISER[0]  |= 0x00000400;


    // Set the TACLR bit and enable Timer A1 in up mode using the
    // MC bits in the CTL register
    TIMER_A1->CTL  |= 0x0004;   // TACLR
    TIMER_A1->CTL  |= 0x0010;   // MC

}

void Timer_A1_Stop(void)
{
    // Halt Timer A1 by clearing the MC bits in the CTL register
    TIMER_A1->CTL  &= ~0x0030;

    // Disable Interrupt 10 in NVIC by setting Bit 10 of the ICER register
    NVIC->ICER[0]   =  0x00000400;
}


void Timer_A1_Ignore(void) {

    TIMER_A1->CCTL[0]  &= ~0x0010;

}


void Timer_A1_Acknowledge(void) {

    TIMER_A1->CCTL[0]  |=  0x0010;
//    TIMER_A1->CCTL[0]  &=  0x0010;

}


void TA1_0_IRQHandler(void)
{

    // Acknowledge Capture/Compare interrupt and clear it
    TIMER_A1->CCTL[0]  &= ~0x0001;


    // Execute the user-defined tasks
    (*Timer_A1_Task)();

}
