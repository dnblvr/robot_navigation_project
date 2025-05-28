/**
 * @file Timer_A3_Capture.c
 * @brief Source code for the Timer_A3_Capture driver.
 *
 * This file contains the function definitions for configuring Timer A3 in Capture mode.
 * with interrupts enabled. The following pins are used:
 *  - P10.4 (Right Encoder A)
 *  - P10.5 (Left Encoder A)
 *
 * The interrupts are triggered when the rising edges of the signals from P10.4 and P10.5 are detected.
 *
 * Timer A3 is used as a base driver for the Tachometer driver.
 *
 * @author Aaron Nanas
 */

#include "../inc/Timer_A3_Capture.h"

void Timer_A3_Capture_Init(void(*task0)(uint16_t time), void(*task1)(uint16_t time))
{
    // Store the first user-defined task function for use during interrupt handling
    Timer_A3_Capture_Task_0 = task0;


    // Store the second user-defined task function for use during interrupt handling
    Timer_A3_Capture_Task_1 = task1;


    // Configure pins P10.4 (PM_TA3.0, Right Encoder A) and
    // P10.5 (PM_TA3.1, Left Encoder A) to use peripheral function mode
    // by setting Bits 4 and 5 of the SEL0 register for P10
    // and clearing Bit 4 and 5 of the SEL1 register for P10
    P10->SEL0 |=  0x30;
    P10->SEL1 &= ~0x30;


    // Configure pins P10.4 and P10.5 as input pins by
    // clearing Bits 4 and 5 of the DIR register for P10
    P10->DIR  &= ~0x30;


    // Halt Timer A3 by clearing the MC bits in the CTL register
    TIMER_A3->CTL &= ~0x0030;


    // In the CTL register, set the TASSEL bits
    // Choose SMCLK as timer clock source (TASSEL = 10b)
    // Choose prescale value of 1 (ID = 0)

    TIMER_A3->CTL &= ~0x03C0;   // clear relevant bits
    TIMER_A3->CTL |=  0x0200;   // TASSEL set to SMCLK
    TIMER_A3->CTL &= ~0x00C0;   // Input Divider cleared to 0b00 or /1


    // Modify the following bits in the CCTL[0] register
    // Capture on Rising Edge (Bit 15-14 = 01)
    // Synchronous Capture Source (Bit 11 = 1)
    // Capture Mode (Bit 8 = 1)
    // Capture / Compare Interrupt Enable (Bit 4 = 1)
    // Clear Capture / Compare Interrupt Flag (Bit 0 = 0)

    TIMER_A3->CCTL[0] &= ~0xC911;   // clear bits: 0b1100 1001 0001 0001
    TIMER_A3->CCTL[0] |=  0x4000;   // capture on rising edge
    TIMER_A3->CCTL[0] |=  0x0800;   // synchronous capture source
    TIMER_A3->CCTL[0] |=  0x0100;   // capture mode
    TIMER_A3->CCTL[0] |=  0x0010;   // interrupt EN
    TIMER_A3->CCTL[0] &= ~0x0001;   // clear CC_IFG


    // Modify the following bits in the CCTL[1] register
    // Capture on Rising Edge (Bit 15-14 = 01)
    // Synchronous Capture Source (Bit 11 = 1)
    // Capture Mode (Bit 8 = 1)
    // Capture / Compare Interrupt Enable (Bit 4 = 1)
    // Clear Capture / Compare Interrupt Flag (Bit 0 = 0)

    TIMER_A3->CCTL[1] &= ~0xC911;   // clear bits: 0b1100 1001 0001 0001
    TIMER_A3->CCTL[1] |=  0x4000;   // capture on rising edge
    TIMER_A3->CCTL[1] |=  0x0800;   // synchronous capture source
    TIMER_A3->CCTL[1] |=  0x0100;   // capture mode
    TIMER_A3->CCTL[1] |=  0x0010;   // interrupt EN
    TIMER_A3->CCTL[1] &= ~0x0001;   // clear CC_IFG


//    TIMER_A3->CCTL[1] |=  0x4000   // capture on rising edge
//    				  |  0x0800   // synchronous capture source
//    				  |  0x0100   // capture mode
//    				  |  0x0010   // interrupt EN
//    				  & ~0x0001;   // clear CC_IFG


    // Divide the SMCLK frequency by 1 by clearing the
    // TAIDEX bits of the EX0 register
    TIMER_A3->EX0 &= ~0x0007;


    // Set interrupt priority level to 2 using the IPR3 register of NVIC
    // Timer A3_0 has an IRQ number of 14
    NVIC->IP[3] = (NVIC->IP[3] & 0xFF00FFFF) | 0x00400000;


    // Set interrupt priority level to 2 using the IPR3 register of NVIC
    // Timer A3_N has an IRQ number of 15
    NVIC->IP[3] = (NVIC->IP[3] & 0x00FFFFFF) | 0x40000000;


    // Enable Interrupt 14 and 15 in NVIC by setting Bits 14 and 15 of the ISER[0] register
    NVIC->ISER[0] |=  0x0000C000;


    // Set the TACLR bit and enable Timer A3 in continuous mode using the
    // MC bits in the CTL register
    TIMER_A3->CTL |=  0x0020;

}

void TA3_0_IRQHandler(void)
{
    // Acknowledge the Capture/Compare interrupt and clear Bit 0 of the CCTL[0] register
    TIMER_A3->CCTL[0] &= ~0x1;

    // Execute the user-defined task and pass the timer value from CCR[0]
    (*Timer_A3_Capture_Task_0)(TIMER_A3->CCR[0]);
}

void TA3_N_IRQHandler(void)
{
    // Acknowledge the Capture/Compare interrupt and clear Bit 0 of the CCTL[1] register
    TIMER_A3->CCTL[1] &= ~0x1;


    // Execute the user-defined task and pass the timer value from CCR[1]
    (*Timer_A3_Capture_Task_1)(TIMER_A3->CCR[1]);
}
