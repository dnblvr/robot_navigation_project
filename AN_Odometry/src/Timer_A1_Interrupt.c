/**
 * @file Timer_A1_Interrupt.c
 * @brief Source code for the Timer_A1_Interrupt driver.
 *
 * This file contains the function definitions for the Timer_A1_Interrupt driver.
 * It uses the Timer_A1 timer to generate periodic interrupts at a specified rate.
 *
 * @note The interrupt rate has been set to 1 kHz for the Periodic Interrupts lab.
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/Timer_A1_Interrupt.h"

void Timer_A1_Interrupt_Init(
        void(*task_0)(void),    uint16_t skip_0,
        void(*task_1)(void),    uint16_t skip_1,
//        void(*task_2)(void),    uint16_t skip_2,

        uint16_t period)
{
    /**
     * @details
     *
     * Under this timer configuration, the interrupt triggers
     *  at the time declared at CCR[0].
     *
     * With SMCLK as a clock (at 12 MHz) and a total clock division of:
     *  - ID * EX0 = 4*5 = 20, and
     *  - a CCR[0] count value of 60,000:
     *
     *      - this should achieve a period time of 0.1 seconds.
     */


    interrupt_counter   = 0;

    // Store the user-defined task function for use during interrupt handling
    Timer_A1_Task_0	= task_0;
    Timer_A1_Task_1	= task_1;
//    Timer_A1_Task_2	= task_2;

    skip_amount_0   = skip_0;
    skip_amount_1   = skip_1;
//    skip_amount_2   = skip_2;


    // Halt Timer A1 by clearing the MC bits in the CTL register
    TIMER_A1->CTL &= ~0x0030;


    // In the CTL register, set the TASSEL and ID bits:
    //  - Choose SMCLK as timer clock source (TASSEL = 10b)
    //  - Choose prescale value of 4 (ID = 2)
    TIMER_A1->CTL  |=  0x0200;
    TIMER_A1->CTL  |=  0x0080;


    // Enable interrupt request of the
    // corresponding Capture/Compare interrupt flag
    // in the CCTL[0] register using the CCIE bit
    TIMER_A1->CCTL[0] |= 0x0010;


    // Store the period in the CCR0 register
    // Note: Timer starts counting from 0
    TIMER_A1->CCR[0] = (period - 1);


    // Divide the SMCLK frequency by 5 by setting the
    // TAIDEX bits of the EX0 register
    TIMER_A1->EX0  &= ~0xFFFF;
    TIMER_A1->EX0  |=  0x0004;


    // Set interrupt priority level to 2 using the IPR2 register of NVIC
    // Timer A1 has an IRQ number of 10
    NVIC->IP[2] = (NVIC->IP[2] & 0xFF00FFFF) | 0x00600000;


    // Enable Interrupt 10 in NVIC by setting Bit 10 of the ISER register
    NVIC->ISER[0] |= 0x00000400;


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

void TA1_0_IRQHandler(void)
{

    // Acknowledge Capture/Compare interrupt and clear it
    TIMER_A1->CCTL[0]  &= ~0x0001;


//    printf("in TA1_0_IRQ_Handler\n");

    // increment and/or reset counter
    interrupt_counter++;
    if (interrupt_counter > MAX_INTERRUPT_COUNTER) {
        interrupt_counter   = 0;
    }


    // Execute the user-defined tasks
    if ( (interrupt_counter % skip_amount_0) == 0 ) {

//        printf("in task_0\t");
        (*Timer_A1_Task_0)();
    }

    if ( (interrupt_counter % skip_amount_1) == 0 ) {

//        printf("in task_1\n");
        (*Timer_A1_Task_1)();
    }

//    if ( (interrupt_counter % skip_amount_2) == 0 ) {
//
//        (*Timer_A1_Task_2)();
//    }
}
