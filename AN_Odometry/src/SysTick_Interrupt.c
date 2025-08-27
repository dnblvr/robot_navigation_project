/**
 * @file SysTick_Interrupt.c
 * @brief Source code for the SysTick_Interrupt driver.
 *
 * This file contains the function definitions for the SysTick_Interrupt driver.
 * It uses the SysTick timer to perform interrupt requests at the specified period.
 * This is used to demonstrate periodic interrupts.
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/SysTick_Interrupt.h"

void SysTick_Interrupt_Init(uint32_t clock_cycles, uint32_t priority)
{
    // Disable SysTick during setup
    SysTick->CTRL = 0;

    // Set the reload value to establish the SysTick interrupt period
    SysTick->LOAD = (clock_cycles - 1);

    // Any write to the current value clears it
    SysTick->VAL = 0;

    // The SHP (System Handler Priority Register) represents an array of registers within the
    // SCB (System Control Block) that control interrupt priorities.
    // Set the priority into the upper 3 bits of the 8-bit field for SysTick interrupt
    // System Handler 15 (tied to SysTick_Handler) can be accessed at index 11 of the SHP array
    SCB->SHP[11] = priority << 5;

    // Enable SysTick with interrupts and the core clock
    SysTick->CTRL = 0x00000007;
}
