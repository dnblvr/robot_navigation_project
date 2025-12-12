/*
 * Profiler.h
 */

#ifndef __INC_PROFILER_H__
#define __INC_PROFILER_H__

#include "Project_Config.h"

#include "../inc/SysTick_Interrupt.h"
#include <stdio.h>
#include <stdint.h>



// The number of clock cycles between SysTick interrupts. The frequency of the
//  clock used is 48 MHz, so each clock cycle would last about 20.83 ns. There-
//  fore, using a value of 48,000 would result in the time interval for a
//  SysTick interrupt of 1 ms.
#define SYSTICK_NUM_CLK_CYCLES_1MS          48000
#define SYSTICK_NUM_CLK_CYCLES_1US          48
#define SYSTICK_NUM_CLK_CYCLES_QUARTER_US   12


// The priority level of the SysTick interrupt
#define SYSTICK_INT_PRIORITY 1

/**
 * @brief Interrupt-driven ISR for the SysTick. This function counts in milliseconds.
 */
void SysTick_Handler();


/**
 * @brief start the timer
 */
void Start_Timer(uint32_t timer_increments);


/**
 * @brief stop the timer and print elapsed time
 */
uint32_t Stop_Timer();


#endif /* __INC_PROFILER_H__ */
