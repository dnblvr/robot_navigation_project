/*
 * Profiler.h
 */

#ifndef __INC_PROFILER_H__
#define __INC_PROFILER_H__

#include "Project_Config.h"

#include "../inc/SysTick_Interrupt.h"
#include <stdio.h>
#include <stdint.h>

/**
 * @brief global variable for timing (defined in Profiler.c)
 */
extern volatile uint32_t time_today;


/**
 * @brief Interrupt-driven ISR for the SysTick. This function counts in milliseconds.
 */
void SysTick_Handler(void);


/**
 * @brief start the timer
 */
void Start_Timer(void);


/**
 * @brief stop the timer and print elapsed time
 */
void Stop_Timer(void);


#endif /* __INC_PROFILER_H__ */
