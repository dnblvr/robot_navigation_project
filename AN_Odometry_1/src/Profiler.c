/**
 * @file Profiler.c
 * @brief Implementation of profiling/timing functions
 */

#include "../inc/Profiler.h"

/**
 * @brief global variable for timing
 */
volatile uint32_t time_today = 0;


/**
 * @brief Interrupt-driven ISR for the SysTick. This function counts in milliseconds.
 * @param None
 * @return None
 */
void SysTick_Handler(void) {
    time_today++;
}


/**
 * @brief start the timer
 */
void Start_Timer(void) {
    time_today = 0;
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);
}


/**
 * @brief stop the timer and print elapsed time
 */
void Stop_Timer(void) {
    SysTick_Interrupt_Disable();

#ifdef DEBUG_OUTPUTS
    printf("dt = %u ms\n", time_today);
#endif
}
