/**
 * @file Profiler.c
 * @brief Source file for Implementing the profiling/timing functions
 */

#include "../inc/Profiler.h"



// ----------------------------------------------------------------------------
//
//  PROFILER FUNCTIONS
//
// ----------------------------------------------------------------------------


/**
 * @brief global variable for timing
 */
volatile uint32_t time_today = 0;


/**
 * @brief Interrupt-driven ISR for the SysTick. This function counts in incre-
 *      ments set by `Start_Timer()` parameter `timer_increments`.
 * @param None
 * @return None
 */
void SysTick_Handler(void) {

    time_today++;

}


/**
 * @brief start the timer using
 */
void Start_Timer(uint32_t timer_increments) {

    // resets the global variable counting time
    time_today = 0;

    // initializes the SysTick interrupt
    SysTick_Interrupt_Init(timer_increments,
                           SYSTICK_INT_PRIORITY);

}


/**
 * @brief stop the timer and print elapsed time
 *
 * @note it's implicit that when `DEBUG_OUTPUT` is on from `Project_Config.h`,
 *      it will print.
 */
uint32_t Stop_Timer(void) {

    // disables the timer entirely
    SysTick_Interrupt_Disable();

//#ifdef DEBUG_OUTPUT
//    printf("dt = %u ms\n", time_today);
//#endif

    return time_today;

}
