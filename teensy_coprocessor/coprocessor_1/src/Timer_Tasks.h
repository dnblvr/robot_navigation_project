/**
 * @file Timer_Tasks.h
 * @author your name (you@domain.com) 
 */

#ifndef __INC_TIMER_A1_TASKS_H__
#define __INC_TIMER_A1_TASKS_H__

#include <stdint.h>

// ----------------------------------------------------------------------------
//
//  TIMER A1 TASK SELECTOR
//
// ----------------------------------------------------------------------------

/**
 * @note here are the following delegated tasks:
 *      TASK 2: Get the measurements made by the tachometers
 *      TASK 3: set flag to collect RPLiDAR C1 data
 *      TASK 4: processes recorded data and runs ICP
 *      TASK 6: print persistently
 *      TASK 7: increment counter
 *
 *  Here are unused tasks:
 *      TASK 0: blank
 *      TASK 1: blank
 *      TASK 5: blank
 */

#define MAX_DIV_FREQ    10

//#define TASK_0_FLAG     0x01 << 0
//#define TASK_0_DIV_FREQ 1
//#define TASK_0_OFFSET   0

//#define TASK_1_FLAG     0x01 << 1
//#define TASK_1_DIV_FREQ 30
//#define TASK_1_OFFSET   0

#define TASK_2_FLAG     0x01 << 2
#define TASK_2_DIV_FREQ 2
#define TASK_2_OFFSET   0

#define TASK_3_FLAG     0x01 << 3
#define TASK_3_DIV_FREQ MAX_DIV_FREQ
#define TASK_3_OFFSET   0

#define TASK_4_FLAG     0x01 << 4
#define TASK_4_DIV_FREQ MAX_DIV_FREQ
#define TASK_4_OFFSET   3

//#define TASK_5_FLAG     0x01 << 5
//#define TASK_5_DIV_FREQ 30
//#define TASK_5_OFFSET   24

#define TASK_6_FLAG     0x01 << 6
#define TASK_6_DIV_FREQ 1
#define TASK_6_OFFSET   0

#define TASK_7_FLAG     0x01 << 7
#define TASK_7_DIV_FREQ MAX_DIV_FREQ
#define TASK_7_OFFSET   MAX_DIV_FREQ



// sets the flag to be performed in the main loop
volatile uint8_t    task_flag           = 0;

/**
 * @brief When non-zero, all task_flag assignments are suppressed.
 *
 * @details Mirrors MSP432 Timer_A1_Ignore() / Timer_A1_Acknowledge().
 *  Set to 1 by the FSM (via _timer_ignore()) when it begins recording a scan
 *  frame; cleared to 0 by the FSM (via _timer_acknowledge()) when the frame
 *  is handed off for processing.  tick_counter still increments so phase is
 *  preserved across the recording window.
 */
volatile uint8_t    timer_ignore_flag   = 0;

// system tick counter
volatile uint32_t   tick_counter        = 0;


/**
 * @brief Task selector function that will run at each tick
 */
void Task_Selector(void) {

    // Dispatch task flags only when the LiDAR FSM is not mid-recording.
    // tick_counter still advances so task phase is preserved.
    if (!timer_ignore_flag) {

    #ifdef TASK_0_FLAG
        if ( ((tick_counter + TASK_0_OFFSET) % TASK_0_DIV_FREQ) == 0 )
            task_flag  |= TASK_0_FLAG;
    #endif

    #ifdef TASK_1_FLAG
        if ( ((tick_counter + TASK_1_OFFSET) % TASK_1_DIV_FREQ) == 0 )
            task_flag  |= TASK_1_FLAG;
    #endif

    #ifdef TASK_2_FLAG
        if ( ((tick_counter + TASK_2_OFFSET) % TASK_2_DIV_FREQ) == 0 )
            task_flag  |= TASK_2_FLAG;
    #endif

    #ifdef TASK_3_FLAG
        if ( ((tick_counter + TASK_3_OFFSET) % TASK_3_DIV_FREQ) == 0 )
            task_flag  |= TASK_3_FLAG;
    #endif

    #ifdef TASK_4_FLAG
        if ( ((tick_counter + TASK_4_OFFSET) % TASK_4_DIV_FREQ) == 0 )
            task_flag  |= TASK_4_FLAG;
    #endif

    #ifdef TASK_5_FLAG
        if ( ((tick_counter + TASK_5_OFFSET) % TASK_5_DIV_FREQ) == 0 )
            task_flag  |= TASK_5_FLAG;
    #endif

    #ifdef TASK_6_FLAG
        if ( ((tick_counter + TASK_6_OFFSET) % TASK_6_DIV_FREQ) == 0 )
            task_flag  |= TASK_6_FLAG;
    #endif

    #ifdef TASK_7_FLAG
        if ( ((tick_counter + TASK_7_OFFSET) % TASK_7_DIV_FREQ) == 0 )
            task_flag  |= TASK_7_FLAG;
    #endif

    } // !timer_ignore_flag

    // Increment the counter
    tick_counter++;

}


#endif /* __INC_TIMER_A1_TASKS_H__ */
