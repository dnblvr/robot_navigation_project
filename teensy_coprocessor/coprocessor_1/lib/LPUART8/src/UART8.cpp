/**
 * @file UART8.cpp
 * @author your name (you@domain.com)
 * @brief abstraction layer for the higher-level UART communication functions used by the application, which intentionally separates the software-only logic away from the hardware details of the LPUART8 module
 * @version 0.1
 * @date 2026-04-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "UART8.h"

// ----------------------------------------------------------------------------
// 
//  DATA STRUCTURES & CONSTANTS
// 
// ----------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @note 
 */
static volatile state_se2_t latest_pose = {0.0f, 0.0f, 0.0f};


/**
 * @brief 
 */
volatile uint8_t comms_state = 0;


// ----------------------------------------------------------------------------
// 
//  FUNCTION DEFINITIONS
// 
// ----------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @param UART_Buffer 
 */
void Handle_UART_Communications(volatile char UART_Buffer[]) {

    // if seen, communication is established
    if (Check_UART_Data(UART_Buffer, "!E")) {

        comms_state    |=  ECHO_REQUEST_FLAG;


    // state command was received. This accumulates the pose and defers
    } else if (Check_UART_Data(UART_Buffer, "#S")) {

    #ifdef DEBUG_OUTPUT
        // Serial.printf("in handle func\n");
    #endif

        // comms_state    |=  STATE_REQUEST_FLAG;

        // @todo do the conversion from UART buffer to the state struct here and set the latest pose variable, which can be retrieved by the Get_Current_State() function. 

        // alternatively, assign the last 12 bytes before the command prefix "#S"
        // state_se2_t state = *((state_se2_t*)(UART8_buffer + 2)); 
        memcpy((void*)&latest_pose,
               (const void*)(&UART_Buffer[0] + 2),
               sizeof(state_se2_t));

        // Clear the buffer after processing the message to prevent stale data from being misinterpreted in future messages. 
        memset((void*)UART_Buffer,
               0,
               UART8_BUFFER_SIZE);

    }

}

void Block_Wait_Until(uint32_t requested_flag) {

    while (!(comms_state & requested_flag)) {

        // wfi assembly code
        // WaitForInterrupt();
        asm("wfi");
    }

}

state_se2_t Get_Current_State(void) {

    return (state_se2_t)latest_pose;
}