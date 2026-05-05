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
//  DOUBLE BUFFER
// 
// ----------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @note 
 */
static volatile state_se2_t pose_buffer[2] = {{0}, {0}};

/**
 * @brief index of the `pose_buffer` written by the ISR
 */
static volatile uint8_t write_index = 0;

/**
 * @brief index of the `pose_buffer` read by the application
 */
static volatile uint8_t read_index  = 1;


// ----------------------------------------------------------------------------
// 
//  COMMUNICATIONS HANDLING
// 
// ----------------------------------------------------------------------------

/**
 * @brief 
 */
volatile uint8_t comms_state = 0;

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


        /**
         * @brief implemented double-buffer logic for the pose buffer 
         */

        // copy the last 12 bytes into whichever buffer the ISR is permitted to write to
        memcpy((void*)&pose_buffer[write_index],
               (const void*)(&UART_Buffer[0] + 2),
               sizeof(state_se2_t));

        // Atomically swap buffer positions. now the freshly-written slot becomes the readable one
        write_index ^= 1;
        read_index  ^= 1;


        // Clear the buffer after processing the message to prevent stale data from being misinterpreted in future messages. 
        memset((void*)UART_Buffer,
               0,
               UART8_BUFFER_SIZE);

    }

}

void Block_Wait_Until(uint32_t requested_flag) {

    while ( !(comms_state & requested_flag) ) {

        // wfi assembly code
        asm("wfi");
    }

}

state_se2_t Get_Current_State(void) {

    // 
    state_se2_t current_state;

    // copy the most recent pose from the buffer into a local variable to return
    memcpy((void*)&current_state,
           (const void*)&pose_buffer[read_index],
           sizeof(state_se2_t));

    return current_state;
}