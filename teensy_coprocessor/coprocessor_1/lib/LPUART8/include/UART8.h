



#ifndef __UART8_H__
#define __UART8_H__


#include "LPUART8.h"
#include <inEKF_se2.h>


// ----------------------------------------------------------------------------
// 
//  DATA STRUCTURES & CONSTANTS
// 
// ----------------------------------------------------------------------------

/**
 * @brief 
 */
#define FLAG_MASK(n)        (1 << (n))

/**
 * @brief 
 */
#define ECHO_REQUEST_FLAG   FLAG_MASK(0)

/**
 * @brief 
 */
#define STATE_REQUEST_FLAG   FLAG_MASK(1)


// ----------------------------------------------------------------------------
// 
//  FUNCTION DEFINITIONS
// 
// ----------------------------------------------------------------------------

/**
 * @brief 
 */
void Handle_UART_Communications(volatile char UART_Buffer[]);

/**
 * @brief generalized function that block-waits until certain UART messages are
 *  received
 * 
 * @note This function assumes the built-in LED is available for visual feedback while waiting
 *
 * @param[in] requested_flag   flag that is requested for listening
 */
void Block_Wait_Until(uint32_t requested_flag);

/**
 * @brief get function for the latest pose received over UART
 * 
 * @return state_se2_t
 */
state_se2_t Get_Current_State(void);


#endif /* __UART8_H__ */ 