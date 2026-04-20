/**
 * @file EUSCI_A2_UART.c
 * @brief Source code for the common EUSCIA UART driver.
 *
 * This file contains the common function definitions for the UART driver.
 *
 * @note For more information regarding the Enhanced Universal Serial
 *      Communication Interface (eUSCI), refer to the MSP432Pxx Micro-
 *      controllers Technical Reference Manual
 *
 * @author Gian Fajardo
 *
 */

#include "../inc/EUSCI_A2_UART.h"



// ----------------------------------------------------------------------------
//
//  UART COMMON FUNCTIONS
//
// ----------------------------------------------------------------------------

uint8_t Check_UART_Data(
        volatile char  UART_Data_Buffer[],
                 char* data_string)
{
    if ( strstr((const char*)UART_Data_Buffer, data_string) != NULL ) {

        return 0x01;

    }

    return 0x00;

}
