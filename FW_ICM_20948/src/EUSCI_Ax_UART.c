/**
 * @file EUSCI_A2_UART.c
 * @brief Source code for the UART driver.
 *
 * This file contains the function definitions for the UART driver.
 *
 * This is used to 
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
//  UART INITIALIZATION
//
// ----------------------------------------------------------------------------

uint8_t Check_UART_Data(
        volatile char  UART_Data_Buffer[],
                 char *data_string)
{
    if (strstr((const char*)UART_Data_Buffer, data_string) != NULL) {

        return 0x01;

    } else {

        return 0x00;
    }
}
