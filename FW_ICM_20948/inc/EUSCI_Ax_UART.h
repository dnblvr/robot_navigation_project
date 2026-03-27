/*
 * EUSCI_Ax_UART.h
 *
 *  Created on: Mar 26, 2026
 *      Author: gianf
 */

#ifndef INC_EUSCI_AX_UART_H_
#define INC_EUSCI_AX_UART_H_



// ----------------------------------------------------------------------------
//
//  CONSTANTS
//
// ----------------------------------------------------------------------------


/**
 * @brief Specifies the size of the buffer used for the UART module
 */
#define UART_BUFFER_SIZE 64

/**
 * @brief Carriage return character
 */
#define CR   0x0D

/**
 * @brief Line feed character
 */
#define LF   0x0A

/**
 * @brief Back space character
 */
#define BS   0x08

/**
 * @brief Escape character
 */
#define ESC  0x1B

/**
 * @brief Space character
 */
#define SP   0x20

/**
 * @brief Delete character
 */
#define DEL  0x7F

/**
 * @brief RX interrupt mask
 */
#define EUSCI_RXIFG  0x0001

/**
 * @brief NVIC IP index for the EUSCIA series
 */
#define IP_EUSCIA  4

/**
 * @brief NVIC IP offset for A2
 */
#define EUSCIA2_OFFSET 21

/**
 * @brief NVIC IP offset for A2
 */
#define EUSCIA3_OFFSET 29


// ----------------------------------------------------------------------------
//
//  TASK INITIALIZATION
//
// ----------------------------------------------------------------------------


/**
 * @brief type definition of an UART ISR function pointer
 */
typedef void (*UART_ISR_Task)(volatile char *);


// ----------------------------------------------------------------------------
//
//  UART OPERATIONS
//
// ----------------------------------------------------------------------------


/**
 * @brief   checks if a specific string is present in the received data buffer.
 *
 * @param UART_Data_Buffer  buffer/reference containing the received data
 * @param data_string       pointer to the string to be checked
 * @return uint8_t
 */
uint8_t Check_UART_Data(
        volatile char  UART_Data_Buffer[],
                 char *data_string);


#endif /* INC_EUSCI_AX_UART_H_ */
