/**
 * @file EUSCI_A2_UART.h
 * @brief Header file for the UART driver.
 *
 * This file contains the function definitions for the UART driver.
 *
 * It interfaces with another device, the T4.0, via UART
 *
 * The following connections must be made:
 *  - UART TXO  (Pin 3)  <-->   MSP432 LaunchPad Pin P9.6 (PM_UCA3RXD)
 *  - UART RXI  (Pin 4)  <-->   MSP432 LaunchPad Pin P9.7 (PM_UCA3TXD)
 *
 * @note For more information regarding the Enhanced Universal Serial
 *      Communication Interface (eUSCI), refer to the MSP432Pxx Micro-
 *      controllers Technical Reference Manual
 *
 * @author Aaron Nanas, Jeff P, and Gian F
 *
 */
#ifndef INC_BLE_A3_UART_H_
#define INC_BLE_A3_UART_H_


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "msp.h"
#include "Clock.h"

#include "Project_config.h"


//#define DEBUG_OUTPUT 1

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


// ----------------------------------------------------------------------------
//
//  TASK INITIALIZATION
//
// ----------------------------------------------------------------------------

/**
 * @brief type definition of an UART ISR function pointer
 */
typedef void (*UART_ISR_Task)(volatile char *);

/**
 * @brief static function pointer that will be used when the UART ISR runs
 */
static UART_ISR_Task task_function;

/**
 * @brief BLE buffer
 */
static volatile char UART_Data_Buffer[UART_BUFFER_SIZE] = {0};

/**
 * @brief
 */
volatile int     message_length;

/**
 * @brief pointer to the array base address
 */
static volatile char   *UART_BUFFER_ADDR    = NULL;

/**
 * @brief pointer to the rolling index address
 */
static volatile char   *uart_buffer_pointer = NULL;


// ----------------------------------------------------------------------------
//
//  BLE INITIALIZATION
//
// ----------------------------------------------------------------------------


/**
 * @brief   initializes the UART module.
 */
void UART_Init(UART_ISR_Task task);

/**
 * @brief   receives a character from the UART module.
 *
 * @return uint8_t
 */
uint8_t UART_InChar();

/**
 * @brief   sends a character to the UART module.
 *
 * @param data character to be sent
 */
void UART_OutChar(uint8_t data);

/**
 * @brief   receives a string from the UART module.
 *
 * @param buffer_pointer    pointer to the character array where the received
 *                              string will be stored.
 * @param buffer_size       expected size of the character array
 * @return int
 */
int UART_InString(char *buffer_pointer, uint16_t buffer_size);

/**
 * @brief   sends a string to the UART module.
 *
 * @param pt    pointer to the character array to be sent
 */
void UART_OutString(char *pt);

/**
 * @brief   sends a fixed-point number to the UART module.
 *
 * @param pt    fixed-point number to be sent
 */
void UART_OutFixed(int32_t pt);

/**
 * @brief   checks if a specific string is present in the received data buffer.
 *
 * @param UART_Data_Buffer  buffer/reference containing the received data
 * @param data_string           pointer to the string to be checked
 * @return uint8_t
 */
uint8_t Check_UART_Data(volatile char UART_Data_Buffer[], char *data_string);

/**
 * @brief Resets the UART module.
 */
void UART_Reset();

#endif /* INC_BLE_A3_UART_H_ */
