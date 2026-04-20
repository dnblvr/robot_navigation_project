/**
 * @file EUSCI_A2_UART.h
 * @brief Header file for the common UART driver.
 *
 * This file contains the function definitions for the UART driver.
 *
 * It interfaces with another device, the T4.0, via UART.
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
#ifndef INC_EUSCI_A2_UART_H_
#define INC_EUSCI_A2_UART_H_


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "msp.h"
#include "Clock.h"

#include "Project_Config.h"
#include "EUSCI_Ax_UART.h"



// ----------------------------------------------------------------------------
//
//  TASK INITIALIZATION
//
// ----------------------------------------------------------------------------


/**
 * @brief static function pointer that will be used when the UART ISR runs
 */
static UART_ISR_Task task_function;

/**
 * @brief UART buffer
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
//  UART INITIALIZATION
//
// ----------------------------------------------------------------------------


/**
 * @brief   initializes the UART module.
 */
void UART_A2_Init(UART_ISR_Task task);

/**
 * @brief   receives a character from the UART module.
 *
 * @return uint8_t
 */
uint8_t UART_A2_InChar();

/**
 * @brief   sends a character to the UART module.
 *
 * @param data character to be sent
 */
void UART_A2_OutChar(uint8_t data);

/**
 * @brief   receives a string from the UART module.
 *
 * @param buffer_pointer    pointer to the character array where the received
 *                              string will be stored.
 * @param buffer_size       expected size of the character array
 * @return int
 */
int UART_A2_InString(char *buffer_pointer, uint16_t buffer_size);

/**
 * @brief   sends a string to the UART module.
 *
 * @param pt    pointer to the character array to be sent
 */
void UART_A2_OutString(char *pt);

/**
 * @brief   sends a fixed-point number to the UART module.
 *
 * @param pt    fixed-point number to be sent
 */
void UART_A2_OutFixed(int32_t pt);


#endif /* INC_BLE_A3_UART_H_ */
