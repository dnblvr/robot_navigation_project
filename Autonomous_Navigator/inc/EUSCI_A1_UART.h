/**
 * @file EUSCI_A1_UART.h
 * @brief Header file for the EUSCI_A1_UART driver.
 *
 * This file contains the function definitions for the EUSCI_A1_UART driver.
 *
 * @note Assumes that the necessary pin configurations for UART communication have been performed
 *       on the corresponding pins. P3.2 is used for UART RX while P3.3 is used for UART TX.
 *
 * @note For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Gian Fajardo
 *
 */

#ifndef INC_EUSCI_A1_UART_H_
#define INC_EUSCI_A1_UART_H_

#include <stdint.h>
#include <string.h>
#include "msp.h"
#include "../inc/GPIO.h"


//#define TX_RX_CHECKS 1

#define BUFFER_LENGTH 256

/**
 * @brief The EUSCI_A1_UART_Init function initializes the EUSCI_A1 module to use UART mode.
 *
 * This function configures the EUSCI_A1 module to enable UART mode
 * with the following configuration:
 *
 * - Parity: Disabled
 * - Bit Order: Most Significant Bit (MSB) first
 * - Character Length: 8 data bits
 * - Stop Bits: 1
 * - Mode: UART
 * - UART Clock Source: SMCLK
 * - Baud Rate: 9600
 *
 * For more information regarding the registers used, refer to the eUSCI_A UART Registers section (24.4)
 * of the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @note This function assumes that the necessary pin configurations for UART communication have been performed
 *       on the corresponding pins. P2.2 is used for UART RX while P2.3 is used for UART TX.
 *
 * @return None
 */
void EUSCI_A1_UART_Init();

/**
 * @brief Receives a single character over UART using the EUSCI_A1 module.
 *
 * This function receives a single character over UART using the EUSCI_A1 module.
 * It waits until a character is available in the UART receive buffer and then reads
 * the received data. A character consists of the following bits:
 *
 * - 1 Start Bit
 * - 8 Data Bits
 * - 1 Stop Bit
 *
 * @return The received unsigned 8-bit data from UART.
 */
uint8_t EUSCI_A1_UART_InChar();

/**
 * @brief Transmits a single character over UART using the EUSCI_A1 module.
 *
 * This function transmits a single character over UART using the EUSCI_A1 module.
 * It waits until the UART transmit buffer is ready to accept new data and then writes the provided data
 * to the transmit buffer for transmission. A character consists of the following bits:
 *
 * - 1 Start Bit
 * - 8 Data Bits
 * - 1 Stop Bit
 *
 * @param data The unsigned 8-bit data to be transmitted over UART.
 *
 * @return None
 */
void EUSCI_A1_UART_OutChar(uint8_t data);

/**
 * @brief 
 * 
 * @param A2_UART_Data_Buffer 
 * @param data_string 
 * @return uint8_t 
 */
uint8_t Check_A2_UART_Data(
        char  A2_UART_Data_Buffer[],
        char *data_string           );

#endif /* INC_EUSCI_A1_UART_H_ */
