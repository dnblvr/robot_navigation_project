/**
 * @file EUSCI_A3_UART.h
 * @brief Header file for the EUSCI_A3_UART driver.
 *
 * This file contains the function definitions for the EUSCI_A3_UART driver.
 *
 * @note Assumes that the necessary pin configurations for UART communication have been performed
 *       on the corresponding pins. P9.6 is used for UART RX while P9.7 is used for UART TX.
 *
 * @note For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_EUSCI_A3_UART_H_
#define INC_EUSCI_A3_UART_H_

#include <stdint.h>
#include "msp.h"
#include "../inc/GPIO.h"

#define BUFFER_LENGTH 256

/**
 * @brief The EUSCI_A3_UART_Init function initializes the EUSCI_A3 module to use UART mode.
 *
 * This function configures the EUSCI_A3 module to enable UART mode
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
void EUSCI_A3_UART_Init();

/**
 * @brief Receives a single character over UART using the EUSCI_A3 module.
 *
 * This function receives a single character over UART using the EUSCI_A3 module.
 * It waits until a character is available in the UART receive buffer and then reads
 * the received data. A character consists of the following bits:
 *
 * - 1 Start Bit
 * - 8 Data Bits
 * - 1 Stop Bit
 *
 * @return The received unsigned 8-bit data from UART.
 */
uint8_t EUSCI_A3_UART_InChar();

/**
 * @brief Transmits a single character over UART using the EUSCI_A3 module.
 *
 * This function transmits a single character over UART using the EUSCI_A3 module.
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
void EUSCI_A3_UART_OutChar(uint8_t data);

/**
 * @brief The Transmit_UART_Data function transmits data over UART based on the status of the user buttons.
 *
 * This function transmits different data values over UART based on the status of Button 1 and Button 2.
 * The data transmitted corresponds to the button status according to the following mapping:
 *
 *  button_status      Transmitted Data
 *  -------------      ----------------
 *      0x00               0x00
 *      0x10               0xAA
 *      0x02               0x46
 *      0x12               0xF0
 *
 * @return None
 */
uint8_t EUSCI_A3_UART_Transmit_Data();

/**
 * @brief The EUSCI_A3_UART_Ramp_Data function transmits the values 0 to 255 to the UART Transmit Buffer.
 *
 * The EUSCI_A3_UART_Ramp_Data function tests the EUSCI_A3_UART module using a loopback test. A loopback test can be done by
 * connecting the P2.3 pin (UART TX) to the P2.2 pin (UART RX). The EUSCI_A3_UART_OutChar function will transmit the values 0 to 255 and write
 * the values to TX_Buffer. Then, the EUSCI_A3_UART_InChar function will read the value from the UART Receive Buffer and the received
 * data will be written to RX_Buffer.
 *
 * @param uint8_t TX_Buffer[] The transmit buffer that is used to store the values transmitted on the UART TX line.
 *
 * @param uint8_t RX_Buffer[] The receive buffer that is used to store the values received from the UART RX line.
 *
 * @return None
 */
void EUSCI_A3_UART_Ramp_Data(uint8_t TX_Buffer[], uint8_t RX_Buffer[]);

/**
 * @brief The EUSCI_A3_UART_Validate_Data function verifies if the data sent and the data received are the same.
 *
 * The EUSCI_A3_UART_Validate_Data function is used to verify whether or not the loopback test was successful by comparing
 * the transmitted data stored in TX_Buffer and the received data stored in RX_Buffer. It prints the contents
 * of both buffers and outputs a warning if they do not match.
 *
 * @param uint8_t TX_Buffer[] The transmit buffer that is used to store the values transmitted on the UART TX line.
 *
 * @param uint8_t RX_Buffer[] The receive buffer that is used to store the values received from the UART RX line.
 *
 * @return None
 */
void EUSCI_A3_UART_Validate_Data(uint8_t TX_Buffer[], uint8_t RX_Buffer[]);

#endif /* INC_EUSCI_A3_UART_H_ */
