/**
 * @file EUSCI_A2_UART.h
 * @brief Header file for the EUSCI_A2_UART driver.
 *
 * This file contains the function definitions for the EUSCI_A2_UART driver.
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

#ifndef INC_EUSCI_A2_UART_H_
#define INC_EUSCI_A2_UART_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "msp.h"
#include "GPIO.h"

#include "Print_Binary.h"


//#define RPLIDAR_DEBUG 1
// #define TX_RX_CHECKS 1
// #define ERROR_CHECKING 1
// #define TIMER_A



/**
 * @brief Buffer length for UART communication.
 */
#define BUFFER_LENGTH 256*9


/**
 * @brief The EUSCI_A2_UART_Init function initializes the EUSCI_A2 module to use UART mode.
 *
 * @details This function configures the EUSCI_A2 module to enable UART mode
 *          with the following configuration:
 *
 *      - Parity: Disabled
 *      - Bit Order: Least Significant Bit (MSB) first
 *      - Character Length: 8 data bits
 *      - Stop Bits: 1
 *      - Mode: UART
 *      - UART Clock Source: SMCLK
 *      - Baud Rate: 460800
 *
 * For more information regarding the registers used, refer to the eUSCI_A UART Registers section (24.4)
 * of the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @note This function assumes that the necessary pin configurations for UART communication have been performed
 *       on the corresponding pins. P2.2 is used for UART RX while P2.3 is used for UART TX.
 *
 * @return None
 */
void EUSCI_A2_UART_Init();

/**
 * @brief Stops the EUSCI_A2 module.
 *
 * This function stops the EUSCI_A2 module by holding it in the reset state,
 * disabling the necessary interrupts, and then releasing it from the reset state.
 */
void EUSCI_A2_UART_Stop();

/**
 * @brief Restarts the EUSCI_A2 module.
 *
 * This function restarts the EUSCI_A2 module by holding it in the reset state,
 * enabling the necessary interrupts, and then releasing it from the reset state.
 */
void EUSCI_A2_UART_Restart();


// -------------------------------------------------------------------------------------

//  DATA SEND AND TRANSMIT

// -------------------------------------------------------------------------------------

/**
 * @brief Receives a single character over UART using the EUSCI_A2 module.
 *
 * This function receives a single character over UART using the EUSCI_A2 module.
 * It waits until a character is available in the UART receive buffer and then reads
 * the received data. A character consists of the following bits:
 *
 * - 1 Start Bit
 * - 8 Data Bits
 * - 1 Stop Bit
 *
 * @return The received unsigned 8-bit data from UART.
 */
uint8_t EUSCI_A2_UART_InChar();

/**
 * @brief Receives a 32-bit unsigned integer over UART using the EUSCI_A2 module.
 *
 * This function receives a 1-byte format in a 32-bit unsigned integer over UART using the EUSCI_A2 module.
 * It waits until a character is available in the UART receive buffer and then reads
 * the received data. The function assumes that the data is sent in little-endian format.
 *
 * @return The received 32-bit unsigned integer from UART.
 */
uint32_t EUSCI_A2_UART_In32UInt();


/**
 * @brief Transmits a single character over UART using the EUSCI_A2 module.
 *
 * This function transmits a single character over UART using the EUSCI_A2 module.
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
void EUSCI_A2_UART_OutChar(uint8_t data);



// -------------------------------------------------------------------------------------
// 
//  HIGHER-LEVEL RPLiDAR FUNCTIONS
// 
// -------------------------------------------------------------------------------------


/**
 * @brief Checks if the given data response packet index matches the expected RPLiDAR
 *              data response pattern.
 *
 * @details From page 15 of the RPLiDAR C1 Interface Protocol and Application Notes,
 *              the expected data response patterns are:
 *
 *          Byte offset 0:      start flag = bit 0      --> bit 0 = !(bit 1)
 *                              expected patterns: 0b10 or 0b01
 *          Byte offset 1:      check flag = bit 0
 *                              expected pattern:  0b01
 *
 * @param pointer Pointer to the array data buffer to check.
 *
 * @return uint8_t 1 if the pattern matches, 0 otherwise.
 */
inline uint8_t pattern(uint8_t *pointer);


/**
 * @brief Converts the raw data from the RPLiDAR to angle and distance values.
 *
 * @param base_ptr       Pointer to the raw data buffer.
 * @param distance_angle Array to store the converted distance and angle values.
 */
void to_angle_distance(
        uint8_t    *base_ptr,
        float       distance_angle[2]);


/**
 * @brief Sends a command to the RPLiDAR and waits for a single response.
 *
 * @param command The command to send.
 */
void Single_Request_No_Response(const uint16_t command[3]);


/**
 * @brief Sends a command to the RPLiDAR and waits for a single response.
 *
 * @param command The command to send.
 * @param RX_DATA_BUFFER The buffer to store the received response.
 * @return uint8_t status
 */
uint8_t Single_Request_Single_Response(
        const uint8_t        command[2],
              uint8_t RX_DATA_BUFFER[BUFFER_LENGTH]);


/**
 * @brief Sends a command to the RPLiDAR and waits for a single response.
 *
 * @param command The command to send.
 * @param RX_DATA_BUFFER The buffer to store the received response.
 */
void Single_Request_Multiple_Response(
        const uint8_t        command[2],
              uint8_t RX_DATA_BUFFER[BUFFER_LENGTH]);



/**
 * @brief       Get data from the RPLiDAR C1
 *
 * @param scan_confirmation Confirmation flag for the scan
 * @return None
 */
void Gather_LiDAR_Data(
        uint8_t scan_confirmation,
        uint8_t RX_Data[BUFFER_LENGTH]);


// -------------------------------------------------------------------------------------
// 
//  TRANSMIT AND RECEIVE CHECKING FUNCTIONS
// 
// -------------------------------------------------------------------------------------

#ifdef TX_RX_CHECKS

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
uint8_t EUSCI_A2_UART_Transmit_Data();

/**
 * @brief The EUSCI_A2_UART_Ramp_Data function transmits the values 0 to 255 to the UART Transmit Buffer.
 *
 * The EUSCI_A2_UART_Ramp_Data function tests the EUSCI_A2_UART module using a loopback test. A loopback test can be done by
 * connecting the P3.3 pin (UART TX) to the P3.2 pin (UART RX). The EUSCI_A2_UART_OutChar function will transmit the values 0 to 255 and write
 * the values to TX_Buffer. Then, the EUSCI_A2_UART_InChar function will read the value from the UART Receive Buffer and the received
 * data will be written to RX_Buffer.
 *
 * @param uint8_t TX_Buffer[] The transmit buffer that is used to store the values transmitted on the UART TX line.
 *
 * @param uint8_t RX_Buffer[] The receive buffer that is used to store the values received from the UART RX line.
 *
 * @return None
 */
void EUSCI_A2_UART_Ramp_Data(uint8_t TX_Buffer[], uint8_t RX_Buffer[]);

/**
 * @brief The EUSCI_A2_UART_Validate_Data function verifies if the data sent and the data received are the same.
 *
 * The EUSCI_A2_UART_Validate_Data function is used to verify whether or not the loopback test was successful by comparing
 * the transmitted data stored in TX_Buffer and the received data stored in RX_Buffer. It prints the contents
 * of both buffers and outputs a warning if they do not match.
 *
 * @param uint8_t TX_Buffer[] The transmit buffer that is used to store the values transmitted on the UART TX line.
 *
 * @param uint8_t RX_Buffer[] The receive buffer that is used to store the values received from the UART RX line.
 *
 * @return None
 */
void EUSCI_A2_UART_Validate_Data(uint8_t TX_Buffer[], uint8_t RX_Buffer[]);

#endif // TX_RX_CHECKS




#endif /* INC_EUSCI_A2_UART_H_ */
