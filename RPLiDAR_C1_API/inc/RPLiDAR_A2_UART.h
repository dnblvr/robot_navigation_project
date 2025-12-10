/**
 * @file RPLIDAR_A2_UART.h
 * @brief Header file for the RPLIDAR_A2_UART driver.
 *
 * This file contains the function definitions for the EUSCI_A2_UART driver.
 *
 * @note Assumes that the necessary pin configurations for UART communication have been
 *      performed on the corresponding pins. P3.2 is used for UART RX while P3.3 is used
 *      for UART TX. Meanwhile, the configuration expects
 *
 * @note For more information regarding the Enhanced Universal Serial Communication In-
 *      terface (eUSCI), refer to the MSP432Pxx Microcontrollers Technical Reference
 *      Manual
 *
 * @author Gian Fajardo
 */

#ifndef __INC_RPLIDAR_A2_UART_H__
#define __INC_RPLIDAR_A2_UART_H__

#include "GPIO_Utilities.h"


#include "Project_Config.h"


#include "msp.h"
#include "Clock.h"
#include "coordinate_transform.h"

#include <stdint.h>
#include <string.h>


#ifdef DEBUG_OUTPUT
#include <stdio.h>          // for printf() debugging
#include "Profiler.h"
#endif



//#define RPLIDAR_DEBUG 1
//#define TX_RX_CHECKS 1
//#define ERROR_CHECKING 1



/**
 * @brief states of the UART record system
 */
typedef enum {

    HOLD            =  0,
    FIND_PATTERN,   // 1
    ADD_OFFSET,     // 2
    SKIP,           // 3
    RECORD          // 4

} Record_States;

/**
 * @brief states of the RPLiDAR C1 system itself
 */
typedef enum {

    IDLING      =  0,
    READY,      // 1
    RECORDING,  // 2
    PROCESSING  // 3

} RPLiDAR_States;


/**
 * @brief configuration struct of the RPLiDAR C1
 *
 * @param   skip_factor     how many 5-byte messages to skip before recording the
 *              nth one
 * @param   RX_POINTER      address of input
 * @param   offset
 * @param   limit_status    indicates where in the counting stage we are at
 *
 * @details     status
 */
typedef volatile struct {

    // --------------------------------------

    uint8_t     skip_factor;

    Record_States   limit_status;
    volatile uint8_t    limit;

    // --------------------------------------

    volatile uint8_t*   buffer_pointer;

    volatile uint32_t   isr_counter;
    volatile uint32_t   buffer_counter;

    // --------------------------------------

    volatile RPLiDAR_States current_state;

    // --------------------------------------

} RPLiDAR_Config;



/**
 * @brief
 *
 * @oaram[in] config    pointer to the RPLiDAR_Config struct for configuration
 *
 * @param[in] RX_Data   pointer to the array
 */
void configure_RPLiDAR_struct(
        const RPLiDAR_Config*   input_config,
        const uint8_t*          RX_Data);


/**
 * @brief The EUSCI_A2_UART_Init function initializes the EUSCI_A2 module to use UART
 *      mode.
 *
 * @details This function configures the EUSCI_A2 module to enable UART mode
 *          with the following configuration:
 *
 *      - Parity:       Disabled
 *      - Bit Order:    Least Significant Bit (MSB) first
 *      - Char. Length: 8 data bits
 *      - Stop Bits:    1
 *      - Mode:         UART
 *      - Clock Source: SMCLK
 *      - Baud Rate:    460800
 *
 * For more information regarding the registers used, refer to the eUSCI_A UART Regi-
 *      sters section (24.4) of the MSP432Pxx Microcontrollers Technical Reference
 *      Manual
 *
 * @note This function assumes that the necessary pin configurations for UART communi-
 *      cation have been performed on the corresponding pins. P2.2 is used for UART RX
 *      while P2.3 is used for UART TX.
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


// ----------------------------------------------------------------------------
// 
//  DATA SEND AND TRANSMIT
// 
// ----------------------------------------------------------------------------

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
 * @brief Transmits a single character over UART using the EUSCI_A2 module.
 *
 * This function transmits a single character over UART using the EUSCI_A2 module. It
 *      waits until the UART transmit buffer is ready to accept new data and then writes
 *      the provided data to the transmit buffer for transmission. A character consists
 *      of the following bits:
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



// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Checks if the given data response packet index matches the expected
 *          RPLiDAR data response pattern.
 *
 * @details From page 15 of the "RPLiDAR C1 Interface Protocol and Application
 *              Notes", the expected pattern of a 5-byte response message is:
 *
 *          Byte offset 0:  start flag = bit 0      --> bit 0 = !(bit 1)
 *                          expected patterns: 0b10 or 0b01
 *
 *          Byte offset 1:  check flag = bit 0
 *                          expected pattern:  0b01
 *
 *          To make sure that the pattern-matching is infallible, future uses
 *              of the pattern matching should check the first four sequential
 *              messages.
 * @note    In theory, this pattern-matching should be applied once and only
 *              after the initial matching is done.
 *
 * @param pointer Pointer to the array data buffer.
 *
 * @return uint8_t 1 if the pattern matches, 0 otherwise.
 */
static inline uint8_t pattern(const uint8_t*   msg_ptr);


/**
 * @brief helper function that confirms if the RPLiDAR data is aligned
 * 
 * @param RX_Data 
 * @return uint8_t
 * @retval 1 if aligned, 0 otherwise
 */
uint8_t confirm_aligned(
        const uint8_t RPLiDAR_RX_Data[RPLiDAR_UART_BUFFER_SIZE]);



// ----------------------------------------------------------------------------
// 
//  TRANSMIT AND RECEIVE CHECKING FUNCTIONS
// 
// ----------------------------------------------------------------------------

#ifdef TX_RX_CHECKS

/**
 * @brief The Transmit_UART_Data function transmits data over UART based on the
 *      status of the user buttons.
 *
 * This function transmits different data values over UART based on the status of Button
 *      1 and Button 2. The data transmitted corresponds to the button status according
 *      to the following mapping:
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

#endif /* TX_RX_CHECKS */


#endif /* __INC_RPLIDAR_A2_UART_H__ */
