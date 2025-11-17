/**
 * @file RPLIDAR_A2_UART.h
 * @brief Header file for the RPLIDAR_A2_UART driver.
 *
 * This file contains the function definitions for the EUSCI_A2_UART driver.
 *
 * @note Assumes that the necessary pin configurations for UART communication have been
 *      performed on the corresponding pins. P3.2 is used for UART RX while P3.3 is used
 *      for UART TX.
 *
 * @note For more information regarding the Enhanced Universal Serial Communication In-
 *      terface (eUSCI), refer to the MSP432Pxx Microcontrollers Technical Reference
 *      Manual
 *
 * @author Gian Fajardo
 *
 */

#ifndef INC_RPLIDAR_A2_UART_H_
#define INC_RPLIDAR_A2_UART_H_

#include <stdint.h>
#include "msp.h"
#include "Clock.h"
#include "coordinate_transform.h"


//#define RPLIDAR_DEBUG 1
// #define TX_RX_CHECKS 1
// #define ERROR_CHECKING 1



/**
 * @brief Buffer length for UART communication.
 */
#define OUTPUT_BUFFER               75
#define MSG_LENGTH                   5
#define RPLiDAR_UART_BUFFER_SIZE    OUTPUT_BUFFER*MSG_LENGTH
#define MSG_LENGTH                   5
#define RPLiDAR_UART_BUFFER_SIZE    OUTPUT_BUFFER*MSG_LENGTH


/**
 * @brief states of the UART record system
 */
typedef enum {

    HOLD            = 0,
    FIND_PATTERN,
    ADD_OFFSET,
    SKIP,
    RECORD

} Record_States;


/**
 * @brief RPLiDAR states
 */
typedef enum {

    IDLING      = 0,
    READY,
    RECORDING,
    PROCESSING
    RECORD

} Record_States;

typedef enum {
    IDLING,
    RECORDING,
    PROCESSING
} RPLiDAR_States;


/**
 * @brief configuration struct of the RPLiDAR C1
 *
 * @param   skip_factor how many 5-byte messages to skip before recording the
 *              nth one
 * @param   RX_POINTER  address of input
 * @param   offset
 * @param   limit_status  indicates where in the counting stage we are at
 *
 * @details     more details in Record_States
 */
typedef struct {

    // --------------------------------------


    // --------------------------------------

    uint8_t     skip_factor;

    uint8_t     wait_index,
                skip_index,
    uint8_t     skip_index,
                find_index;

    uint8_t    *RX_POINTER;

    Record_States   limit_status;
    uint8_t         limit;
    Record_States   limit_status;
    uint8_t         limit;

    // --------------------------------------
    // --------------------------------------

    volatile uint8_t   *buffer_pointer;

    volatile uint32_t   isr_counter;
    volatile uint32_t   buffer_counter;

    // --------------------------------------
    volatile uint32_t   buffer_counter;

    // --------------------------------------

    uint8_t     process_data;
    uint8_t     record_data;

    volatile RPLiDAR_States  current_state;

    // --------------------------------------
    uint8_t     record_data;

    RPLiDAR_States current_state;

    // --------------------------------------

} RPLiDAR_Config;


/**
 * @brief local instance of the configuration struct which links to the outside world
 */
RPLiDAR_Config *config;


/**
 * @brief
 */
void configure_RPLiDAR_struct(
        RPLiDAR_Config *input_config,
        uint8_t        *RX_Data);


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
 * @oaram[in] config    pointer to the RPLiDAR_Config struct for configuration
 *
 * @param[in] RX_Data   pointer to the array
 *
 * @return None
 */
void EUSCI_A2_UART_Init();
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
// 
//  DATA SEND AND TRANSMIT
// 
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



// -------------------------------------------------------------------------------------
// 
//  HIGHER-LEVEL RPLiDAR FUNCTIONS
// 
// -------------------------------------------------------------------------------------



/**
 * @brief Checks if the given data response packet index matches the expected RPLiDAR
 *              data response pattern.
 *
 * @details From page 15 of the "RPLiDAR C1 Interface Protocol and Application Notes",
 *              the expected pattern of a 5-byte response message is:
 *
 *          Byte offset 0:      start flag = bit 0      --> bit 0 = !(bit 1)
 *                              expected patterns: 0b10 or 0b01
 *
 *          Byte offset 1:      check flag = bit 0
 *                              expected pattern:  0b01
 *
 *          To make sure that the pattern-matching is infallible, future uses of
 *              the pattern matching should check the first four sequential messages.
 *
 * @note    In theory, this pattern-matching should be applied once and only after
 *              the initial matching is done.
 *
 * @param pointer Pointer to the array data buffer.
 *
 * @return uint8_t 1 if the pattern matches, 0 otherwise.
 */
inline uint8_t pattern(
        uint8_t    *pointer);


/**
 * @brief Converts the raw data from the RPLiDAR to angle and distance values.
 *
 * @param base_ptr       Pointer to the raw data buffer.
 * @param distance_angle Array to store the converted distance and angle values.
 */
uint8_t to_angle_distance(
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
              uint8_t RX_DATA_BUFFER[RPLiDAR_UART_BUFFER_SIZE]);


/**
 * @brief Sends a command to the RPLiDAR and waits for a single response.
 *
 * @param command The command to send.
 * @param RX_DATA_BUFFER The buffer to store the received response.
 */
void Single_Request_Multiple_Response(
        const uint8_t        command[2],
              uint8_t RX_DATA_BUFFER[RPLiDAR_UART_BUFFER_SIZE]);



/**
 * @brief       Get data from the RPLiDAR C1
 *
 * @param scan_confirmation Confirmation flag for the scan
 * @return None
 */
void Gather_LiDAR_Data(
        RPLiDAR_Config       *cfg,

        uint8_t scan_confirmation,
        uint8_t           RX_Data[RPLiDAR_UART_BUFFER_SIZE],

        float                 out[OUTPUT_BUFFER][3]);


// -------------------------------------------------------------------------------------
// 
//  TRANSMIT AND RECEIVE CHECKING FUNCTIONS
// 
// -------------------------------------------------------------------------------------

#ifdef TX_RX_CHECKS

/**
 * @brief The Transmit_UART_Data function transmits data over UART based on the status
 *      of the user buttons.
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

#endif // TX_RX_CHECKS




#endif /* INC_EUSCI_A2_UART_H_ */
