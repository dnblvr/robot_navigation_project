/**
 * @file EUSCI_B1_I2C.h
 * @brief Header file for the EUSCI_B1_I2C driver.
 *
 * This file contains the function definitions for the EUSCI_B1_I2C driver.
 * The EUSCI_B1_I2C driver uses busy-wait implementation.
 *
 * @note This function assumes that the necessary pin configurations for I2C communication have been performed
 *       on the corresponding pins. The output from the pins will be observed using an oscilloscope.
 *       - P6.4 (SDA)
 *       - P6.5 (SCL)
 *
 * For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_EUSCI_B1_I2C_H_
#define INC_EUSCI_B1_I2C_H_

#include <stdint.h>
#include "msp.h"

/**
 * @brief Initializes the I2C module EUSCI_B1 for communication.
 *
 * This function configures the EUSCI_B1 module to enable I2C communication
 * with the following configuration:
 *
 * - CTWL0 Register Configuration:
 *
 *   Bit(s)      Field       Value       Description
 *   -----       -----       -----       -----------
 *    15         UCA10        0x0        Own address is a 7-bit address
 *    14         UCSLA10      0x0        Address slave has a 7-bit address
 *    13         UCMM         0x0        Single master
 *    12         Reserved     0x0        Reserved
 *    11         UCMST        0x1        Master mode is selected
 *    10-9       UCMODEx      0x3        I2C mode is selected
 *    8          UCSYNC       0x1        Synchronous mode
 *    7-6        UCSSELx      0x3        eUSCI clock source is SMCLK
 *    5          UCTXACK      0x0        Transmit ACK condition in slave mode
 *    4          UCTR         0x0        Transmitter / Receiver
 *    3          UCTXNACK     0x0        Transmit NACK condition in slave mode
 *    2          UCTXSTP      0x0        Transmit STOP condition in master mode
 *    1          UCTXSTT      0x1        Transmit START condition in master mode
 *    0          UCSWRST      0x1        eUSCI logic held in reset state
 *
 * - CTWL1 Register Configuration:
 *
 *   Bit(s)      Field       Value       Description
 *   -----       -----       -----       -----------
 *    15-9       Reserved     0x0        Reserved
 *    8          UCETXINT     0x0        Early UCTXIFG0 flag in slave mode
 *    7-6        UCCLTO       0x0        Disable clock low timeout
 *    5          UCSTPNACK    0x0        Send NACK before STOP condition in master receiver mode
 *    4          UCSWACK      0x0        Address acknowledge of slave is controlled by eUSCI module
 *    3-2        UCASTPx      0x0        No automatic STOP generation in slave mode when UCBCNTIFG is available
 *    1-0        UCGLITx      0x0        Deglitch time of 50 ns
 *
 * For setting the SCL frequency, the clock source used is 12 MHz.
 * To achieve a 400 kHz SCL frequency, the BRW value is set to 30.
 *
 * For more information regarding the registers used, refer to the EUSCI_B I2C Registers section
 * of the MSP432Pxx Microcontrollers Technical Reference Manual.
 *
 * @note This function assumes that the necessary pin configurations for I2C communication have been performed:
 *       - P6.4 (SDA, Serial Data)
 *       - P6.5 (SCL, Serial Clock)
 *
 * @return None
 */
void EUSCI_B1_I2C_Init();

/**
 * @brief Sends a byte of data to a specified I2C slave device using EUSCI_B1 module.
 *
 * This function sends a byte of data to an I2C slave device with the specified
 * slave address using the EUSCI_B1 I2C module. It follows the I2C communication protocol
 * to initiate the transmission, send the data, and terminate the communication.
 *
 * @param slave_address The 7-bit address of the I2C slave device.
 * @param data The data byte to be sent to the slave device.
 *
 * @note Before using this function, ensure that the I2C module (EUSCI_B1) has been properly
 *       initialized using the EUSCI_B1_I2C_Init function.
 *
 * @return None
 */
void EUSCI_B1_I2C_Send_A_Byte(uint8_t slave_address, uint8_t data);

/**
 * @brief Sends multiple bytes of data to a specified I2C slave device using EUSCI_B1 module.
 *
 * This function sends an array of data bytes to an I2C slave device with the specified
 * slave address using the EUSCI_B1 I2C module. It follows the I2C communication protocol
 * to initiate the transmission, send the data, and terminate the communication.
 *
 * @param slave_address The 7-bit address of the I2C slave device.
 * @param data_buffer   A pointer to an array of data bytes to be sent to the slave device.
 * @param packet_length The number of data bytes to send in the data_buffer.
 *
 * @note Before using this function, ensure that the I2C module (EUSCI_B1) has been properly
 *       initialized using the EUSCI_B1_I2C_Init function.
 *
 * @return None
 */
void EUSCI_B1_I2C_Send_Multiple_Bytes(uint8_t slave_address, uint8_t *data_buffer, uint32_t packet_length);

/**
 * @brief Receives a single byte of data from a specified I2C slave device using EUSCI_B1 module.
 *
 * This function receives a single byte of data from an I2C slave device with the specified
 * slave address using the EUSCI_B1 I2C module. It follows the I2C communication protocol
 * to initiate the reception, retrieve the data, and terminate the communication.
 *
 * @param slave_address The 7-bit address of the I2C slave device.
 *
 * @note Before using this function, ensure that the I2C module (EUSCI_B1) has been properly
 *       initialized using the EUSCI_B1_I2C_Init function.
 *
 * @return The received data byte from the slave device.
 */
uint8_t EUSCI_B1_I2C_Receive_A_Byte(uint8_t slave_address);

/**
 * @brief Receives multiple bytes of data from a specified I2C slave device using EUSCI_B1 module.
 *
 * This function receives an array of data bytes from an I2C slave device with the specified
 * slave address using the EUSCI_B1 I2C module. It follows the I2C communication protocol
 * to initiate the reception, retrieve the data, and terminate the communication.
 *
 * @param slave_address The 7-bit address of the I2C slave device.
 * @param data_buffer   A pointer to an array where received data bytes will be stored.
 * @param packet_length The number of data bytes to receive and store in data_buffer.
 *
 * @note Before using this function, ensure that the I2C module (EUSCI_B1) has been properly
 *       initialized using the EUSCI_B1_I2C_Init function.
 *
 * @return None
 */
void EUSCI_B1_I2C_Receive_Multiple_Bytes(uint8_t slave_address, uint8_t *data_buffer, uint16_t packet_length);

#endif /* INC_EUSCI_B1_I2C_H_ */
