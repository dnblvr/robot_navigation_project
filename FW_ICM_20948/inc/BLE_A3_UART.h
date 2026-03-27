/**
 * @file BLE_A3_UART.h
 * @brief Header file for the BLE_UART driver.
 *
 * This file contains the function definitions for the BLE_UART driver.
 *
 * It interfaces with the Adafruit Bluefruit LE UART Friend Bluetooth Low
 *      Energy (BLE) module, which uses the UART communication protocol.
 *  - Product Link: https://www.adafruit.com/product/2479
 *
 * The following connections must be made:
 *  - BLE UART MOD  (Pin 1)  <-->   MSP432 LaunchPad Pin P1.6
 *  - BLE UART CTS  (Pin 2)  <-->   MSP432 LaunchPad GND
 *  - BLE UART TXO  (Pin 3)  <-->   MSP432 LaunchPad Pin P9.6 (PM_UCA3RXD)
 *  - BLE UART RXI  (Pin 4)  <-->   MSP432 LaunchPad Pin P9.7 (PM_UCA3TXD)
 *  - BLE UART VIN  (Pin 5)  <-->   MSP432 LaunchPad VCC (3.3V)
 *  - BLE UART RTS  (Pin 6)  <-->   Not Connected
 *  - BLE UART GND  (Pin 7)  <-->   MSP432 LaunchPad GND
 *  - BLE UART DFU  (Pin 8)  <-->   Not Connected
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
 * @brief BLE buffer
 */
static volatile char BLE_UART_Data_Buffer[UART_BUFFER_SIZE] = {0};

/**
 * @brief
 */
volatile int     message_length;

/**
 * @brief pointer to the array base address
 */
static volatile char   *BLE_BUFFER_ADDR    = NULL;

/**
 * @brief pointer to the rolling index address
 */
static volatile char   *BLE_buffer_pointer = NULL;


// ----------------------------------------------------------------------------
//
//  BLE INITIALIZATION
//
// ----------------------------------------------------------------------------


/**
 * @brief   initializes the BLE UART module.
 */
void BLE_UART_Init(UART_ISR_Task task);

/**
 * @brief   receives a character from the BLE UART module.
 *
 * @return uint8_t
 */
uint8_t BLE_UART_InChar();

/**
 * @brief   sends a character to the BLE UART module.
 *
 * @param data character to be sent
 */
void BLE_UART_OutChar(uint8_t data);

/**
 * @brief   receives a string from the BLE UART module.
 *
 * @param buffer_pointer    pointer to the character array where the received
 *                              string will be stored.
 * @param buffer_size       expected size of the character array
 * @return int
 */
int BLE_UART_InString(char *buffer_pointer, uint16_t buffer_size);

/**
 * @brief   sends a string to the BLE UART module.
 *
 * @param pt    pointer to the character array to be sent
 */
void BLE_UART_OutString(char *pt);

/**
 * @brief   sends a fixed-point number to the BLE UART module.
 *
 * @param pt    fixed-point number to be sent
 */
void BLE_UART_OutFixed(int32_t pt);

/**
 * @brief Resets the BLE UART module.
 */
void BLE_UART_Reset();

#endif /* INC_BLE_A3_UART_H_ */
