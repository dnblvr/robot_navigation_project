
/**
 * @file BLE_UART.h
 * @brief Header file for the BLE_UART driver.
 *
 * This file contains the function definitions for the BLE_UART driver.
 *
 * It interfaces with the Adafruit Bluefruit LE UART Friend Bluetooth Low Energy (BLE) module, which uses the UART communication protocol.
 *  - Product Link: https://www.adafruit.com/product/2479
 *
 * The following connections must be made:
 *  - BLE UART MOD  (Pin 1)     <-->  MSP432 LaunchPad Pin P1.6
 *  - BLE UART CTS  (Pin 2)     <-->  MSP432 LaunchPad GND
 *  - BLE UART TXO  (Pin 3)     <-->  MSP432 LaunchPad Pin P9.6 (PM_UCA3RXD)
 *  - BLE UART RXI  (Pin 4)     <-->  MSP432 LaunchPad Pin P9.7 (PM_UCA3TXD)
 *  - BLE UART VIN  (Pin 5)     <-->  MSP432 LaunchPad VCC (3.3V)
 *  - BLE UART RTS  (Pin 6)     <-->  Not Connected
 *  - BLE UART GND  (Pin 7)     <-->  MSP432 LaunchPad GND
 *  - BLE UART DFU  (Pin 8)     <-->  Not Connected
 *
 * @note For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#ifndef __BLE_INTERFACE_H__
#define __BLE_INTERFACE_H__

#include <stdint.h>
#include <string.h>

#ifndef VSCODE_MODE
#include "msp.h"
#include "Clock.h"
#endif

/**
 * @brief Specifies the size of the buffer used for BLE UART module.
 */
 #define BLE_UART_BUFFER_SIZE 128

/**
 * @brief carriage return character
 */
#define CR 0x0D

/**
 * @brief Line feed character
 */
#define LF 0x0A

/**
 * @brief Backspace character
 */
 #define BS 0x08

/**
 * @brief escape character
 */
 #define ESC 0x1B

/**
 * @brief space character
 */
 #define SP 0x20

/**
 * @brief delete character
 */
 #define DEL 0x7F

/**
 * @brief initializes the BLE UART module.
 * 
 */
void BLE_UART_Init();

/**
 * @brief receives a character from the BLE UART module.
 * 
 * @return uint8_t 
 */
uint8_t BLE_UART_InChar();

/**
 * @brief sends a character to the BLE UART module.
 * 
 * @param data character to be sent
 */
void BLE_UART_OutChar(uint8_t data);

/**
 * @brief receives a string from the BLE UART module.
 * 
 * @param buffer_pointer    pointer to the character array where the received string will be stored
 * @param buffer_size       expected size of the character array
 * @return int 
 */
int BLE_UART_InString(char *buffer_pointer, uint16_t buffer_size);


/**
 * @brief sends a string to the BLE UART module.
 * 
 * @param pt pointer to the character array to be sent
 */
void BLE_UART_OutString(char *pt);



//void BLE_UART_OutFloat(float pt);

void BLE_UART_OutFixed(int32_t pt);


/**
 * @brief 
 * 
 * @param BLE_UART_Data_Buffer 
 * @param data_string 
 * @return uint8_t 
 */
uint8_t Check_BLE_UART_Data(char BLE_UART_Data_Buffer[], char *data_string);

void BLE_UART_Reset();


#endif // __BLE_INTERFACE_H__