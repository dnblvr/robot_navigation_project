/**
 * @file EUSCI_A0_UART.h
 * @brief Header file for the EUSCI_A0_UART driver.
 *
 * This file contains the function definitions for the EUSCI_A0_UART driver.
 *
 * For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @note The pins P1.2 and P1.3 are used for UART communication via USB.
 *
 * @author Aaron Nanas
 *
 */

#ifndef EUSCI_A0_UART_H_
#define EUSCI_A0_UART_H_

#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "file.h"

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
 * @brief escape character
 */
#define ESC  0x1B
/**
 * @brief space character
 */
#define SP   0x20
/**
 * @brief delete character
 */
#define DEL  0x7F

/**
 * @brief Initializes the UART module EUSCI_A0 for communication.
 *
 * This function configures the EUSCI_A0 module to enable UART communication
 * with the following configuration:
 *
 * - Parity: Disabled
 * - Stop bits: 1
 * - Data bits: 8
 * - Baud rate: 115200
 * - Mode: UART
 * - LSB first
 * - UART clock source: SMCLK
 * - Interrupts disabled
 *
 * For more information regarding the registers used, refer to the eUSCI_A UART Registers section (24.4)
 * of the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @note Pins P1.2 and P1.3 are used for UART communication via USB.
 *
 * @return None
 */
void EUSCI_A0_UART_Init();

/**
 * @brief The EUSCI_A0_UART_InChar function reads a character from the UART receive buffer.
 *
 * This function waits until a character is available in the UART receive buffer (EUSCI_A0)
 * from the serial terminal input and returns the received character as a char type.
 *
 * @param None
 *
 * @return The received character from the serial terminal as a char type.
 */
char EUSCI_A0_UART_InChar();

/**
 * @brief The EUSCI_A0_UART_OutChar function transmits a character via UART to the serial terminal.
 *
 * This function waits until the UART transmit buffer (EUSCI_A0) is ready to accept
 * a new character and then writes the specified character in the transmit buffer to the serial terminal.
 *
 * @param letter The character to be transmitted to the serial terminal.
 *
 * @return None
 */
void EUSCI_A0_UART_OutChar(char letter);

/**
 * @brief The EUSCI_A0_UART_InString function reads a string from the UART receive buffer.
 *
 * This function reads characters from the UART receive buffer (EUSCI_A0) in the serial terminal
 * until a carriage return (CR) character is encountered.
 * The characters are stored in the provided buffer (bufPt) up to the specified maximum length (max).
 * The function supports backspace (BS) character for deleting characters from the buffer.
 *
 * @param bufPt Pointer to the buffer where the received string will be stored.
 * @param max Maximum length of the buffer.
 *
 * @return None
 */
void EUSCI_A0_UART_InString(char *bufPt, uint16_t max);

/**
 * @brief The EUSCI_A0_UART_OutString function transmits a null-terminated string via UART to the serial terminal.
 *
 * This function transmits characters from the provided string (pt) via UART to the serial terminal
 * until a null character is encountered.
 *
 * @param pt Pointer to the null-terminated string to be transmitted.
 *
 * @return None
 */
void EUSCI_A0_UART_OutString(char *pt);

/**
 * @brief The EUSCI_A0_UART_InUDec function reads an unsigned decimal number from the UART receive buffer.
 *
 * This function reads characters from the UART receive buffer (EUSCI_A0) in the serial terminal
 * until a carriage return (CR) character is encountered.
 * It converts the received characters into an unsigned decimal number and returns the result.
 * The function supports backspace (BS) character for deleting digits from the number during input.
 *
 * @param None
 *
 * @return The received unsigned decimal number from the serial terminal as a uint32_t type.
 */
uint32_t EUSCI_A0_UART_InUDec();

/**
 * @brief The EUSCI_A0_UART_OutUDec function transmits an unsigned decimal number via UART to the serial terminal.
 *
 * This function transmits the provided unsigned decimal number (n) via UART to the serial terminal.
 *
 * @param n The unsigned decimal number to be transmitted to the serial terminal.
 *
 * @return None
 */
void EUSCI_A0_UART_OutUDec(uint32_t n);

/**
 * @brief The EUSCI_A0_UART_OutSDec function transmits a signed decimal number via UART to the serial terminal.
 *
 * This function transmits the provided signed decimal number (n) via UART to the serial terminal.
 * If the number is negative, a minus sign '-' is transmitted first.
 *
 * @param n The signed decimal number to be transmitted to the serial terminal.
 *
 * @return None
 */
void EUSCI_A0_UART_OutSDec(int32_t n);

/**
 * @brief The EUSCI_A0_UART_OutUFix function transmits an unsigned fixed-point number via UART to the serial terminal.
 *
 * This function transmits the provided unsigned fixed-point number (n) via UART to the serial terminal.
 * The number is assumed to have one decimal place, and a period '.' is transmitted before the fractional part.
 *
 * @param n The unsigned fixed-point number to be transmitted to the serial terminal.
 *
 * @return None
 */
void EUSCI_A0_UART_OutUFix(uint32_t n);

/**
 * @brief The UART0_InUHex function reads an unsigned hexadecimal number from the UART receive buffer.
 *
 * This function reads characters from the UART receive buffer (EUSCI_A0) in the serial terminal
 * until a carriage return (CR) character is encountered.
 * It converts the received characters into an unsigned hexadecimal number and returns the result.
 * The function supports backspace (BS) character for deleting digits from the number during input.
 * The input characters 'A' to 'F' (uppercase or lowercase) are considered valid hexadecimal digits.
 *
 * @param None
 *
 * @return The received unsigned hexadecimal number from teh serial terminal as a uint32_t type.
 */
uint32_t UART0_InUHex();

/**
 * @brief The EUSCI_A0_UART_OutUHex function transmits an unsigned hexadecimal number via UART to the serial terminal.
 *
 * This function transmits the provided unsigned hexadecimal number (number) via UART to the serial terminal.
 * The number is converted into a hexadecimal ASCII string and transmitted character by character.
 *
 * @param number The unsigned hexadecimal number to be transmitted to the serial terminal.
 *
 * @return None
 */
void EUSCI_A0_UART_OutUHex(uint32_t number);

/**
 * @brief The EUSCI_A0_UART_Open function initializes the UART communication.
 *
 * This function initializes the UART module (EUSCI_A0) for communication.
 * It is called when opening a UART device.
 *
 * @param path Path to the UART device.
 * @param flags Flags for opening the device.
 * @param llv_fd Low-level virtual file descriptor.
 *
 * @return 0 if successful, otherwise an error code.
 */
int EUSCI_A0_UART_Open(const char *path, unsigned flags, int llv_fd);

/**
 * @brief The EUSCI_A0_UART_Close function closes the UART communication.
 *
 * This function closes the UART module (EUSCI_A0) for communication.
 * It is called when closing a UART device.
 *
 * @param dev_fd Device file descriptor.
 *
 * @return 0 if successful, otherwise an error code.
 */
int EUSCI_A0_UART_Close(int dev_fd);

/**
 * @brief The EUSCI_A0_UART_Read function reads data from the UART receive buffer.
 *
 * This function reads data from the UART receive buffer (EUSCI_A0) and stores it in the provided buffer (buf).
 * It reads a single character at a time and echoes it back to the UART for display.
 *
 * @param dev_fd Device file descriptor.
 * @param buf Pointer to the buffer where the read data will be stored.
 * @param count Number of bytes to read.
 *
 * @return Number of bytes read.
 */
int EUSCI_A0_UART_Read(int dev_fd, char *buf, unsigned count);

/**
 * @brief The EUSCI_A0_UART_Write function writes data to the UART transmit buffer.
 *
 * This function writes data from the provided buffer (buf) to the UART transmit buffer (EUSCI_A0) for transmission.
 * It transmits each character one by one and handles newline character ('\n') by sending a carriage return ('\r') first.
 *
 * @param dev_fd Device file descriptor.
 * @param buf Pointer to the buffer containing the data to be transmitted.
 * @param count Number of bytes to write.
 *
 * @return Number of bytes written.
 */
int EUSCI_A0_UART_Write(int dev_fd, const char *buf, unsigned count);

/**
 * @brief The EUSCI_A0_UART_LSeek function changes the read/write position in the UART device.
 *
 * This function is not supported for UART devices and always returns 0.
 *
 * @param dev_fd Device file descriptor.
 * @param ioffset Offset value.
 * @param origin Origin position.
 *
 * @return Always returns 0.
 */
off_t EUSCI_A0_UART_LSeek(int dev_fd, off_t ioffset, int origin);

/**
 * @brief The EUSCI_A0_UART_Unlink function is not supported for UART devices.
 *
 * This function is not supported for UART devices and always returns 0.
 *
 * @param path Path to the UART device.
 *
 * @return Always returns 0.
 */
int EUSCI_A0_UART_Unlink(const char * path);

/**
 * @brief The EUSCI_A0_UART_Rename function is not supported for UART devices.
 *
 * This function is not supported for UART devices and always returns 0.
 *
 * @param old_name Old name of the UART device.
 * @param new_name New name of the UART device.
 *
 * @return Always returns 0.
 */
int EUSCI_A0_UART_Rename(const char *old_name, const char *new_name);

/**
 * @brief The EUSCI_A0_UART_Init_Printf function initializes the UART communication for printf output.
 *
 * This function initializes the UART module (EUSCI_A0) for communication and configures it for printf output.
 * It adds the UART device to the device list, sets stdout to use the UART output, and turns off buffering for stdout.
 *
 * @param None
 *
 * @return None
 */
void EUSCI_A0_UART_Init_Printf();

#endif /* EUSCI_A0_UART_H_ */
