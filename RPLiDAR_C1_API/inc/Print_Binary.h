/**
 * @file Print_Binary.h
 * @brief Header file for Print_Binary function.
 *
 * This file contains the function prototype for the Print_Binary function,
 * which takes in a uint8_t value and prints the value in binary format.
 * It relies on the EUSCI_A0_UART driver.
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_PRINT_BINARY_H_
#define INC_PRINT_BINARY_H_

#include <stdint.h>
#include "msp.h"
#include "EUSCI_A0_UART.h"

/**
 * @brief Prints the binary representation of an 8-bit value.
 *
 * This function takes an 8-bit unsigned integer as input and prints its binary
 * representation with an underscore (_) separator after the 4th bit. If the input
 * value is 0, it prints "Line Sensor: 0000_0000".
 *
 * @param value_to_convert - The 8-bit value to be converted and printed.
 *
 * @return None
 */
void Print_Binary(uint8_t value_to_convert);

#endif /* INC_PRINT_BINARY_H_ */
