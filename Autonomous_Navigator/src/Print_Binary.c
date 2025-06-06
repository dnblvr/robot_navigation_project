/**
 * @file Print_Binary.c
 * @brief Source code for Print_Binary function implementation.
 *
 * This file contains the function implementation for the Print_Binary function,
 * which takes in a uint8_t value and prints the value in binary format.
 * It relies on the EUSCI_A0_UART driver.
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/Print_Binary.h"

void Print_Binary(uint8_t value_to_convert)
{
    int i;

    if (value_to_convert == 0)
    {
        printf("Line Sensor: 0000_0000\n");
        return;
    }

    printf("Line Sensor: ");

    for (i = 7; i >= 0; i--)
    {
        printf("%d", (value_to_convert >> i) & 1);

        if (i == 4)
        {
            printf("_");
        }
    }

    printf("\n");
}



void print_binary_sequence(
        const uint8_t  *data,
               size_t   length)
{
    size_t i;
    for (i = 0; i < length; ++i) {
        printf("%02X ", data[i]);
    }

    printf("\n\n");
}

