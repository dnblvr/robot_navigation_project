/*
 * RPLiDAR.c
 *
 *  Created on: Apr 28, 2025
 *      Author: thatg
 */


#include "./inc/RPLiDAR.h"



void Single_Request_Single_Response(uint8_t *output, uint8_t *command) {

    uint8_t response, i, N;
    N = *(command + 1);

    printf("start flag\n");

    EUSCI_A3_UART_OutChar(0xA5);        // start flag
//    EUSCI_A3_UART_OutChar(0x5A);

    printf("\t0x%02X\n",  *(command + 0));

    EUSCI_A3_UART_OutChar( *(command + 0) );        // start flag

    printf("\t0x%02X\n",  *(command + 2));
    EUSCI_A3_UART_OutChar( *(command + 2) );        // start flag


    // catch when it's not 0xA5
    while (EUSCI_A3_UART_InChar() != 0xA5) {
        response = EUSCI_A3_UART_InChar();
        printf("\tresponse = 0x%02X\n", response);
    }

    if (EUSCI_A3_UART_InChar() == 0x5A) {
        for (i = 0; i < N; i++) {
            *(output + i) = EUSCI_A3_UART_InChar();
        }
    }
}
