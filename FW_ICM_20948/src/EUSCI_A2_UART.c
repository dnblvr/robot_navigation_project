/**
 * @file EUSCI_A2_UART.c
 * @brief Source code for the UART driver.
 *
 * This file contains the function definitions for the UART driver.
 *
 * It interfaces with the Adafruit Bluefruit LE UART Friend Bluetooth Low
 *      Energy (BLE) module, which uses the UART communication protocol.
 *  - Product Link: https://www.adafruit.com/product/2479
 *
 * The following connections must be made:
 *  - BLE UART TXO  (Pin 3)     <-->  MSP432 LaunchPad Pin P9.6 (PM_UCA3RXD)
 *  - BLE UART RXI  (Pin 4)     <-->  MSP432 LaunchPad Pin P9.7 (PM_UCA3TXD)
 *
 * @note For more information regarding the Enhanced Universal Serial
 *      Communication Interface (eUSCI), refer to the MSP432Pxx Micro- 
 *      controllers Technical Reference Manual
 *
 * @author Gian Fajardo
 *
 */

#include "../inc/EUSCI_A2_UART.h"



// ----------------------------------------------------------------------------
//
//  UART INITIALIZATION
//
// ----------------------------------------------------------------------------


void UART_A2_Init(UART_ISR_Task task)
{

    task_function = task;

    // establish the address of the pointer for later assignment
    UART_BUFFER_ADDR    = &UART_Data_Buffer[0];
    uart_buffer_pointer = &UART_Data_Buffer[0];

    message_length      =  0;


    // Configure pins P3.2 (PM_UCA2RXD) and P3.3 (PM_UCA2TXD) to use the
    // primary module function:
    //    - by  setting  Bits 3 and 2 in the SEL0 register for P3
    //    - and clearing Bits 3 and 2 in the SEL1 register for P3
    P3->SEL0           |=  0x0C;
    P3->SEL1           &= ~0x0C;


    // Hold the EUSCI_A2 module in the reset state by setting the UCSWRST bit
    // (Bit 0) in the CTLW0 register
    EUSCI_A2->CTLW0    |=  0x0001;

    // ------------------------------------------------------------------------


    // Clear all of the bits in the Modulation Control Word (MCTLW) register
    EUSCI_A2->MCTLW    &= ~0x00FF;


    // Disable the parity bit by clearing the UCPEN bit (Bit 15) in the CTLW0
    // register
    EUSCI_A2->CTLW0    &= ~0x8000;


    // Select odd parity for the parity bit by clearing the UCPAR bit (Bit 14)
    // in the CTLW0 register. Note that the UCPAR bit is not used when the
    // parity bit is disabled. If the parity bit is disabled, then the UCPAR
    // bit does not need to be configured
    EUSCI_A2->CTLW0    &= ~0x4000;


    // Set the bit order to Most Significant Bit (MSB) first by setting the
    // UCMSB bit (Bit 13) in the CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x2000;


    // Select 8-bit character length by clearing the UC7BIT bit (Bit 12) in the
    // CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x1000;


    // Select one stop bit by clearing the UCSPB bit (Bit 11) in the CTLW0
    // register
    EUSCI_A2->CTLW0    &= ~0x0800;


    // Enable UART mode by writing 00b to UCMODEx field (Bits 10 to 9) in the
    // CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x0600;


    // Disable synchronous mode by clearing the UCSYNC bit (Bit 8) in the CTLW0
    // register
    EUSCI_A2->CTLW0    &= ~0x0100;


    // Configure the EUSCI_A2 module to use SMCLK as the clock source by
    // writing a value of 10b to the UCSSELx field (Bit 7 to 6) in the CTLW0
    // register
    EUSCI_A2->CTLW0    &= ~0x00C0;
    EUSCI_A2->CTLW0    |=  0x0080;


    // Set the baud rate value by writing to the UCBRx field (Bits 15 to 0) in
    // the BRW register
    // f_clk/baud = 12,000,000/460,800  = 26.0416666667
    EUSCI_A2->BRW       =  26;



    // Disable the following interrupts by clearing the corresponding bits in
    // the IE register:
    // - Transmit Complete Interrupt (UCTXCPTIE, Bit 3)
    // - Start Bit Interrupt         (UCSTTIE,   Bit 2)
    EUSCI_A2->IE       &= ~0x000C;


    // Enable the following interrupts by setting the corresponding bits in the
    // IE register:
    // - Transmit Interrupt (UCTXIE, Bit 1)
    // - Receive Interrupt  (UCRXIE, Bit 0)
    EUSCI_A2->IE       |=  0x0003;


    // ------------------------------------------------------------------------

    // Release the EUSCI_A2 module from the reset state by clearing the UCSWRST
    // BIT (Bit 0) in the CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x0001;



    // ISER[0] = 1 << 18
    // turn on interrupt number 18 using ISER[0]
    NVIC->ISER[0]       =  1 << 18;


    // IP[4] = 0x01 << 21
    // set priority to 1
    NVIC->IP[IP_EUSCIA] =   (NVIC->IP[IP_EUSCIA] & 0x0FFFFFFF)
                          | (0x01 << EUSCIA2_OFFSET);

}


void EUSCIA2_IRQHandler(void) {


    // statement checks if the RXBUF register is full
    // @note statement equiv. to `(EUSCI_A2->IFG & 0x01) == 1)`
    if (EUSCI_A2->IFG & EUSCI_RXIFG) {


        // Read the last received data from the UART Receive Buffer
        // @note RXIFG clears when RXBUF is read
        char character  = (char)EUSCI_A2->RXBUF;


        // BUTTON-BASED COMMANDS ------------------------------------------

        // Check if the received character is a button command from the EUSCI_A2
        if (character == '!') {

            uart_buffer_pointer = UART_BUFFER_ADDR;
            message_length      = 0;

        }

        // otherwise, add the character to the buffer and increment the pointer
        *(uart_buffer_pointer++)    = character;
        message_length++;


        // early return if the message is incomplete
        if (!Check_UART_Data(UART_BUFFER_ADDR, "\r\n"))
//        if (message_length < 4)
            return;


#ifdef DEBUG_OUTPUT
        {
//            int j;
//
//            printf("UART Data: ");
//
//            for (j = 0; j < message_length; j++) {
//                printf("%c", UART_Data_Buffer[j]);
//            }
//
//            printf("\n");
        }
#endif

        // to apply the command only once, clear the buffer memory
        message_length  = 0;


        // to make this function generalizable, make this a function pointer
        (*task_function)(UART_Data_Buffer);

        memset((void*)UART_Data_Buffer, 0, UART_BUFFER_SIZE);
        uart_buffer_pointer = UART_BUFFER_ADDR;

    }

}


uint8_t UART_A2_InChar()
{
    // Check the Receive Interrupt flag (UCRXIFG, Bit 0) in the IFG register
    // and wait if the flag is not set. If the UCRXIFG is set, then the Receive
    // Buffer (UCAXRXBUF) has received a complete character
    while ((EUSCI_A2->IFG & 0x01) == 0);

    // Return the data from the Receive Buffer (UCAXRXBUF). Reading the
    // UCAXRXBUF will reset the UCRXIFG flag
    return EUSCI_A2->RXBUF;
}

void UART_A2_OutChar(uint8_t data)
{
    // Check the Transmit Interrupt flag (UCTXIFG, Bit 1) in the IFG register
    // and wait if the flag is not set. If the UCTXIFG is set, then the
    // Transmit Buffer (UCAXTXBUF) is empty
    while ((EUSCI_A2->IFG & 0x02) == 0);

    // Write the data to the Transmit Buffer (UCAXTXBUF). Writing to the
    // UCAXTXBUF will clear the UCTXIFG flag
    EUSCI_A2->TXBUF = data;

}

int UART_A2_InString(char *buffer_pointer, uint16_t buffer_size)
{

    int length      = 4;
    int string_size = 4;

    // Read the last received data from the UART Receive Buffer
    char character = UART_A2_InChar();



    // MODE 1: BUTTON-BASED COMMANDS ------------------------------------------

    // Check if the received character is a button command from the BLE UART Friend.
    // Otherwise, proceed with the rest of the string-based code.
    if (character == '!') {

        *buffer_pointer     = character;
        buffer_pointer++;

        while (length) {

            *buffer_pointer = UART_A2_InChar();
            buffer_pointer++;
            length--;

        }

        return 4;

    }



    // MODE 2: STRING-BASED COMMANDS ------------------------------------------

    // reset the variables
    length      = 0;
    string_size = 0;

    // Check if the received character is a carriage return. Otherwise, if it
    // is a valid character, then store it in buffer_pointer. For each valid
    // character, increment the string_size variable which will indicate how
    // many characters have been detected from the Barcode Scanner module
    while (character != LF)
    {
        // Remove the character from the buffer if the received character is
        // a backspace character
        if (character == BS) {

            if (length)
            {
                buffer_pointer--;
                length--;
                UART_A2_OutChar(BS);
            }

        // Otherwise, if there are more characters to be read, store them in
        // the buffer
        } else if (length < buffer_size) {

            *buffer_pointer = character;
            buffer_pointer++;
            length++;
            string_size++;

        }

        character = UART_A2_InChar();
    }

    *buffer_pointer = 0;

    return string_size;
}



void UART_A2_OutFixed(int32_t pt)
{
    int i;
    for (i = 24; i >= 0; i = i - 8) {

        char temp = (char)(pt >> i);

        UART_A2_OutChar(temp);
    }
}


void UART_A2_OutString(char *pt)
{
    while (*pt)
    {
        UART_A2_OutChar(*pt);
        pt++;
    }
}

