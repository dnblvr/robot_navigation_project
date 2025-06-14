/**
 * @file BLE_A3_UART.c
 * @brief Source code for the BLE_UART driver.
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
 * @author Aaron Nanas, Jeff P, and Gian F
 *
 */

#include <inc/BLE_A3_UART.h>

void BLE_UART_Init()
{
    // Configure pins P9.6 (PM_UCA3RXCD) and P9.7 (UCA3TXD) to use the primary module function
    // by setting Bits 7 and 6 in the SEL0 register for P9
    // and clearing Bits 7 and 6 in the SEL1 register for P9
    P9->SEL0   |=  0xC0;
    P9->SEL1   &= ~0xC0;

    // Configure the P1.6 pin as an output GPIO pin by clearing Bit 6 of the SEL0 and SEL1
    // registers and setting Bit 6 of the DIR register for P1. The P1.6 pin will be connected
    // to the Mode Select (MOD) pin of the Adafruit BLE UART module.
    P1->SEL0   &= ~0x40;
    P1->SEL1   &= ~0x40;
    P1->DIR    |=  0x40;

    // Hold the EUSCI_A3 module in the reset state by setting the UCSWRST bit (Bit 0) in
    // the CTLW0 register
    EUSCI_A3->CTLW0    |=  0x01;

    // --------------------------------------------------------------------------------


    // Clear all of the bits in the Modulation Control Word (MCTLW) register
    EUSCI_A3->MCTLW    &= ~0xFF;

    // Disable the parity bit by clearing the UCPEN bit (Bit 15) in the CTLW0 register
    EUSCI_A3->CTLW0    &= ~0x8000;

    // Select odd parity for the parity bit by clearing the UCPAR bit (Bit 14) in the CTLW0
    // register. Note that the UCPAR bit is not used when the parity bit is disabled. If the
    // parity bit is disabled, then the UCPAR bit does not need to be configured
    EUSCI_A3->CTLW0    &= ~0x4000;


    // Set the bit order to Most Significant Bit (MSB) first by setting the UCMSB bit (Bit 13)
    // in the CTLW0 register
    EUSCI_A3->CTLW0    &= ~0x2000;


    // Select 8-bit character length by clearing the UC7BIT bit (Bit 12) in the CTLW0
    // register
    EUSCI_A3->CTLW0    &= ~0x1000;


    // Select one stop bit by clearing the UCSPB bit (Bit 11) in the CTLW0 register
    EUSCI_A3->CTLW0    &= ~0x0800;


    // Enable UART mode by writing 00b to UCMODEx field (Bits 10 to 9) in the CTLW0 register
    EUSCI_A3->CTLW0    &= ~0x0600;


    // Disable synchronous mode by clearing the UCSYNC bit (Bit 8) in the CTLW0 register
    EUSCI_A3->CTLW0    &= ~0x0100;


    // Configure the EUSCI_A3 module to use SMCLK as the clock source by writing a value
    // of 10b to the UCSSELx field (Bit 7 to 6) in the CTLW0 register
//    EUSCI_A3->CTLW0    |=  0x00C0;

    EUSCI_A3->CTLW0    &= ~0x00C0;
    EUSCI_A3->CTLW0    |=  0x0080;


    // Set the baud rate value by writing to the UCBRx field (Bits 15 to 0) in the BRW register
    // f_clk/baud = 12,000,000/9600 = 1250
    EUSCI_A3->BRW       =  1250;


    // Disable the Transmit Complete Interrupt (UCTXCPTIE) and the Start Bit Interrupt (UCSTTIE)
    // by clearing the Bits 3 and 2 in the IE register
    EUSCI_A3->IE       &= ~0x0C;


    //Enable the Transmit Interupt (UCTXIE) and the Receive Interrupt (UCRXIE) by setting
    // bits 3 and 2 of the IE register
    EUSCI_A3->IE       |=  0x03;


    // --------------------------------------------------------------------------------

    // Release the EUSCI_A3 module from the reset state by clearing the UCSWRST BIT (Bit 0) in
    // the CTLW0 register
    EUSCI_A3->CTLW0    &= ~0x01;
}

uint8_t BLE_UART_InChar()
{
    // Check the Receive Interrupt flag (UCRXIFG, Bit 0)
    // in the IFG register and wait if the flag is not set
    // If the UCRXIFG is set, then the Receive Buffer (UCAXRXBUF) has
    // received a complete character
    while((EUSCI_A3->IFG & 0x01) == 0);

    // Return the data from the Receive Buffer (UCAXRXBUF)
    // Reading the UCAXRXBUF will reset the UCRXIFG flag
    return EUSCI_A3->RXBUF;
}

void BLE_UART_OutChar(uint8_t data)
{
    // Check the Transmit Interrupt flag (UCTXIFG, Bit 1)
    // in the IFG register and wait if the flag is not set
    // If the UCTXIFG is set, then the Transmit Buffer (UCAXTXBUF) is empty
    while((EUSCI_A3->IFG & 0x02) == 0);

    // Write the data to the Transmit Buffer (UCAXTXBUF)
    // Writing to the UCAXTXBUF will clear the UCTXIFG flag
    EUSCI_A3->TXBUF = data;

}

int BLE_UART_InString(char *buffer_pointer, uint16_t buffer_size)
{

    printf("within InString()\n");
    int length      = 0;
    int string_size = 0;

    // Read the last received data from the UART Receive Buffer
    char character = BLE_UART_InChar();



    // MODE 1: BUTTON-BASED COMMANDS --------------------------------------------

    // Check if the received character is a button command. Otherwise, proceed
    // with the rest of the string-based code.
    if (character == '!') {

        *buffer_pointer = character;
        buffer_pointer++;
        string_size = 4;
        length      = 4;

        while (length) {
            *buffer_pointer = BLE_UART_InChar();
            buffer_pointer++;
            length--;
        }

        return string_size;

    }



    // MODE 2: STRING-BASED COMMANDS ---------------------------------------------

    // reset the variables
    length      = 0;
    string_size = 0;

    // Check if the received character is a carriage return. Otherwise,
    // if it is a valid character, then store it in buffer_pointer.
    // For each valid character, increment the string_size variable
    // which will indicate how many characters have been detected from
    // the Barcode Scanner module
    while (character != LF)
    {
        // Remove the character from the buffer if the received character is
        // a backspace character
        if (character == BS)
        {
            if (length)
            {
                buffer_pointer--;
                length--;
                BLE_UART_OutChar(BS);
            }
        }

        // Otherwise, if there are more characters to be read, store them
        // in the buffer
        else if (length < buffer_size)
        {
            *buffer_pointer = character;
            buffer_pointer++;
            length++;
            string_size++;

        }

        character = BLE_UART_InChar();
    }

    *buffer_pointer = 0;

    return string_size;
}



void BLE_UART_OutFixed(int32_t pt)
{
    int i;
    for (i = 24; i >= 0; i = i - 8) {

        char temp = (char)(pt >> i);

        BLE_UART_OutChar(temp);
    }
}


void BLE_UART_OutString(char *pt)
{
    while (*pt)
    {
        BLE_UART_OutChar(*pt);
        pt++;
    }
}


uint8_t Check_BLE_UART_Data(char BLE_UART_Data_Buffer[], char *data_string)
{
    if (strstr(BLE_UART_Data_Buffer, data_string) != NULL)
    {
        return 0x01;
    }
    else
    {
        return 0x00;
    }
}

void BLE_UART_Reset()
{
    // Switch to CMD mode by setting the MOD pin (P1.6) to 1
    P1->OUT |= 0x40;
    Clock_Delay1ms(1000);

    // Send the system reset command by sending the "ATZ" string
    // to the BLE UART module
    BLE_UART_OutString("ATZ\r\n");
    Clock_Delay1ms(3000);

    // Switch back to DATA mode by clearing the MOD pin (P1.6) to 0
    P1->OUT &= ~0x40;
}
