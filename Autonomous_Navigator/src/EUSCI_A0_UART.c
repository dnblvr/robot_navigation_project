/**
 * @file EUSCI_A0_UART.c
 * @brief Source code for the EUSCI_A0_UART driver.
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

#include "../inc/EUSCI_A0_UART.h"

void EUSCI_A0_UART_Init()
{
    // Hold the EUSCI_A0 module in reset mode
    EUSCI_A0->CTLW0 |= 1;

    // Set all the bits of the modulation control register to zero
    EUSCI_A0->MCTLW |= 0;

    // Hold the EUSCI_A0 module in reset mode
    // Set the clock source to SMCLK
    EUSCI_A0->CTLW0 |= 0x00C1;

    // Set the baud rate
    // N = (Clock Frequency) / (Baud Rate) = (12,000,000 / 115,200) = 104.1667
    // Use only the integer part, so N = 104
    EUSCI_A0->BRW |= 104;

    // Configure P1.2 and P1.3 as primary module function
    P1->SEL0 |= 0x0C;
    P1->SEL1 &= ~0x0C;

    // Clear the software reset bit to enable the EUSCI_A0 module
    EUSCI_A0->CTLW0 &= ~1;

    // Ensure that the following interrupts are disabled:
    // - Receive Interrupt
    // - Transmit Interrupt
    // - Start Bit Interrupt
    // - Transmit Complete Interrupt
    EUSCI_A0->IE &= ~0xF;
}

char EUSCI_A0_UART_InChar()
{
    while((EUSCI_A0->IFG&0x01) == 0);

    return((char)(EUSCI_A0->RXBUF));
}

void EUSCI_A0_UART_OutChar(char letter)
{
    while((EUSCI_A0->IFG&0x02) == 0);

    EUSCI_A0->TXBUF = letter;
}

void EUSCI_A0_UART_InString(char *bufPt, uint16_t max)
{
    int length = 0;
    char character = EUSCI_A0_UART_InChar();

    while(character != CR)
    {
        if (character == BS)
        {
            if (length)
            {
                bufPt--;
                length--;
                EUSCI_A0_UART_OutChar(BS);
            }
        }
        else if (length < max)
        {
            *bufPt = character;
            bufPt++;
            length++;
            EUSCI_A0_UART_OutChar(character);
        }
        character = EUSCI_A0_UART_InChar();
    }
    *bufPt = 0;
}

void EUSCI_A0_UART_OutString(char *pt)
{
    while(*pt)
    {
        EUSCI_A0_UART_OutChar(*pt);
        pt++;
    }
}

uint32_t EUSCI_A0_UART_InUDec()
{
    uint32_t number = 0;
    uint32_t length = 0;
    char character = EUSCI_A0_UART_InChar();

    // Accepts until <enter> is typed
    // The next line checks that the input is a digit, i.e. 0-9.
    // If the character is not 0-9, it is ignored and not echoed
    while(character != CR)
    {
        if ((character>='0') && (character<='9'))
        {
            // this line overflows if above 4294967295
            number = 10*number+(character-'0');
            length++;
            EUSCI_A0_UART_OutChar(character);
        }
        // If the input is a backspace, then the return number is
        // changed and a backspace is output to the screen
        else if ((character==BS) && length)
        {
            number /= 10;
            length--;
            EUSCI_A0_UART_OutChar(character);
        }
        character = EUSCI_A0_UART_InChar();
    }
  return number;
}

void EUSCI_A0_UART_OutUDec(uint32_t n)
{
    if (n >= 10)
    {
        EUSCI_A0_UART_OutUDec(n/10);
        n = n%10;
    }
    EUSCI_A0_UART_OutChar(n + '0'); /* n is between 0 and 9 */
}

void EUSCI_A0_UART_OutSDec(int32_t n)
{
    if (n < 0)
    {
        EUSCI_A0_UART_OutChar('-');
        EUSCI_A0_UART_OutUDec(-n);
    }
    else
    {
        EUSCI_A0_UART_OutUDec(n);
    }
}

void EUSCI_A0_UART_OutUFix(uint32_t n)
{
    EUSCI_A0_UART_OutUDec(n/10);
    EUSCI_A0_UART_OutChar('.');
    EUSCI_A0_UART_OutUDec(n%10);
}

uint32_t UART0_InUHex()
{
    uint32_t number = 0;
    uint32_t length = 0;
    uint32_t digit;

    char character = EUSCI_A0_UART_InChar();

    while(character != CR)
    {
        digit = 0x10; // assume bad
        if ((character>='0') && (character<='9'))
        {
            digit = character-'0';
        }
        else if ((character>='A') && (character<='F'))
        {
            digit = (character-'A')+0xA;
        }
        else if ((character>='a') && (character<='f'))
        {
            digit = (character-'a')+0xA;
        }

        // If the character is not 0-9 or A-F, it is ignored and not echoed
        if (digit <= 0xF)
        {
            number = number*0x10+digit;
            length++;
            EUSCI_A0_UART_OutChar(character);
        }
        // Backspace outputted and return value changed if a backspace is inputted
        else if ((character==BS) && length)
        {
            number /= 0x10;
            length--;
            EUSCI_A0_UART_OutChar(character);
        }
    character = EUSCI_A0_UART_InChar();
    }
    return number;
}

void EUSCI_A0_UART_OutUHex(uint32_t number)
{
    // Use recursion to convert the number of
    // unspecified length as an ASCII string
    if (number >= 0x10)
    {
        EUSCI_A0_UART_OutUHex(number/0x10);
        EUSCI_A0_UART_OutUHex(number%0x10);
    }
    else
    {
        if (number < 0xA)
        {
            EUSCI_A0_UART_OutChar(number+'0');
        }
        else
        {
            EUSCI_A0_UART_OutChar((number-0x0A)+'A');
        }
    }
}

int EUSCI_A0_UART_Open(const char *path, unsigned flags, int llv_fd)
{
    EUSCI_A0_UART_Init();
    return 0;
}

int EUSCI_A0_UART_Close(int dev_fd)
{
    return 0;
}
int EUSCI_A0_UART_Read(int dev_fd, char *buf, unsigned count)
{
    char ch;
    // Receive char from the serial terminal
    ch = EUSCI_A0_UART_InChar();
    // Return by reference
    ch = *buf;
    // Output the received char from the serial terminal
    EUSCI_A0_UART_OutChar(ch);
    return 1;
}

int EUSCI_A0_UART_Write(int dev_fd, const char *buf, unsigned count)
{
    unsigned int num = count;

    while(num)
    {
        if(*buf == 10)
        {
            EUSCI_A0_UART_OutChar(13);
        }
        EUSCI_A0_UART_OutChar(*buf);
        buf++;
        num--;
    }
    return count;
}

off_t EUSCI_A0_UART_LSeek(int dev_fd, off_t ioffset, int origin)
{
    return 0;
}

int EUSCI_A0_UART_Unlink(const char * path)
{
    return 0;
}

int EUSCI_A0_UART_Rename(const char *old_name, const char *new_name)
{
    return 0;
}

void EUSCI_A0_UART_Init_Printf()
{
    int ret_val;
    FILE *fptr;

    EUSCI_A0_UART_Init();

    ret_val = add_device("uart", _SSA,
                         EUSCI_A0_UART_Open,
                         EUSCI_A0_UART_Close,
                         EUSCI_A0_UART_Read,
                         EUSCI_A0_UART_Write, EUSCI_A0_UART_LSeek,
                         EUSCI_A0_UART_Unlink,
                         EUSCI_A0_UART_Rename);

    // Return if there is an error
    if (ret_val) return;

    fptr = fopen("uart","w");

    // Return if there is an error
    if (fptr == 0) return;

    // Redirect stdout to UART
    freopen("uart:", "w", stdout);

    // Turn off buffering for stdout
    setvbuf(stdout, NULL, _IONBF, 0);
}
