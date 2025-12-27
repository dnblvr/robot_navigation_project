/**
 * @file EUSCI_B1_I2C.c
 * @brief Source code for the EUSCI_B1_I2C driver.
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

#include "../inc/EUSCI_B1_I2C.h"

void EUSCI_B1_I2C_Init()
{
    // Hold the EUSCI_B1 module in reset mode by setting the
    // UCSWRST bit (Bit 0) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 |= 0x0001;

    // Configure the master and slave device addresses to be 7 bits by clearing the
    // UCA10 and UCSLA10 bits (Bits 15 to 14) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 &= ~0xC000;

    // Configure the EUSCI_B1 module to operate as a single master by clearing the
    // UCMM bit (Bit 13) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 &= ~0x2000;

    // Configure the EUSCI_B1 module to operate in master mode by
    // setting the UCMST bit (Bit 11) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 |= 0x0800;

    // Configure the EUSCI_B1 module to use I2C mode by writing a value of
    // 11b (0x3) in the UCMODEx field (Bits 10 to 9) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 |= 0x0600;

    // Enable synchronous mode for the EUSCI_B1 module by setting
    // the UCSYNC bit (Bit 8) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 |= 0x0100;

    // Select SMCLK as the clock source for the EUSCI_B1 module by writing a value of
    // 11b (0x3) in the UCSSELx field (Bits 7 to 6) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 |= 0x00C0;

    // Configure the EUSCI_B1 module to not transmit the ACK condition in slave mode
    // with enabled address mask by clearing the UCTXACK bit (Bit 5) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 &= ~0x0020;

    // Configure the EUSCI_B1 module to operate in I2C master receiver mode by clearing the
    // UCTR bit (Bit 4) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 &= ~0x0010;

    // Clear the UCTXNACK bit (Bit 3) in the UCBxCTLW0 register since it is used only in slave receiver mode
    EUSCI_B1->CTLW0 &= ~0x0008;

    // Configure the EUSCI_B1 module not to generate the STOP condition in master mode by clearing
    // the UCTXSTP bit (Bit 2) in the UCBxCTLW0 register. Note that in master receiver mode,
    // the STOP condition is preceded by a NACK
    EUSCI_B1->CTLW0 &= ~0x0004;

    // Configure the EUSCI_B1 module to not generate the START condition in master mode by clearing
    // the UCTXSTT bit (Bit 1) in the UCBxCTLW0 register. Note that in master receiver mode,
    // the START condition is preceded by a NACK
    EUSCI_B1->CTLW0 &= ~0x0002;

    // Clear all of the bits in the UCBxCTLW1 register (Bits 8 to 0) since the
    // advanced I2C features will not be used
    EUSCI_B1->CTLW1 &= ~0x01FF;

    // Set the I2C clock prescaler value to 30 to divide the SMCLK clock frequency
    // from 12 MHz to 400 kHz
    // N = (Clock Frequency) / (SCL Frequency) = (12,000,000 / 400,000) = 30
    EUSCI_B1->BRW = 30;

    // Configure the P6.4 (SDA) and P6.5 (SCL)  pins to use the primary module function (I2C)
    // by setting Bits 5 to 4 in the SEL0 register and clearing Bits 5 to 4 in the SEL1 register
    P6->SEL0 |= 0x30;
    P6->SEL1 &= ~0x30;

    // Ensure that all of the I2C interrupts are disabled by clearing
    // Bits 14 to 0 in the UCBxIE register
    EUSCI_B1->IE &= ~0x7FFF;

    // Take the EUSCI_B1 module out of reset mode by clearing the
    // UCSWRST bit (Bit 0) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 &= ~0x0001;
}

void EUSCI_B1_I2C_Send_A_Byte(uint8_t slave_address, uint8_t data)
{
    // Wait until the EUSCI_B1 module is not busy by checking the
    // UCBBUSY bit (Bit 4) in the UCBxSTATw register
    while((EUSCI_B1->STATW & 0x0010) != 0);

    // Assign the slave device's address to the UCBxI2CSA register
    EUSCI_B1->I2CSA = slave_address;

    // Set the UCTR bit (Bit 4) in the UCBxCTLW0 register to configure the EUSCI_B1 module
    // in master transmitter mode. Then, clear the UCTXSTP bit (Bit 2) to not generate the STOP condition
    // Lastly, set the UCTXSTT bit (Bit 1) to generate the START condition
    EUSCI_B1->CTLW0 = (EUSCI_B1->CTLW0 & ~0x0004) | 0x0012;

    // Wait until the transmit interrupt flag is not pending by checking the
    // UCTXIFG0 bit (Bit 1) in the UCBxIFG register
    while((EUSCI_B1->IFG & 0x0002) == 0);

    // Store the 8-bit data in the Transmit Buffer by writing the data
    // to the UCBxTXBUF register
    EUSCI_B1->TXBUF = data;

    // Wait until the transmit interrupt flag is not pending by checking the
    // UCTXIFG0 bit (Bit 1) in the UCBxIFG register
    while((EUSCI_B1->IFG & 0x0002) == 0);

    // Generate the STOP condition by setting the
    // UCTXSTP bit (Bit 2) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 |= 0x0004;

    // Ensure that the transmit interrupt flag is not set by clearing the
    // UCTXIFG0 bit (Bit 1) in the UCBxIFG register
    EUSCI_B1->IFG &= ~0x0002;
}

void EUSCI_B1_I2C_Send_Multiple_Bytes(uint8_t slave_address, uint8_t *data_buffer, uint32_t packet_length)
{
    // Wait until the EUSCI_B1 module is not busy by checking the
    // UCBBUSY bit (Bit 4) in the UCBxSTATw register
    while((EUSCI_B1->STATW & 0x0010) != 0);

    // Assign the slave device's address to the UCBxI2CSA register
    EUSCI_B1->I2CSA = slave_address;

    // Set the UCTR bit (Bit 4) in the UCBxCTLW0 register to configure the EUSCI_B1 module
    // in master transmitter mode. Then, clear the UCTXSTP bit (Bit 2) to not generate the STOP condition
    // Lastly, set the UCTXSTT bit (Bit 1) to generate the START condition
    EUSCI_B1->CTLW0 = (EUSCI_B1->CTLW0 & ~0x0004) | 0x0012;

    // Use a loop to transfer the data individually from the array to the Transmit Buffer
    for (int i = 0; i < packet_length; i++)
    {
        // Wait until the transmit interrupt flag is not pending by checking the
        // UCTXIFG0 bit (Bit 1) in the UCBxIFG register
        while((EUSCI_B1->IFG & 0x0002) == 0);

        // Store the 8-bit data in the Transmit Buffer by writing the data
        // to the UCBxTXBUF register
        EUSCI_B1->TXBUF = data_buffer[i];
    }

    // Wait until the transmit interrupt flag is not pending by checking the
    // UCTXIFG0 bit (Bit 1) in the UCBxIFG register
    while((EUSCI_B1->IFG & 0x0002) == 0);

    // Generate the STOP condition by setting the
    // UCTXSTP bit (Bit 2) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 |= 0x0004;

    // Ensure that the transmit interrupt flag is not set by clearing the
    // UCTXIFG0 bit (Bit 1) in the UCBxIFG register
    EUSCI_B1->IFG &= ~0x0002;
}

uint8_t EUSCI_B1_I2C_Receive_A_Byte(uint8_t slave_address)
{
    // Wait until the EUSCI_B1 module is not busy by checking the
    // UCBBUSY bit (Bit 4) in the UCBxSTATw register
    while((EUSCI_B1->STATW & 0x0010) != 0);

    // Hold the EUSCI_B1 module in reset mode by setting the
    // UCSWRST bit (Bit 0) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 |= 0x0001;

    // Set the byte counter threshold to 1 by writing to the UCTBCNTx field in the UCBxTBCNT register.
    // This register can only be modified when the UCSWRST bit (Bit 0) in the UCBxCTLW0 register is set
    EUSCI_B1->TBCNT |= 0x0001;

    // Take the EUSCI_B1 module out of reset mode by clearing the
    // UCSWRST bit (Bit 0) in the UCBxCTLW0 register
    EUSCI_B1->CTLW0 &= ~0x0001;

    // Assign the slave device's address to the UCBxI2CSA register
    EUSCI_B1->I2CSA = slave_address;

    // Clear the UCTR bit (Bit 4) in the UCBxCTLW0 register to configure the EUSCI_B1 module
    // in master receiver mode. Then, set the UCTXSTP bit (Bit 2) to generate the STOP condition
    // Lastly, set the UCTXSTT bit (Bit 1) to generate the START condition
    EUSCI_B1->CTLW0 = (EUSCI_B1->CTLW0 & ~0x0010) | 0x0006;

    // Wait until the receive interrupt flag is not pending by checking the
    // UCRXIFG0 bit (Bit 0) in the UCBxIFG register
    while((EUSCI_B1->IFG & 0x0001) == 0);

    // Return the received data from the Receive Buffer and ensure that it has a type of uint8_t
    return ((uint8_t)(EUSCI_B1->RXBUF));
}

void EUSCI_B1_I2C_Receive_Multiple_Bytes(uint8_t slave_address, uint8_t *data_buffer, uint16_t packet_length)
{
    // Assign the slave device's address to the UCBxI2CSA register
    EUSCI_B1->I2CSA = slave_address;

    // Clear the UCTR bit (Bit 4) in the UCBxCTLW0 register to configure the EUSCI_B1 module
    // in master receiver mode. Then, set the UCTXSTT bit (Bit 1) to generate the START condition
    EUSCI_B1->CTLW0 = (EUSCI_B1->CTLW0 & ~0x0010) | 0x0002;

    // Use a loop to transfer the data individually from the Receive Buffer to the array
    for (int i = 0; i < packet_length; i++)
    {
        // Check if it is the last byte and then set the UCTXSTP bit (Bit 2) to generate the STOP condition
        if (i == (packet_length - 1))
        {
            EUSCI_B1->CTLW0 |= 0x0004;
        }

        // Wait until the receive interrupt flag is not pending by checking the
        // UCRXIFG0 bit (Bit 0) in the UCBxIFG register
        while((EUSCI_B1->IFG & 0x0001) == 0);

        // Transfer the received data from the Receive Buffer and write it to data_buffer
        data_buffer[i] = EUSCI_B1->RXBUF;
    }

    // Wait until the STOP condition is transmitted by checking the status of the
    // UCTXSTP bit (Bit 2) in the UCBxCTLW0 register
    while((EUSCI_B1->CTLW0 & 0x0004) != 0);
}
