/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */



#include "../inc/platform.h"

#include <stdint.h>

#define I2C_RT_DEBUG 1



uint32_t read_byte_counter      = 0;
uint32_t prev_read_byte_counter = 0;



void EUSCIB0_IRQHandler(void) {

    if (EUSCI_B0->IFG &   0x0001) {
        // clear the RXIFG
        EUSCI_B0->IFG &= ~0x0001;

        // perform task here
        read_byte_counter++;

    }
}




void EUSCI_B0_RX_Stop(void) {
    // clear the RXIFG I2C interrupt
    EUSCI_B0->IE &= ~0x0001;

    // clear it in hte nested vector interrupt table
    NVIC->ICER[0] &= ~0x00100000;
}


uint8_t VL53L5CX_RdByte(
    VL53L5CX_Platform *p_platform,

    uint16_t     RegisterAddress,
    uint8_t     *p_value)
{

//    VL53L5CX_wake();

    // Wait until the I²C bus is not busy (UCBBUSY bit cleared)
    while ((EUSCI_BLOCK->STATW & 0x0010) != 0);

#ifdef I2C_RT_DEBUG
    printf("RB a");
#endif

    // Hold the EUSCI_BLOCK module in reset mode
//    EUSCI_BLOCK->CTLW0 |=  0x0001;
//
//    // Set the byte counter threshold to 1 by writing to the UCTBCNTx field in the UCBxTBCNT register.
//    // This register can only be modified when the UCSWRST bit (Bit 0) in the UCBxCTLW0 register is set
//    EUSCI_BLOCK->TBCNT  =  2 + 1;
//
//    // Take the EUSCI_BLOCK module out of reset mode
//    EUSCI_BLOCK->CTLW0 &= ~0x0001;

    // Set the slave address (7-bit, right-justified)
    EUSCI_BLOCK->I2CSA = p_platform->address;

    // --- Phase 1: Write Register Address (Master Transmitter) ---
    EUSCI_BLOCK->CTLW0 = (EUSCI_BLOCK->CTLW0 & ~0x0004) | 0x0012; // UCTR=1, UCTXSTT=1

    // Transmit high byte of RegisterAddress
    EUSCI_BLOCK->TXBUF = (RegisterAddress >> 8) & 0xFF;
    while ((EUSCI_BLOCK->IFG & 0x0002) == 0) {}

#ifdef I2C_RT_DEBUG
    printf("b");
#endif


    // Transmit low byte of RegisterAddress
    EUSCI_BLOCK->TXBUF = RegisterAddress & 0xFF;
    while ((EUSCI_BLOCK->IFG & 0x0002) == 0) {}

#ifdef I2C_RT_DEBUG
    printf("c");
#endif


    // --- Phase 2: Read Data (Master Receiver) ---
    // Clear the UCTR bit (Bit 4) in the UCBxCTLW0 register to configure the EUSCI_BLOCK module
    // in master receiver mode. Then, set the UCTXSTP bit (Bit 2) to generate the STOP condition
    // Lastly, set the UCTXSTT bit (Bit 1) to generate the START condition
    EUSCI_BLOCK->CTLW0  = (EUSCI_BLOCK->CTLW0 & ~0x0010) | 0x0002; // UCTR=0, UCTXSTP=1
    EUSCI_BLOCK->CTLW0 |= 0x0004; // UCTXSTP=1

    // Wait for data reception (UCRXIFG0) or error
//    while ((EUSCI_BLOCK->IFG & 0x0001) == 0) {}
    while (prev_read_byte_counter == read_byte_counter);
    prev_read_byte_counter = read_byte_counter;

#ifdef I2C_RT_DEBUG
    printf("d\n");
#endif

    *p_value = (uint8_t)(EUSCI_BLOCK->RXBUF);


    // checks for TXNACK bit
//    while ((EUSCI_BLOCK->IFG & 0x0020) == 0) {}


    EUSCI_BLOCK->IFG &= ~0x0001;



    return 0;
}


uint8_t VL53L5CX_WrByte(
		VL53L5CX_Platform *p_platform,

		uint16_t    RegisterAddress,
		uint8_t     value)
{
    uint8_t status = 255;

//    VL53L5CX_wake();

    // --- Phase 1: Write Device Address (Master Transmitter) ---

    // Wait until the I²C bus is not busy (b4 UCBBUSY bit cleared)
    while ((EUSCI_BLOCK->STATW & 0x0010) != 0) {}

#ifdef I2C_RT_DEBUG
    printf("WB a");
#endif


    // Set the slave address (7-bit, right-justified in UCBxI2CSA)
    EUSCI_BLOCK->I2CSA     = (p_platform->address) & 0x7F;


    // --- Phase 2: Write Register Address (Master Transmitter) ---

    // Configure as master transmitter (UCTR=1), generate START (UCTXSTT=1)
    EUSCI_BLOCK->CTLW0     = (EUSCI_BLOCK->CTLW0 & ~0x0004) | 0x0012; // UCTR + UCTXSTT


    // Check UCTXIFG0 or errors, then transmit high byte of RegisterAddress
    while ((EUSCI_BLOCK->IFG & 0x0002) == 0) {}
    EUSCI_BLOCK->TXBUF     = (RegisterAddress >> 8) & 0xFF;

#ifdef I2C_RT_DEBUG
    printf("b");
#endif


    // Check UCTXIFG0 or errors, then transmit low byte of RegisterAddress
    while ((EUSCI_BLOCK->IFG & 0x0002) == 0) {}
    EUSCI_BLOCK->TXBUF     =  RegisterAddress & 0xFF;

#ifdef I2C_RT_DEBUG
    printf("c");
#endif


    // --- Phase 3: Write Data (Master Transmitter) ---

    // Wait for data reception (UCRXIFG0) or error
    while ((EUSCI_BLOCK->IFG & 0x0002) == 0) {}
    EUSCI_BLOCK->TXBUF     = value;


    while ((EUSCI_BLOCK->IFG & 0x0002) == 0) {

        // checks for arbitration flag and not-ACK flag
        if (EUSCI_BLOCK->IFG & 0x0030) {
            status = EUSCI_BLOCK->IFG;
            EUSCI_B0_I2C_Init();
            return status;

        }
    }


#ifdef I2C_RT_DEBUG
    printf("d\n");
#endif



    EUSCI_BLOCK->CTLW0    |= 0x0004; // UCTXSTP=1



    // Ensure that the transmit interrupt flag is not set
    EUSCI_BLOCK->IFG &= ~0x0002;


    // return good status
//    VL53L5CX_sleep();

    // Configure the pins by setting P1.(7-6) high so I2C is busy
//    P1->OUT  |=  0xC0;
    return 0;

}


uint8_t VL53L5CX_WrMulti(
		VL53L5CX_Platform *p_platform,

		uint16_t     RegisterAddress,
		uint8_t     *p_values,
		uint32_t     size)
{
    uint8_t counter;

    // --- Phase 1: Write Device Address (Master Transmitter) ---

    // Wait until the I²C bus is not busy (b4 UCBBUSY bit cleared)
    while ((EUSCI_BLOCK->STATW & 0x0010) != 0);


    // Set the slave address (7-bit, right-justified in UCBxI2CSA)
    EUSCI_BLOCK->I2CSA     = (p_platform->address) & 0x7F;



    // --- Phase 2: Write Register Address (Master Transmitter) ---

    // Configure as master transmitter (UCTR=1), generate START (UCTXSTT=1)
    EUSCI_BLOCK->CTLW0     = (EUSCI_BLOCK->CTLW0 & ~0x0004) | 0x0012; // UCTR + UCTXSTT


    // Check UCTXIFG0 or errors, then transmit high byte of RegisterAddress
    while ((EUSCI_BLOCK->IFG & 0x0002) == 0);
    EUSCI_BLOCK->TXBUF     = (RegisterAddress >> 8) & 0xFF;


    // Check UCTXIFG0 or errors, then transmit low byte of RegisterAddress
    while ((EUSCI_BLOCK->IFG & 0x0002) == 0);
    EUSCI_BLOCK->TXBUF     =  RegisterAddress & 0xFF;


    // --- Phase 3: Write Data (Master Transmitter) ---
    for (counter = 0; counter < size; counter++) {

        // Wait for data reception (UCRXIFG0) or error
        while ((EUSCI_BLOCK->IFG & 0x0002) == 0);
        EUSCI_BLOCK->TXBUF     = *(p_values + counter);
    }


    while ((EUSCI_BLOCK->IFG & 0x0002) == 0);


    EUSCI_BLOCK->CTLW0 |= 0x0004; // UCTXSTP=1



    // Ensure that the transmit interrupt flag is not set
    EUSCI_BLOCK->IFG &= ~0x0002;


    // return good status
    return 0;
}


uint8_t VL53L5CX_RdMulti(
		VL53L5CX_Platform *p_platform,

		uint16_t     RegisterAddress,
		uint8_t     *p_values,
		uint32_t     size)
{
	uint8_t status = 255,
	        address;
	
	/* Need to be implemented by customer. This function returns 0 if OK */
	

	return status;
}


uint8_t VL53L5CX_Reset_Sensor(
		VL53L5CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
    VL53L5CX_sleep();
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	VL53L5CX_WaitMs(p_platform, 100);


    EUSCI_B0_I2C_Init();


	/* Set pin LPN of to HIGH */
	VL53L5CX_wake();
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	VL53L5CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L5CX_SwapBuffer(
		uint8_t         *buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for (i = 0; i < size; i = i + 4) {

		tmp =   (buffer[i]     << 24)
              | (buffer[i+1]   << 16)
              | (buffer[i+2]   <<  8)
              | (buffer[i+3]        );
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t VL53L5CX_WaitMs(
		VL53L5CX_Platform *p_platform,

		uint32_t    TimeMs)
{

	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */
	
	SysTick_Wait_ms(TimeMs);
	status = 0;

	return status;
}


