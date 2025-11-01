/**
 * @file RPLIDAR_A2_UART.c
 * @brief Source code for the RPLIDAR_A2_UART driver.
 *
 * This file contains the function definitions for the EUSCI_A2_UART driver.
 *
 * @note Assumes that the necessary pin configurations for UART communication have been
 *      performed on the corresponding pins. P3.2 is used for UART RX while P3.3 is used
 *      for UART TX.
 *
 * @note For more information regarding the Enhanced Universal Serial Communication
 *      Interface (eUSCI), refer to the MSP432Pxx Microcontrollers Technical Reference
 *      Manual
 *
 *
 * @author Gian Fajardo
 *
 */

#include "inc/RPLiDAR_A2_UART.h"


RPLiDAR_Config *config = NULL;


void configure_RPLiDAR_struct(
        RPLiDAR_Config *input_config,
        uint8_t        *RX_Data)
{

    // link the RPLiDAR_Config struct to the outside
    config  = input_config;


    // Assign current buffer position and the absolute position of the array
    // to the pointer.
    config->     RX_POINTER = RX_Data;
    config-> buffer_pointer = RX_Data;


    // Generate the counter variables and the start and stop indices of the nth
    // message.
    config->    isr_counter = 0;
    config-> buffer_counter = 0;
    config->   wait_index   = MSG_LENGTH*12;
    config->   find_index   = MSG_LENGTH* 4;
    config->   skip_index   = MSG_LENGTH*config->skip_factor;

    config->  limit_status  = HOLD;
    config->current_state   = IDLING;
//    printf("a\n");


    printf(" 0 -> %2d -> %2d\n", config->skip_index, config->skip_index + 5);

}


void EUSCI_A2_UART_Init()
{

    // Configure pins P3.2 (PM_UCA2RXD) and P3.3 (PM_UCA2TXD) to use the
    // primary module function:
    //    - by  setting  Bits 3 and 2 in the SEL0 register for P3
    //    - and clearing Bits 3 and 2 in the SEL1 register for P3
    P3->SEL0 |=  0x0C;
    P3->SEL1 &= ~0x0C;


    // Hold the EUSCI_A2 module in the reset state by setting the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A2->CTLW0    |=  0x01;


    // Clear all of the bits in the Modulation Control Word (MCTLW) register
    EUSCI_A2->MCTLW    &= ~0xFF;


    // Disable the parity bit by clearing the UCPEN bit (Bit 15) in the CTLW0
    // register
    EUSCI_A2->CTLW0    &= ~0x8000;


    // Select odd parity for the parity bit by clearing the UCPAR bit (Bit 14)
    // in the CTLW0 register
    // @note the UCPAR bit is not used when parity is disabled
    EUSCI_A2->CTLW0    &= ~0x4000;


    // Set the bit order to Most Significant Bit (MSB) first by setting the
    // UCMSB bit (Bit 13) in the CTLW0 register
    // EUSCI_A2->CTLW0    |=  0x2000;

    // Set the bit order to Least Significant Bit (MSB) first by clearing the
    // UCMSB bit (Bit 13) in the CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x2000;


    // Select 8-bit character length by clearing the UC7BIT bit (Bit 12) in the
    // CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x1000;


    // Select one stop bit by clearing the UCSPB bit (Bit 11) in the CTLW0
    // register
    EUSCI_A2->CTLW0    &= ~0x0800;


    // Enable UART mode by writing 00b to the UCMODEx field (Bits 10-9) in the
    // CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x0600;


    // Disable synchronous mode by clearing the UCSYNC bit (Bit 8) in the CTLW0
    // register
    EUSCI_A2->CTLW0    &= ~0x0100;


    // Configure the EUSCI_A2 module to use SMCLK as the clock source by
    // writing a value of 10b to the UCSSELx field (Bits 7 to 6) in the CTLW0 register
//    EUSCI_A2->CTLW0    |=  0x00C0;
    EUSCI_A2->CTLW0    &= ~0x00C0;
    EUSCI_A2->CTLW0    |=  0x0080;


    // Set the baud rate value by writing to the UCBRx field (Bits 15 to 0)
    // in the BRW register
    // - N = (Clock Frequency) / (Baud Rate) = (12,000,000 / 460,800)
    //     = 26.0416666667
    // Use only the integer part, so N = 26
    EUSCI_A2->BRW       =  26;  // 460,800


    // Disable the following interrupts by clearing the
    // corresponding bits in the IE register:
    // - Transmit Complete Interrupt (UCTXCPTIE, Bit 3)
    // - Start Bit Interrupt         (UCSTTIE,   Bit 2)
    EUSCI_A2->IE       &= ~0x0C;


    // Enable the following interrupts by setting the
    // corresponding bits in the IE register
    // - Transmit Interrupt (UCTXIE, Bit 1)
    // - Receive Interrupt  (UCRXIE, Bit 0)
    EUSCI_A2->IE       |=  0x03;


    // Release the EUSCI_A2 module from the reset state by clearing the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x01;


    // ISER[0] = 1 << 18
    // turn on interrupt number 18 using ISER[0]
    NVIC->ISER[0] =  0x00040000;


    // IP[4] = 0x02 << 21
    // set priority to 2
    NVIC->IP[4]     = (NVIC->IP[4] & 0xFF0FFFFF) | 0x00400000;

}

void EUSCI_A2_UART_Stop() {

    // Hold the EUSCI_A2 module in the reset state by setting the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A2->CTLW0    |=  0x01;
    

    // Disable the following interrupts by clearing the
    // corresponding bits in the IE register
    // - Transmit Interrupt (UCTXIE, Bit 1)
    // - Receive Interrupt  (UCRXIE, Bit 0)
    EUSCI_A2->IE       &= ~0x03;


    // Release the EUSCI_A2 module from the reset state by clearing the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x01;

}

void EUSCI_A2_UART_Restart() {

    // Hold the EUSCI_A2 module in the reset state by setting the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A2->CTLW0    |=  0x01;

    // Enable the following interrupts by setting the
    // corresponding bits in the IE register
    // - Transmit Interrupt (UCTXIE, Bit 1)
    // - Receive Interrupt  (UCRXIE, Bit 0)
    EUSCI_A2->IE       |=  0x03;

    // Release the EUSCI_A2 module from the reset state by clearing the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A2->CTLW0    &= ~0x01;

}


// ----------------------------------------------------------------------------
//
//  DATA SEND AND TRANSMIT
//
// ----------------------------------------------------------------------------


uint8_t EUSCI_A2_UART_InChar()
{

    // Check the Receive Interrupt flag (UCRXIFG, Bit 0)
    // in the IFG register and wait if the flag is not set
    // If the UCRXIFG is set, then the Receive Buffer (UCAxRXBUF) has
    // received a complete character
    while((EUSCI_A2->IFG & 0x01) == 0);


    // Return the data from the Receive Buffer (UCAxRXBUF)
    // Reading the UCAxRXBUF will reset the UCRXIFG flag
    return EUSCI_A2->RXBUF;

}


void EUSCI_A2_UART_OutChar(uint8_t data)
{

    // Check the Transmit Interrupt flag (UCTXIFG, Bit 1)
    // in the IFG register and wait if the flag is not set
    // If the UCTXIFG is set, then the Transmit Buffer (UCAxTXBUF) is empty
    while((EUSCI_A2->IFG & 0x02) == 0);


    // Write the data to the Transmit Buffer (UCAxTXBUF)
    // Writing to the UCAxTXBUF will clear the UCTXIFG flag
    EUSCI_A2->TXBUF = data;

}


void EUSCIA2_IRQHandler(void) {


    /**
     * @todo: figure out what to do in the HOLD state:
     *  If the counting system is a trustless system (has to find the RPLiDAR C1
     *      message pattern for every single iteration), then to make it a
     *      trusted system I have to prioritize a few things:
     * 
     *  1. minimize the overhead in the UART ISR by:
     *      a. using memcpy() to copy the intended struct properties
     *          at each stage...
     *      b. ... or just plainly assign struct to struct...
     * 
     *      because the states are also likely to be *very* predictable.
     * 
     *  2. and take the hold state to SKIP (?).
     *  3. make sure that isr_counter counts in multiples of MSG_LENGTH
     */

    uint8_t data;


    // if EUSCI_A2 RXIFG flag is read, then do the following commands
    if (EUSCI_A2->IFG & 0x01) {


        /**
         *  the code is made such that it ignores recording certain messages
         *  above a skip point
         *  @note this should be recorded first and foremost!
         */
        data = (uint8_t)EUSCI_A2->RXBUF;


        /**
         * Add to the buffer if in the desired states, and increment
         *  buffer_counter.
         *  - FIND_PATTERN: 0b001
         *  - RECORD:       0b100
         */
        if (config->limit_status & (FIND_PATTERN | RECORD)) {

            // assign `data` and increment pointer
          *(config->buffer_pointer++)   = data;
            config->buffer_counter++;

        }

        // increment isr_counter
        config->isr_counter++;


        /**
         * If isr_counter does arrive at the limit, achieve the following:
         *  1. transition to the next limit_status
         *  2. change limits based on limit_status
         */
        if (config->isr_counter >= config->limit) {

            // reset the counter
            config->isr_counter = 0;


            // switch statement uses jump tables which take up the least
            // overhead which is perfect for this application
            switch(config->limit_status) {


            case HOLD: {  // --------------------------------------------------

                if (config->current_state == READY) {

                    config->current_state   = RECORDING;
                    config->  limit_status  = FIND_PATTERN;
                    config->  limit         = config->find_index;
                    config-> buffer_pointer = config->RX_POINTER;

//                    printf("c\n");
                }

                break;

            }


            case FIND_PATTERN: {  // ------------------------------------------

                int     i;
                uint8_t found;

                for (i = 0; i < (MSG_LENGTH + 1); i++) {

                    found =    pattern(config->RX_POINTER + MSG_LENGTH*0 + i)
                            && pattern(config->RX_POINTER + MSG_LENGTH*1 + i)
                            && pattern(config->RX_POINTER + MSG_LENGTH*2 + i);

                    if (found) {

                        // "MSG_LENGTH - i" is the amount needed to re-align to
                        //  the *next* first byte of the message
                        config->limit_status    = ADD_OFFSET;
                        config->limit           = MSG_LENGTH - i;

                        break;

                    }

                }

                // restart to fill whole array regardless of the outcome
                config->buffer_pointer  = config->RX_POINTER;
                config->buffer_counter  = 0;

                break;

            }


            case ADD_OFFSET: {  // --------------------------------------------

                config->limit_status    = SKIP;
                config->limit           = config->skip_index;

                break;

            }


            /**
             * @note after the ADD_OFFSET stage, it trades between SKIP and
             *      RECORD until
             */

            case SKIP: {  // --------------------------------------------------

                if (config->buffer_counter >= RPLiDAR_UART_BUFFER_SIZE) {

                    config->current_state   = PROCESSING;
                    config->limit_status    = HOLD;
                    config->limit           = config->find_index;
                    config->buffer_pointer  = config->RX_POINTER;
                    config->buffer_counter  = 0;

//                    printf("e1\n");

                } else {

                    // up-/left-shift to RECORD
                    // config->limit_status    = RECORD;
                    config->limit_status   += 1;
                    config->limit           = MSG_LENGTH;

//                    printf("d1\n");

                }

                break;

            }


            case RECORD: {  // ------------------------------------------------

                if (config->buffer_counter >= RPLiDAR_UART_BUFFER_SIZE) {

                    config->current_state   = PROCESSING;
                    config->limit_status    = HOLD;
                    config->limit           = config->wait_index;
                    config->buffer_pointer  = config->RX_POINTER;
                    config->buffer_counter  = 0;

//                    printf("e2\n");

                } else {

                    // down-/right-shift to SKIP
                    // config->limit_status    = SKIP;
                    config->limit_status   -= 1;
                    config->limit           = config->skip_index;

//                    printf("d2\n");
                }

                break;

            }


            default: {  // ----------------------------------------------------

                config->limit_status    = HOLD;
                config->record_data     = 0;
                break;

            }


            } // end switch(config->limit_status)  // -------------------------

        }

    }

}



// ----------------------------------------------------------------------------
//
//  HIGHER-LEVEL RPLiDAR FUNCTIONS
//
// ----------------------------------------------------------------------------


inline uint8_t pattern(
        uint8_t *pointer)
{
    // checks byte 0 for pattern 0x2 or 0x1 (0b10 or 0b01) ...
    return      ( ((*pointer & 0x3) == 0x2) || ((*pointer & 0x3) == 0x1) )

            // ... and pattern 0x10
            &&  ( (*(pointer + 1) & 0x1) == 0x1 );
}


uint8_t to_angle_distance(
        uint8_t    *base_ptr,
        float       distance_angle[2])
{

#ifdef ERROR_CHECKING
    uint8_t start       = (*base_ptr)      & 0x01,
            start_inv   = (*base_ptr) >> 1 & 0x01;

    if (start != start_inv) return;
#endif

    uint16_t distance, angle;

    angle       = ( *(base_ptr + 2) << 7 ) | ( *(base_ptr + 1) >> 1 );
    distance    = ( *(base_ptr + 4) << 8 ) | ( *(base_ptr + 3)      );

    if (distance < 0x01) {
        return 0;
    }

    // If the above condition does not hold, then calculate the float
    // values
    distance_angle[0]   = (float)(distance / 4.f);
    distance_angle[1]   = (float)(DEG_TO_RAD * angle / 64.f);

    return 1;
}


void Single_Request_No_Response(const uint16_t command[3]) {

    EUSCI_A2_UART_OutChar(       0xA5 );    // start flag
    EUSCI_A2_UART_OutChar( command[0] );

#ifdef TIMER_A
#else

    // wait for the RPLiDAR to process the command
    Clock_Delay1ms(command[2]);

#endif // TIMER_A

}


uint8_t Single_Request_Single_Response(
        const uint8_t        command[2],
              uint8_t RX_DATA_BUFFER[RPLiDAR_UART_BUFFER_SIZE])
{

    char start_flag_1, start_flag_2;

//    uint32_t result;
//    uint8_t send_mode = 0, datatype = 0;


    // send commands
    EUSCI_A2_UART_OutChar(       0xA5 );    // start flag
    EUSCI_A2_UART_OutChar( command[0] );

    // read the response descriptor from the RPLiDAR C1
    start_flag_1    = EUSCI_A2_UART_InChar();
    start_flag_2    = EUSCI_A2_UART_InChar();


#ifdef RPLIDAR_DEBUG
    printf("0x%02X; 0x%02X\n", start_flag_1, start_flag_2);
#endif

    // if the response descriptor identifies the correct sequence
    if ( (start_flag_1 == 0xA5) && (start_flag_2 == 0x5A) ) {

        // for-loop is too slow to use on the RPLiDAR C1
        RX_DATA_BUFFER[0]   = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[1]   = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[2]   = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[3]   = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[4]   = EUSCI_A2_UART_InChar();

//        result  =   (uint32_t)(RX_DATA_BUFFER[3] << 22)
//                  | (uint32_t)(RX_DATA_BUFFER[2] << 14)
//                  | (uint32_t)(RX_DATA_BUFFER[1] <<  6)
//                  | (uint32_t)(RX_DATA_BUFFER[0] >>  2);

        // Extract the send mode bits and the datatype (2 bits) from the last
        // response byte
//        send_mode   = (RX_DATA_BUFFER[3] << 2) & 0x3;
//        datatype    =  RX_DATA_BUFFER[4];

#ifdef RPLIDAR_DEBUG

        print_binary_sequence(RX_DATA_BUFFER, command[1]);

        printf("RPLiDAR Response: 0x%08X\n", result);
        printf("RPLiDAR Response type: 0x%02X\n", send_mode);
        printf("RPLiDAR Response datatype: 0x%02X\n", datatype);
#endif

        return 1;


    // if the response descriptor IDs the incorrect sequence
    } else {

#ifdef RPLIDAR_DEBUG
        printf("Invalid Response\n");
#endif

        // wait for some time before resetting the RPLiDAR
        Clock_Delay1ms(5000);
        return 0;
    }
}



void Single_Request_Multiple_Response(
        const uint8_t        command[2],
              uint8_t RX_DATA_BUFFER[RPLiDAR_UART_BUFFER_SIZE])
{

    char start_flag_1, start_flag_2;

//    uint32_t result;
//    uint8_t send_mode = 0, datatype = 0;


    EUSCI_A2_UART_OutChar(       0xA5 );    // start flag
    EUSCI_A2_UART_OutChar( command[0] );

    start_flag_1 = EUSCI_A2_UART_InChar();
    start_flag_1 = EUSCI_A2_UART_InChar();
    start_flag_2 = EUSCI_A2_UART_InChar();


#ifdef RPLIDAR_DEBUG
    printf("0x%02X & 0x%02X\n", start_flag_1, start_flag_2);
#endif


    if ( (start_flag_1 == 0xA5) && (start_flag_2 == 0x5A) ) {

        // for-loop is too slow to use on the RPLiDAR C1
        RX_DATA_BUFFER[0] = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[1] = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[2] = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[3] = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[4] = EUSCI_A2_UART_InChar();

//        result  =   (uint32_t)(RX_DATA_BUFFER[3] << 22)
//                  | (uint32_t)(RX_DATA_BUFFER[2] << 14)
//                  | (uint32_t)(RX_DATA_BUFFER[1] <<  6)
//                  | (uint32_t)(RX_DATA_BUFFER[0] >>  2);

        // Extract the send mode bits and the datatype (2 bits) from the last
        // response byte
//        send_mode   = (RX_DATA_BUFFER[3] << 2) & 0x3;
//        datatype    =  RX_DATA_BUFFER[4];

#ifdef RPLIDAR_DEBUG

        print_binary_sequence(RX_DATA_BUFFER, command[1]);

        printf("RPLiDAR Response: 0x%08X\n", result);
        printf("RPLiDAR Response: %d\n", result);
        printf("RPLiDAR Response type: 0x%02X\n", send_mode);
        printf("RPLiDAR Response datatype: 0x%02X\n", datatype);
#endif

    } else {
        printf("Invalid Response\n");

        // wait for some time before resetting the RPLiDAR
        Clock_Delay1ms(5000);
        return;
    }
}



// -------------------------------------------------------------------------------------
// 
//  TRANSMIT AND RECEIVE CHECKING FUNCTIONS
// 
// -------------------------------------------------------------------------------------


#ifdef TX_RX_CHECKS

uint8_t EUSCI_A2_UART_Transmit_Data()
{
    uint8_t button_status = Get_Buttons_Status();
    uint8_t tx_data = 0x00;

    switch(button_status)
    {
        // Button 1 and Button 2 are pressed
        case 0x00:
        {
            tx_data = 0x00;
            EUSCI_A2_UART_OutChar(tx_data);
            break;
        }

        // Button 1 is pressed
        // Button 2 is not pressed
        case 0x10:
        {
            tx_data = 0xAA;
            EUSCI_A2_UART_OutChar(tx_data);
            break;
        }

        // Button 1 is not pressed
        // Button 2 is pressed
        case 0x02:
        {
            tx_data = 0x46;
            EUSCI_A2_UART_OutChar(tx_data);
            break;
        }

        // Button 1 and Button 2 are not pressed
        case 0x12:
        {
            tx_data = 0xF0;
            EUSCI_A2_UART_OutChar(tx_data);
            break;
        }

        default:
        {
            tx_data = 0xF0;
            EUSCI_A2_UART_OutChar(tx_data);
            break;
        }
    }

    return tx_data;
}

void EUSCI_A2_UART_Ramp_Data(uint8_t TX_Buffer[], uint8_t RX_Buffer[])
{
    int i;
    // Create a for-loop that starts from index 0 to index 255 (use BUFFER_LENGTH)
    for (i = 0; i < BUFFER_LENGTH; i++)
    {
        // Make a function call to EUSCI_A2_UART_OutChar and pass the index value
        EUSCI_A2_UART_OutChar( (uint8_t)i );

        // Assign the value of the index to TX_Buffer[i]
        TX_Buffer[i] = (uint8_t)i;

        // Assign the value returned by EUSCI_A2_UART_InChar to RX_Buffer[i]
        RX_Buffer[i] = EUSCI_A2_UART_InChar();
    }
}

void EUSCI_A2_UART_Validate_Data(uint8_t TX_Buffer[], uint8_t RX_Buffer[])
{
    int i;

    // Create a for-loop that starts from index 0 to index 255 (use BUFFER_LENGTH)
    for (i = 0; i < BUFFER_LENGTH; i++)
    {
        // Print the contents of TX_Buffer[i] and RX_Buffer[i] in one line. There should
        // be a newline (i.e. \n) for each iteration
        printf("i=%d:\tTX: 0x%02X -->\tRX: 0x%02X\n", i, TX_Buffer[i], RX_Buffer[i]);

        // Include a condition that checks if TX_Buffer[i] != RX_Buffer[i]. If there is
        // a data mismatch between TX_Buffer[i] and RX_Buffer[i], then indicate in a
        // printf message that there is a mismatch and specify which set of data is not
        // the same
        if ( TX_Buffer[i] != RX_Buffer[i] ) {
            printf("\ti=%d is not equal!\n");
        }
    }
}

#endif // TX_RX_CHECKS





