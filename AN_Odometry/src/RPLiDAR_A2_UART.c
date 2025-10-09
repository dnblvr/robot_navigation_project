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

#include "../inc/RPLiDAR_A2_UART.h"


void EUSCI_A2_UART_Init(RPLiDAR_Config *input_config, uint8_t *RX_Data)
{

    // link the RPLiDAR_Config struct to the outside
    config = input_config;


    // assign current buffer position and the absolute position of the array
    // to the pointer
    config->RX_POINTER      = RX_Data;
    config->buffer_pointer  = RX_Data;


    // Generate the counter variables and the start and stop indices of the nth
    // message. 
    // @note: when the proper offset value is reached, `isr_counter` need to be
    //      inc-/decremented
    config->  isr_counter   = 0;
    config->print_counter   = 0;
    config-> find_index     = MESSAGE_LENGTH*4;
    config-> skip_index     = MESSAGE_LENGTH*config->skip_factor;

    config->limit_status    = HOLD;

    // temporary state
//    config-> record_data    = 1;


//    printf("%2d -> %2d\n", config->start_index, config->stop_index);



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
    // N = (Clock Frequency) / (Baud Rate) = (12,000,000 / 460,800)
    //   = 26.0416666667
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

    // @todo: figure out what to do in the HOLD state

    // if the counting system is a trustless system (has to run find_pattern
    // every single time), then to make it a trusted system I have to *really*
    // minimize the overhead in the ISR and take the hold state to SKIP. make
    // sure though that the ISR_counter is still limited in multiples of MESSAGE_LENGTH


    uint8_t data;


    // if EUSCI_A2 RXIFG flag is read, then do the following commands
    if ((EUSCI_A2->IFG & 0x01)) {

        /**
         *  the code is made such that it ignores recording certain messages
         *  above a skip point
         *  @note this should be recorded first and foremost!
         */
        data = (uint8_t)EUSCI_A2->RXBUF;


        // add to the buffer based on the desired state it is, and increment
        // print_counter
        if (config->limit_status & (FIND_PATTERN | RECORD)) {

            // apparently this operation takes one cycle
            *(config->buffer_pointer++) = data;
            config->print_counter++;

        }

        // increment isr_counter
        config->isr_counter++;


        /**
         * if isr_counter does arrive at the limit, do the following objectives:
         *  1. transition to the next limit_status
         *  2. change limits based on limit_status
         */
        if (config->isr_counter >= config->limit) {

            // reset the counter
            config->isr_counter = 0;


            // switch statement uses jump tables which take up the least overhead
            // which is perfect for this application
            switch(config->limit_status) {


            case HOLD: {  // -----------------------------------------------------

                if (config->record_data) {

                    config->limit_status    = FIND_PATTERN;
                    config->limit           = config->find_index;
                    config->buffer_pointer  = config->RX_POINTER;
                }

                break;

            }


            case FIND_PATTERN: {  // ---------------------------------------------

//                printf("%02X: ", config->limit_status);

                int     i;
                uint8_t found;

                for (i = 0; i < (MESSAGE_LENGTH + 1); i++) {

                    found =   (0x1 * pattern(config->RX_POINTER + MESSAGE_LENGTH*0 + i))
                            | (0x2 * pattern(config->RX_POINTER + MESSAGE_LENGTH*1 + i))
                            | (0x4 * pattern(config->RX_POINTER + MESSAGE_LENGTH*2 + i));

//                    printf("%01X ", found);

                    if ( (found & 0x7) == 0x7 ) {

//                        printf("found\n");
//                        printf("a");
//                        printf("a\n");

                        // "MESSAGE_LENGTH - i" is the amount needed to re-align to the
                        // first byte of the message
                        config->limit_status    = ADD_OFFSET;
                        config->limit           = MESSAGE_LENGTH - i;

                        break;

                    }

                }

                // regardless of the outcome, reset whole array
                config->print_counter   = 0;
                config->buffer_pointer  = config->RX_POINTER;

                break;

            }


            case ADD_OFFSET: {  // -----------------------------------------------

//                printf("b");

                config->limit_status    = SKIP;
                config->limit           = config->skip_index;

                break;

            }

            /**
             * @note after the ADD_OFFSET stage, it trades between SKIP and RECORD
             */


            case SKIP: {  // -----------------------------------------------------

                if (config->print_counter >= RPLiDAR_UART_BUFFER_SIZE) {

//                    printf("\n");

                    config->limit_status    = HOLD;
                    config->limit           = config->find_index;
                    config->buffer_pointer  = config->RX_POINTER;
                    config->print_counter   = 0;

                } else {

//                    printf("c");

                    // up-/left-shift to RECORD
//                    config->limit_status    = RECORD;
                    config->limit_status   += 1;
                    config->limit           = MESSAGE_LENGTH;

                }

                break;

            }


            case RECORD: {  // ---------------------------------------------------

                // if previously at the RECORD state

                if (config->print_counter >= RPLiDAR_UART_BUFFER_SIZE) {

//                    printf("\n");

                    config->limit_status    = HOLD;
                    config->limit           = config->find_index;
                    config->buffer_pointer  = config->RX_POINTER;
                    config->print_counter   = 0;

                } else {

//                    printf("d");

                    // down-/right-shift to SKIP
//                    config->limit_status    = SKIP;
                    config->limit_status   -= 1;
                    config->limit           = config->skip_index;

                }

                break;

            }

            case PROCESS: {

//                if (config->)

                break;

            }



            default: {  // -------------------------------------------------------

                config->limit_status    = HOLD;
                config->record_data     = 0;
                break;

            }

            } // end switch(config->limit_status)

//            printf("%02X %5d\n", *(config->buffer_pointer), config->isr_counter);
//            printf("%02X\n", *(config->buffer_pointer));
//            printf("%5d\n", config->isr_counter);

        } else {

            return;

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

        // Extract the send mode bits and the datatype (2 bits) from the last response
        // byte
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

        // Extract the send mode bits and the datatype (2 bits) from the last response
        // byte
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


void Gather_LiDAR_Data(
        RPLiDAR_Config       *cfg,

        uint8_t scan_confirmation,
        uint8_t           RX_Data[RPLiDAR_UART_BUFFER_SIZE],

        float                 out[FLOAT_BUFFER][3])
{

    uint16_t i, j, start, end, data_len = 5;

    float distance_angle[2] = {0};

    EUSCI_A2_UART_Restart();

    // Gathering data from the RPLiDAR C1. At this point, it
    // should stop recording data as soon as the data stops
    // recording.
    for (i = 0; i < RPLiDAR_UART_BUFFER_SIZE; i++) {

        RX_Data[i]  = EUSCI_A2_UART_InChar();
    }

    EUSCI_A2_UART_Stop();


    // find the pattern in the data using pattern() function.
    for (i = 0; i < (data_len + 1); i++) {

        // if the increment is found
        if (   pattern(RX_Data + data_len*0 + i)
            && pattern(RX_Data + data_len*1 + i)
            && pattern(RX_Data + data_len*2 + i)
            && pattern(RX_Data + data_len*3 + i))
        {
            start = i;

#ifdef RPLIDAR_DEBUG
            printf("i = %d\n", start);
#endif

            break;
        }

        // If the increment passes the critical range
        //  from "0" to "data_len - 1"
        if (i == data_len) {

#ifdef RPLIDAR_DEBUG
            printf("pattern not found\n");
#endif
            return;
        }
    }

    end = start + RPLiDAR_UART_BUFFER_SIZE - MESSAGE_LENGTH*cfg->skip_factor;

#ifdef RPLIDAR_DEBUG

    // print data
    for (i = start; i < end; i++) {

        if (i % 20 == 0) {
            printf("\n");
        }

        printf("%02X\n", RX_Data[i]);

    }

#endif

    // Then, fill up the FLOAT_BUFFER

    j = 0;
    for (i = start; i < end; i += MESSAGE_LENGTH*cfg->skip_factor) {

        uint8_t is_nonzero;

        if (j >= FLOAT_BUFFER)
            break;

        // convert the data to distance and angle
        is_nonzero = to_angle_distance( &RX_Data[i], distance_angle );

        // if zero radius, then disregard the data
        if (!is_nonzero)
            continue;

        // convert the polar coordinates to Cartesian coordinates
        polar_to_cartesian( distance_angle, out[j] );


#ifdef RPLIDAR_DEBUG
        // print the distance and angle
        fprintf(stdout, "%i\t%3.2f rad. @ %5.2f mm\n",
                j, distance_angle[1], distance_angle[0]);

#endif

        // print the position
        fprintf(stdout, "%i\t%10.2f %10.2f\n",
                j, out[j][0], out[j][1]);

        ++j;

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





