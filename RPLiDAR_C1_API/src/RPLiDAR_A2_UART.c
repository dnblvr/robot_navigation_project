/**
 * @file RPLIDAR_A2_UART.c
 * @brief Source code for the RPLIDAR_A2_UART driver.
 *
 * This file contains the function definitions for the EUSCI_A2_UART driver.
 *
 * @note Assumes that the necessary pin configurations for UART communication
 *      have been performed on the corresponding pins. P3.2 is used for UART RX
 *      while P3.3 is used for UART TX.
 *
 * @note For more information regarding the Enhanced Universal Serial
 *      Communication Interface (eUSCI), refer to the MSP432Pxx 
 *      Microcontrollers Technical Reference Manual
 *
 * @note for uses for other (cooler) microcontrollers, be aware of initialization,
 *
 *      the amount of data captured per interrupt cycle.
 *
 *      This current system depends on a one-byte capture per interrupt cycle.
 *
 * @author Gian Fajardo
 */

#include "../inc/RPLiDAR_A2_UART.h"
#include "../inc/Timer_A1_Interrupt.h"


/**
 * @brief local variable
 */
static volatile RPLiDAR_Config* config = NULL;

static uint32_t wait_index,
                find_index,
                skip_index;
static uint8_t* RX_POINTER = NULL;


void configure_RPLiDAR_struct(
        const RPLiDAR_Config*   input_config,
        const uint8_t*          RX_Data)
{

    // link the RPLiDAR_Config struct to the outside
    config  = (RPLiDAR_Config*)input_config;


    // calculate the indices based on the skip factor
    wait_index  = MSG_LENGTH*8;
    find_index  = MSG_LENGTH*4;
    skip_index  = MSG_LENGTH*config->skip_factor;


    // Assign current buffer position and the absolute position of the array
    // to the pointer
    RX_POINTER              = (uint8_t*)RX_Data;


    // Generate the counter variables
    config->    isr_counter = 0;
    config-> buffer_counter = 0;
    config-> buffer_pointer = RX_POINTER;

    config->  limit_status  = HOLD;
    config->current_state   = IDLING;
    config->  limit         = wait_index;   // Initialize to HOLD state


#ifdef DEBUG_OUTPUT
    // printf(" 0 -> %2d -> %2d\n", skip_index, skip_index + MSG_LENGTH);
#endif

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

    // ------------------------------------------------------------------------


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
    // writing a value of 10b to the UCSSELx field (Bits 7 to 6) in the CTLW0 
    // register
    // EUSCI_A2->CTLW0    |=  0x00C0;
    EUSCI_A2->CTLW0    &= ~0x00C0;
    EUSCI_A2->CTLW0    |=  0x0080;


    // Set the baud rate value by writing to the UCBRx field (Bits 15 to 0)
    // in the BRW register with the chosen clock frequency of the above
    // modification...
    //
    // - with a target baud rate of 460,800:
    //      - N = (Clock Frequency) / (Baud Rate)
    //          = (SMCLK Frequency) / (Baud Rate) = (12,000,000 / 460,800)
    //          = 26.0416666667
    // 
    // ...Use only the integer part, so N = 26
    //
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


    // ------------------------------------------------------------------------

    // turn on interrupt number 18 using ISER[0]
    // ISER[0] = 1 << 18
    NVIC->ISER[0] =  0x00040000;


    // set priority to n
    // IP[4] = 0x02 << 21
    // NVIC->IP[4] = (NVIC->IP[4] & 0xFF0FFFFF) | 0x00000000;  // 0
     NVIC->IP[4] = (NVIC->IP[4] & 0xFF0FFFFF) | 0x00200000;  // 1
//    NVIC->IP[4] = (NVIC->IP[4] & 0xFF0FFFFF) | 0x00400000;  // 2
    // NVIC->IP[4] = (NVIC->IP[4] & 0xFF0FFFFF) | 0x00600000;  // 3

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
     *
     * @todo: there is another problem with the distance measurements. Some data
     *  is literally zero-distance. here's a sample of the data I collected:
     *      RAW[02 75 0D 00 00] -> dist=0 ang=27.8 deg
     *      RAW[02 2B 0E 00 00] -> dist=0 ang=28.7 deg
     *      RAW[02 DF 0E 00 00] -> dist=0 ang=29.5 deg
     *      RAW[02 C9 13 00 00] -> dist=0 ang=39.1 deg
     *      RAW[02 23 14 00 00] -> dist=0 ang=40.5 deg
     *      RAW[96 7D 14 9B 09] -> dist=2459 ang=42.0 deg
     *      RAW[76 D7 14 74 09] -> dist=2420 ang=41.4 deg
     *      RAW[56 2F 15 68 09] -> dist=2408 ang=42.7 deg
     *      RAW[02 89 15 00 00] -> dist=0 ang=42.1 deg
     *      RAW[02 E1 15 00 00] -> dist=0 ang=43.5 deg
     *      RAW[02 3B 16 00 00] -> dist=0 ang=44.9 deg
     *      RAW[02 99 16 00 00] -> dist=0 ang=44.4 deg
     *      RAW[02 F3 16 00 00] -> dist=0 ang=45.8 deg
     *
     *      the ISR should be the first point-of-entry and queue handler to check
     *          if the buffer is full!
     */

    /**
     * @note this particular codebase uses a 32-bit microprocessor. This container
     *  is 32-bits so it conforms to the width of its ALU even though each data is
     *  only 8 bits wide.
     */
    uint32_t data;
//    uint32_t process_data_flag;


    // if EUSCI_A2 RXIFG flag is read, then do the following commands
    if (EUSCI_A2->IFG & 0x01) {


        /**
         *  the code is made such that it ignores recording certain messages
         *  above a skip point
         *  @note this should be recorded first and foremost!
         */
        data = (uint8_t)EUSCI_A2->RXBUF;


        // TASKS to perform while counting -----------------------------

        /**
         * Add to the buffer if in the desired states, and increment
         *  buffer_counter.
         *  - FIND_PATTERN: 0b001
         *  - RECORD:       0b100
         */
        if (config->limit_status & (FIND_PATTERN | RECORD)) {

            // assign `data` and increment pointer afterwards
          *(config->buffer_pointer++)   = (uint8_t)data;
            config->buffer_counter++;

        }

        /**
         * @todo    In the future, after recording the five-char message,
         *  sort it for zero-distance and convert it to angle and distance
         *  *immediately*.
         * @note Because of the 48 MHz clock and a message is recorded
         *      every half-MHz, quite a lot of tasks can be done!
         *      just use `Profiler.h` to make sure the sorting method
         *      is up-to-spec.
         *
         */
//        if (config->limit_status & (SKIP)) {
//
//
//        }

        // TASKS to perform while counting end -------------------------


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

                    Timer_A1_Ignore();

                    config->current_state   = RECORDING;
                    config->  limit_status  = FIND_PATTERN;
                    config->  limit         = find_index;
                    config-> buffer_pointer = RX_POINTER;
                    config-> buffer_counter = 0;

//                    printf("a\n");

                }

                break;

            }


            case FIND_PATTERN: {  // ------------------------------------------

                // printf("b");

                uint32_t    offset;
                uint32_t    found   = 0;

                // check which `offset` his most aligned
                for (offset = 0; offset < MSG_LENGTH; offset++) {

                    found =    pattern(RX_POINTER + MSG_LENGTH*0 + offset)
                            && pattern(RX_POINTER + MSG_LENGTH*1 + offset)
                            && pattern(RX_POINTER + MSG_LENGTH*2 + offset);

                    if (found) {

                        // if already aligned, go directly to SKIP
                        if (offset == 0) {
                            
                            config->limit_status    = SKIP;
                            config->limit           = skip_index;
                        
                        // if not yet aligned go to ADD_OFFSET
                        } else {
                            
                            config->limit_status    = ADD_OFFSET;
                            config->limit           = offset;
                        }

                        // printf("1");

                        break;

                    }

                }

                Timer_A1_Acknowledge();

                // regardless of if pattern is found or not found, reset the
                // pointer and counter for the full data record
                config->buffer_pointer  = RX_POINTER;
                config->buffer_counter  = 0;

                // printf("2");

                break;

            }


            case ADD_OFFSET: {  // --------------------------------------------

                config-> limit_status   = SKIP;
                config->buffer_pointer  = RX_POINTER;
                config->limit           = skip_index;
                config->buffer_counter  = 0;

                break;

            }


            /**
             * @note after the ADD_OFFSET stage, it trades between SKIP and
             *      RECORD until `config->buffer_counter` exceeds
             *      RPLiDAR_UART_BUFFER_SIZE
             *
             * @todo switch it with RECORD and then SKIP
             *
             *      process_data_flag
             */

            case SKIP: {  // --------------------------------------------------

                if (config->buffer_counter > RPLiDAR_UART_BUFFER_SIZE) {

                    config->current_state   = PROCESSING;
                    config->limit_status    = HOLD;
                    config->limit           = wait_index;
                    config->buffer_pointer  = RX_POINTER;
                    config->buffer_counter  = 0;

//                    Stop_Timer();

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

                if (config->buffer_counter > RPLiDAR_UART_BUFFER_SIZE) {

                    config->current_state   = PROCESSING;
                    config->limit_status    = HOLD;
                    config->limit           = wait_index;
                    config->buffer_pointer  = RX_POINTER;
                    config->buffer_counter  = 0;

//                    Timer_A1_Acknowledge();

//                    Stop_Timer();

//                    printf("e2\n");

                } else {

                    // down-/right-shift to SKIP
                    // config->limit_status    = SKIP;
                    config->limit_status   -= 1;
                    config->limit           = skip_index;

//                    printf("d2\n");
                }

                break;

            }


            default: {  // ----------------------------------------------------

                config->limit_status    = HOLD;
                config->limit           = wait_index;
                break;

            }


            } // end switch(config->limit_status)  // -------------------------

        }

    }

}



// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
// 
// ----------------------------------------------------------------------------

static inline uint8_t pattern(const uint8_t*   msg_ptr) {

#define BYTE_1_CHECK 0x01

    // Check byte 0: bits 0 and 1 should be complements (01 or 10)
    // Valid values are 0x01 or 0x02, invalid are 0x00 or 0x03
    uint8_t start_bits  = msg_ptr[0] & 0x03;
    
    return     (start_bits == 0x01 || start_bits == 0x02)
            && ((msg_ptr[1] & BYTE_1_CHECK) == BYTE_1_CHECK);
}


uint8_t confirm_aligned(
        const uint8_t RPLiDAR_RX_Data[RPLiDAR_UART_BUFFER_SIZE])
{
    uint32_t i;
    uint32_t aligned;

    // Check all MSGS until proven otherwise
    aligned     = 1;
    for (i = 0; i < RPLiDAR_UART_BUFFER_SIZE; i += MSG_LENGTH)
    {
        // return early if mis-aligned packet is found
        if ( !pattern(RPLiDAR_RX_Data + i) ) {
         	aligned = 0;
         	break;
//            return 0;
        }

    }

//    printf("aligned = %d\n", aligned);
    return aligned;
//    return 1;
}



// ----------------------------------------------------------------------------
// 
//  TRANSMIT AND RECEIVE CHECKING FUNCTIONS
// 
// ----------------------------------------------------------------------------


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

