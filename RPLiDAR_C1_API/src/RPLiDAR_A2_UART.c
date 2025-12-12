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
 * @note For uses on other (cooler) microcontrollers, be aware of initialization,
 *      the amount of data captured per interrupt cycle, etc.
 *
 *      This current system depends on a one-byte capture per interrupt cycle.
 *
 * @author Gian Fajardo
 */

#include "../inc/RPLiDAR_A2_UART.h"


/**
 * @brief local variables to be used in this file
 */
RPLiDAR_Config* config = NULL;

static uint32_t wait_index = 0,
                find_index = 0,
                skip_index = 0;

static uint8_t*  RX_POINTER     = NULL;

uint32_t* INTERM_POINTER = NULL;


uint32_t process_data_flag;


// ----------------------------------------------------------------------------
//
//  CONFIGURATION FUNCTIONS
//
// ----------------------------------------------------------------------------

void Configure_RPLiDAR_Struct(
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
    RX_POINTER      = (uint8_t*)RX_Data;
    INTERM_POINTER  = (uint32_t*)&container;


    // Generate the reset state
    Reset_State();

    config->isr_counter = 0;


    // set up flag for in-motion processing
    process_data_flag = 0;


#ifdef DEBUG_OUTPUT
     printf(" 0 -> %2d -> %2d\n", skip_index, skip_index + MSG_LENGTH);
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


#ifndef FUNCTION_TABLES

void EUSCIA2_IRQHandler(void) {

    /**
     * @todo: currently this ISR a trustless system (has to find the C1
     *      message pattern for every single iteration), then to make it a
     *      trusted system I have to prioritize a few things:
     * 
     *  1. minimize the overhead in the UART ISR
     * 
     *  2. and take the hold state to SKIP (?).
     *  3. make sure that `isr_counter` counts in multiples of MSG_LENGTH
     *
     * @todo: there is another problem with the distance measurements. Some data
     *  are zero-distance. here's a sample of the data I collected:
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
     *      the ISR should be the first point-of-entry to filter out these things.
     */

    /**
     * @note this particular codebase uses a 32-bit microprocessor. This container
     *  is 32-bits so it conforms to the width of its ALU even though each data is
     *  only 8 bits wide.
     */
    uint32_t data;


    // if EUSCI_A2 RXIFG flag is read, then do the following commands
    if (EUSCI_A2->IFG & 0x01) {


        /**
         *  the code is made such that it ignores recording certain messages
         *  above a skip point
         *  @note this should be recorded first and foremost!
         */
        data = (uint8_t)EUSCI_A2->RXBUF;


        /** ------------------------------------------------------------
         * tasks to perform while counting
         */

        /**
         * Add to the buffer if in the desired states, and increment
         *  buffer_counter.
         *  - FIND_PATTERN: 0b001   - RECORD: 0b100
         */
        if (config->limit_status & (FIND_PATTERN | RECORD)) {

            // assign `data` and increment pointer afterwards
            *(config->buffer_pointer++)   = (uint8_t)data;
              config->buffer_counter++;
        }

        /**
         * @todo    In the future, after recording the five-char message,
         *  filter for zero-distance and convert it to angle and distance
         *  *immediately*.
         *
         * @note Because of the 48 MHz clock and a message is recorded
         *      every half-MHz, quite a lot of tasks can be done!
         *      just use `Profiler.h` to make sure the sorting method
         *      is up-to-spec.
         */
        if (    config->limit_status & (SKIP)
             && process_data_flag)
        {
            // clear the flag
            process_data_flag = 0;

            // perform binary insertion
//            to_distance_angle();
        }


        /** ------------------------------------------------------------------
         * increment `isr_counter`
         */

        config->isr_counter++;

        // if within the current counting limit, return early -->
        if (config->isr_counter < config->limit)
            return;


        /** ------------------------------------------------------------------
         * If `isr_counter` does arrive at the limit, achieve the following:
         *  1. transition to the next `limit_status`
         *  2. change limits based on `limit_status`
         */

        // reset the counter
        config->isr_counter = 0;


        // switch statement uses jump tables which take up the least
        // overhead which is perfect for this application
        switch(config->limit_status) {


        case HOLD: {  // -----------------------------------------------------

            if (config->current_state == READY) {

                Timer_A1_Ignore();

                config->current_state   = RECORDING;
                config->  limit_status  = FIND_PATTERN;
                config->  limit         = find_index;
                config-> buffer_pointer = RX_POINTER;
                config-> buffer_counter = 0;

            }

            break;

        }

        case FIND_PATTERN: {  // ---------------------------------------------

            uint32_t    offset;
            uint32_t    found   = 0;

            /**
             * @brief When in `FIND_PATTERN`, we have `n+1` cols of data. We're
             *  sweeping this data with `n` cols of pattern-checking slides
             *  with an `offset` at each iteration. When found, `offset` is the
             *  amount left to re-align to the correct index when done.
             */
            for (offset = 0; offset < MSG_LENGTH; offset++) {

                found =    pattern(RX_POINTER + MSG_LENGTH*0 + offset)
                        && pattern(RX_POINTER + MSG_LENGTH*1 + offset)
                        && pattern(RX_POINTER + MSG_LENGTH*2 + offset);

                if (found) {

                    // if already aligned, go directly to `SKIP`
                    if (offset == 0) {

                        config->limit_status    = SKIP;
                        config->limit           = skip_index;

                    // if not yet aligned, go to `ADD_OFFSET`
                    } else {
                        
                        config->limit_status    = ADD_OFFSET;
                        config->limit           = offset;
                    }

                    break;

                }

            }

            // regardless of the state of pattern-match, start buffer over
            config->buffer_pointer  = RX_POINTER;
            config->buffer_counter  = 0;

            break;

        }


        case ADD_OFFSET: {  // -----------------------------------------------

            config-> limit_status   = SKIP;
            config-> limit          = skip_index;

            break;

        }


        /**
         * @note after the ADD_OFFSET stage, it trades between SKIP and
         *      RECORD until `config->buffer_counter` exceeds
         *      RPLiDAR_UART_BUFFER_SIZE
         *
         * @todo switch it with RECORD and then SKIP
         */

        case SKIP: {  // -----------------------------------------------------

            if (config->buffer_counter > RPLiDAR_UART_BUFFER_SIZE) {

                End_Record();

                Timer_A1_Acknowledge();

                // Stop_Timer();

            } else {

                // up-/left-shift to RECORD
                // config->limit_status    = RECORD;
                config->limit_status   += 1;
                config->limit           = MSG_LENGTH;

            }

            break;

        }


        case RECORD: {  // ---------------------------------------------------

            // down-/right-shift to SKIP
            // config->limit_status    = SKIP;
            config->limit_status   -= 1;
            config->limit           = skip_index;

            // @todo mark the flag every skip for in-motion processing
            process_data_flag       = 1;

            break;

        }


        default: {  // -------------------------------------------------------

            Reset_State();

            break;

        }

        } // end switch(config->limit_status)  // -----------------------------

    }

} // end void EUSCIA2_IRQHandler(void) {


#else  // #ifndef FUNCTION_TABLES


static void Hold_Action(void) {

    config->isr_counter++;

    // if within the current counting limit, return early -->
    if (config->isr_counter < config->limit)
        return;

    // reset the counter
    config->isr_counter = 0;


    /**
     * @note
     *  - if `READY`, then carry on to `FIND_PATTERN`
     *  - if not, then stay in `READY` until the next `config->limit` is
     *      reached.
     */
    if (config->current_state == READY) {

        Timer_A1_Ignore();

        config->current_state   = RECORDING;
        config->  limit_status  = FIND_PATTERN;
        config->  limit         = find_index;
        config-> buffer_pointer = RX_POINTER;
        config-> buffer_counter = 0;

    }
}

static void Find_Pattern_Action(void) {

    // assign `data` and increment pointer afterwards
    *(config->buffer_pointer++)   = (uint8_t)data;
    config->buffer_counter++;

    config->isr_counter++;

    // if within the current counting limit, return early -->
    if (config->isr_counter < config->limit)
        return;

    // reset the counter
    config->isr_counter = 0;

    uint32_t    offset;
    uint32_t    found   = 0;

    /**
     * @brief When in `FIND_PATTERN`, we have `n+1` cols of data. We're
     *  sweeping this data with `n` cols of pattern-checking slides
     *  with an `offset` at each byte. When found, `offset` is the
     *  amount left to re-align to the correct index.
     */
    for (offset = 0; offset < MSG_LENGTH; offset++) {

        found =    pattern(RX_POINTER + MSG_LENGTH*0 + offset)
                && pattern(RX_POINTER + MSG_LENGTH*1 + offset)
                && pattern(RX_POINTER + MSG_LENGTH*2 + offset);

        if (found) {

            // if already aligned, go directly to `SKIP`
            if (offset == 0) {

//                config->limit_status    = SKIP;
//                config->limit           = skip_index;

                config-> limit_status   = RECORD;
                config-> limit          = MSG_LENGTH;

            // if not yet aligned, go to `ADD_OFFSET`
            } else {
                
                config->limit_status    = ADD_OFFSET;
                config->limit           = offset;
            }

            break;
        }
    }

    // regardless of the state of pattern-match, start buffer over
    config->buffer_pointer  = RX_POINTER;
    config->buffer_counter  = 0;


    config-> interm_buffer_pointer  = INTERM_POINTER;
    config-> interm_buffer_counter  = 0;

}

static void Add_Offset_Action(void) {

    config->isr_counter++;

    // if within the current counting limit, return early -->
    if (config->isr_counter < config->limit)
        return;

    // reset the counter
    config->isr_counter = 0;

//    config-> limit_status   = SKIP;
//    config-> limit          = skip_index;

    config-> limit_status   = RECORD;
    config-> limit          = MSG_LENGTH;

}

static void Skip_Action(void) {

    if (process_data_flag) {

        // clear the flag
        process_data_flag = 0;

        // perform binary insertion
        *(config->interm_buffer_pointer++) = Perform_Tasks(config->buffer_pointer - MSG_LENGTH);
        config->interm_buffer_counter++;
    }


    config->isr_counter++;

    // if within the current counting limit, return early -->
    if (config->isr_counter < config->limit)
        return;

    // reset the counter
    config->isr_counter = 0;


    // if `buffer_counter` exceeds `RPLiDAR_UART_BUFFER_SIZE`
    if (config->buffer_counter > RPLiDAR_UART_BUFFER_SIZE) {

        End_Record();

        Timer_A1_Acknowledge();

        // Stop_Timer();

    } else {

        // up-/left-shift to RECORD
        // config->limit_status    = RECORD;
        config->limit_status   += 1;
        config->limit           = MSG_LENGTH;

    }
    
}

static void Record_Action(void) {

    // assign `data` and increment pointer afterwards
    *(config->buffer_pointer++) = (uint8_t)data;
    config->buffer_counter++;

    config->isr_counter++;

    // if within the current counting limit, return early -->
    if (config->isr_counter < config->limit)
        return;

    // reset the counter
    config->isr_counter = 0;


    // down-/right-shift to SKIP
    // config->limit_status    = SKIP;
    config->limit_status   -= 1;
    config->limit           = skip_index;

    // @todo mark the flag every skip for in-motion processing
//    if (    !(*(config->buffer_pointer - 1))
//         && !(*(config->buffer_pointer - 2))) {
        process_data_flag   = 1;

    // if it does not pass the zero-distance filtering...
//    } else {

//        config->buffer_pointer -= MSG_LENGTH;
//        config->buffer_counter -= 1;

//    }

    
}

/**
 * @brief array definition of the smaller FSM table
 */
RPLiDAR_State_t FSM_Table[5] = {

    /* HOLD */
    {.action    = &Hold_Action,         .next_state = HOLD},

    /* FIND_PATTERN */
    {.action    = &Find_Pattern_Action, .next_state = FIND_PATTERN},

    // ADD_OFFSET
    {.action    = &Add_Offset_Action,   .next_state = ADD_OFFSET},

    // SKIP
    {.action    = &Skip_Action,         .next_state = SKIP},

    // RECORD
    {.action    = &Record_Action,       .next_state = RECORD}

};


void EUSCIA2_IRQHandler(void) {

    /**
     * if EUSCI_A2 RXIFG flag is read, then do the following commands:
     *  - record data
     *  - perform the tasks associated by `limit_status` of the function table
     */
    if (EUSCI_A2->IFG & 0x01) {

        // data should be recorded
        data    = (uint8_t)EUSCI_A2->RXBUF;

        // call the action function based on the current state
        FSM_Table[config->limit_status].action();

    }

} // end void EUSCIA2_IRQHandler(void) {

#endif  // #ifndef FUNCTION_TABLES


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
    
    return      (start_bits == 0x01 || start_bits == 0x02)
            && ((msg_ptr[1] & BYTE_1_CHECK) == BYTE_1_CHECK);
}


uint8_t Confirm_Aligned(
        const uint8_t RPLiDAR_RX_Data[RPLiDAR_UART_BUFFER_SIZE])
{
    uint32_t i;

    // Check all messages until proven otherwise
    for (i = 0; i < RPLiDAR_UART_BUFFER_SIZE; i += MSG_LENGTH)
    {
        // return early if mis-aligned packet is found
        if ( !pattern(RPLiDAR_RX_Data + i) )
            return 0;
    }

    return 1;
}


static inline uint32_t Perform_Tasks(volatile uint8_t* msg_ptr) {

    uint32_t angle_dist =   (  msg_ptr[2]         << 23 ) \
                          | ( (msg_ptr[1] & 0x7F) << 16 ) \
                          | (  msg_ptr[4]         <<  8 ) \
                          | (  msg_ptr[3]         <<  0 ) ;

    return angle_dist;
}



static void Reset_State(void) {

    config->current_state   = IDLING;
    config->  limit_status  = HOLD;
    config->  limit         = wait_index;
    config-> buffer_pointer = RX_POINTER;
    config-> buffer_counter = 0;

    config-> interm_buffer_pointer  = INTERM_POINTER;
    config-> interm_buffer_counter  = 0;

}


void Start_Record(void) {

    // only when idling and invoking this function makes it ready
    if (config->current_state == IDLING) {

        config->current_state   = READY;
    }

}


static void End_Record(void) {

//    printf("%5d\n", config->interm_buffer_counter);

    config->current_state   = PROCESSING;
    config->  limit_status  = HOLD;
    config->  limit         = wait_index;
    config-> buffer_pointer = RX_POINTER;
    config-> buffer_counter = 0;

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

