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
 * @note this file has been modified by Prof. Aaron Nanas to
 *
 * @note For more information regarding the Enhanced Universal Serial
 *  Communication Interface (eUSCI), refer to the MSP432Pxx Microcontrollers
 *  Technical Reference Manual
 *
 * @note For uses on other (cooler) microcontrollers, be aware of
 *  initialization, the amount of data captured per interrupt cycle, etc.
 *
 *  This current system depends on a one-byte capture per interrupt cycle.
 *
 * @author Gian Fajardo
 */

#include "../inc/RPLiDAR_A2_UART.h"


/**
 * @brief local variables to be used in this file
 */
RPLiDAR_Config* config      = NULL;

static uint8_t*  RX_POINTER = NULL;

uint32_t* INTERM_POINTER    = NULL;


uint32_t process_data_flag;


uint8_t Scan_All(uint32_t data) {
    return 1;
}


// ----------------------------------------------------------------------------
//
//  CONFIGURATION FUNCTIONS
//
// ----------------------------------------------------------------------------

void Configure_RPLiDAR_Struct(
        const RPLiDAR_Config*   input_config)
{

    // link the `RPLiDAR_Config` struct to the outside
    config  = (RPLiDAR_Config*)input_config;


    // Assign current buffer position and the absolute position of the array
    // to the pointer
    RX_POINTER      = (uint8_t*)&uart_container;
    INTERM_POINTER  = (uint32_t*)&container;


    // Generate the reset state
    Reset_State();

    config->isr_counter = 0;


    // set up flag for in-motion processing
    process_data_flag   = 0;


#ifdef DEBUG_OUTPUT
    printf(" 0 -> %2d -> %2d\n", SKIP_INDEX, SKIP_INDEX + MSG_LENGTH);
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



// ----------------------------------------------------------------------------
//
//  INTERRUPT HANDLER FUNCTIONS
//
// ----------------------------------------------------------------------------

static void Hold_Action(void) {

    config->isr_counter++;

    // if within the current counting limit, return early -->
    if (config->isr_counter < WAIT_INDEX)   return;

    // reset the counter
    config->isr_counter = 0;

    /**
     * @note
     *  - if `READY`, then carry on to `FIND_PATTERN`
     *  - if not, then stay in `READY` until the `WAIT_INDEX` limit is
     *      reached.
     */
    if (config->current_state == READY) {

        Timer_A1_Ignore();

        config->current_state   = RECORDING;
        config->  limit_status  = FIND_PATTERN;
        config-> buffer_pointer = RX_POINTER;

    }
}

static void Find_Pattern_Action(void) {

    // assign `data` and increment pointer afterwards
    *(config->buffer_pointer++)   = (uint8_t)data;

    config->isr_counter++;

    // if within `FIND_INDEX`, return early -->
    if (config->isr_counter < FIND_INDEX)   return;

    // reset the counter
    config->isr_counter = 0;

//    printf("fp\n");


    uint32_t    found   = 0;

    /**
     * @brief When in `FIND_PATTERN`, we have `n+1` cols of data. We're
     *      sweeping this data with `n` cols of pattern-checking slides
     *      with an `offset` at each byte. When found, `offset` is the
     *      amount left to re-align to the correct index.
     */
    for (offset = 0; offset < MSG_LENGTH; offset++) {

        // if the pattern is found on the first 5-bytes, find the rest
        if ( pattern(RX_POINTER + MSG_LENGTH*0 + offset) ) {
            found =     pattern(RX_POINTER + MSG_LENGTH*1 + offset)
                     && pattern(RX_POINTER + MSG_LENGTH*2 + offset);
        }

        if (!found)
            continue;
        
        // `RECORD` if already aligned; else `ADD_OFFSET`
        config->limit_status    = (offset == 0) ? RECORD : ADD_OFFSET;

        break;
    }

    // regardless of the state of pattern-match, start buffer over
    config->buffer_pointer  = RX_POINTER;

    config->interm_buffer_pointer   = INTERM_POINTER;
    config->interm_buffer_counter   = 0;

}

static void Add_Offset_Action(void) {

    config->isr_counter++;

    // if within the variable `offset` limit, return early -->
    if (config->isr_counter < offset)       return;

    // reset the counter
    config->isr_counter = 0;

    config->limit_status    = RECORD;

}

static void Skip_Action(void) {

    if (process_data_flag) {

        // clear the flag
        process_data_flag = 0;

        // @todo min and max angle rejection
        if ( config->angle_filter(*config->interm_buffer_pointer) ) {
            config->interm_buffer_pointer++;
            config->interm_buffer_counter++;
        }

    }

    config->isr_counter++;

    // if within `SKIP_INDEX`, return early -->
    if (config->isr_counter < SKIP_INDEX)   return;

    // reset the counter
    config->isr_counter = 0;


    // if `interm_buffer_counter` exceeds `RPLiDAR_UART_BUFFER_SIZE`
    if (config->interm_buffer_counter > INTERMEDIARY_BUFFER) {

        End_Record();

        Timer_A1_Acknowledge();

#ifdef DEBUG_OUTPUT
//        printf("b\n");
#endif

        // Stop_Timer();

    } else {

        // up-/left-shift to RECORD
         config->limit_status    = RECORD;
    }
    
}

static void Record_Action(void) {

    // assign `data` and increment pointer afterwards
    *(config->buffer_pointer++) = (uint8_t)data;

    config->isr_counter++;

    // if within `MSG_LENGTH`, return early -->
    if (config->isr_counter < MSG_LENGTH)   return;

    // reset the counter
    config->isr_counter     = 0;
    config->buffer_pointer  = RX_POINTER;

    // down-/right-shift to `SKIP`
    config->limit_status    = SKIP;


    // if it passes the zero-distance filter...
    if (    config->buffer_pointer[4]
         && config->buffer_pointer[3]) {

        // perform tasks
        *(config->interm_buffer_pointer) \
//        *(config->interm_buffer_pointer++)
                = Compact_Data(config->buffer_pointer);


        // ...mark the flag for in-motion processing
        process_data_flag   = 1;

//        printf("f");
    }

}


/**
 * @brief array definition of the FSM table
 */
RPLiDAR_State_t FSM_Table[5] = {

    {.action    = &Hold_Action},
    {.action    = &Find_Pattern_Action},
    {.action    = &Add_Offset_Action},
    {.action    = &Skip_Action},
    {.action    = &Record_Action}

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

}


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
// 
// ----------------------------------------------------------------------------

void Start_Record(Angle_Filter filter) {

    // assign the filter function
    config->angle_filter    = (filter == NULL) ? &Scan_All : filter;


    // only when idling and invoking this function makes it ready
    if (config->current_state == IDLING)
        config->current_state   = READY;
}

static inline uint8_t pattern(const uint8_t*   msg_ptr) {

#define BYTE_1_CHECK 0x01

    // Check byte 0: bits 0 and 1 should be complements (01 or 10)
    // Valid values are 0x01 or 0x02, invalid are 0x00 or 0x03
    uint8_t start_bits  = msg_ptr[0] & 0x03;
    
    return     ((start_bits == 0x01) || (start_bits == 0x02))
            && ((msg_ptr[1] & BYTE_1_CHECK) == BYTE_1_CHECK);
}


static inline uint32_t Compact_Data(volatile uint8_t* msg_ptr) {

    uint32_t angle_dist =   (  msg_ptr[2]         << 23 ) \
                          | ( (msg_ptr[1] & 0x7F) << 16 ) \
                          | (  msg_ptr[4]         <<  8 ) \
                          | (  msg_ptr[3]         <<  0 ) ;

    return angle_dist;
}



static void Reset_State(void) {

    config->current_state   = IDLING;
    config->  limit_status  = HOLD;
    config-> buffer_pointer = RX_POINTER;

    config-> interm_buffer_pointer  = INTERM_POINTER;
    config-> interm_buffer_counter  = 0;
}


static void End_Record(void) {

//    printf("%5d\n", config->interm_buffer_counter);

    config->current_state   = PROCESSING;
    config->  limit_status  = HOLD;
//    config->  limit         = WAIT_INDEX;
    config-> buffer_pointer = RX_POINTER;
//    config-> buffer_counter = 0;

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

