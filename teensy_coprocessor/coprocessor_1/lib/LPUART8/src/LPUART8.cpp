/**
 * @file LPUART8.cpp
 * @brief Teensy HAL for the LPUART8 driver with emphasis on bare-metal RX
 *  handling.
 *
 * @details This is a direct reference of RPLiDAR_UART codebase. The public API
 *  surface is intentionally kept somewhat identical to keep a consistent
 *  interface. Only the hardware seam is swapped.
 *
 * @note Call LPUART8_SetPort() before anything else, then LPUART8_Init() to
 *  start the serial port at a given baud rate.
 *
 * @note For every ISR call, feed all FIFO bytes to LPUART8_ProcessByte(b).
 *  The FSM will handle the rest.
 * 
 * @note This is inspired by Paul Stoffregen's musing on his forums about
 *  barebones T4.0 serial drivers. He prefers that we not spend time writing a
 *  full drivers, considering that he spent a full year writing the core
 *  library. Instead, he prefers that we write drivers based on his Arduino
 *  Teensy core library and whittle down to the bare metal. This is part of the
 *  result of this approach.
 * 
 * @note LPUART modules are aligned to Serialx ports according to Paul
 *  Stoffregen's IMXRT1060RM annotations. This is the table for convenience:
 * 
 *      | Module  | Serial Port |           | Serial Port | Module  |
 *      | :-----: | :---------: |           | :---------: | :-----: |
 *      | LPUART1 |   Serial6   |           |   Serial1   | LPUART6 |
 *      | LPUART2 |   Serial3   |           |   Serial2   | LPUART3 |
 *      | LPUART3 |   Serial2   |  <----->  |   Serial3   | LPUART2 |
 *      | LPUART4 |   Serial4   |           |   Serial4   | LPUART4 |
 *      | LPUART5 |   Serial8   |           |   Serial5   | LPUART8 |
 *      | LPUART6 |   Serial1   |           |   Serial6   | LPUART1 |
 *      | LPUART7 |   Serial7   |           |   Serial7   | LPUART7 |
 *      | LPUART8 |   Serial5   |           |   Serial8   | LPUART5 |
 * 
 * @author Gian Fajardo
 */

#include "LPUART8.h"


// ----------------------------------------------------------------------------
//
//  MODULE-LEVEL STATE
//
// ----------------------------------------------------------------------------

/** 
 * @brief Bound serial port — set by LPUART8_SetPort()
 */
static HardwareSerial* _serial = nullptr;

/** 
 * @brief Pointer into the raw 5-byte staging buffer
 */
static volatile char*  RX_POINTER    = nullptr;

/**
 * @brief pointer to the thingy thing thing
 */
static volatile char* uart_buffer_pointer = nullptr;

/**
 * @brief 
 */
static LPUART_ISR_Task task_function = nullptr;

/**
 * @brief   byte-mask for the LPUARTx DATA register
 * 
 * @details has other accessible bits stored which must be masked to get the 
 *  FIFO-drainable byte value  
 */
#define BYTE_0_MASK 0xFFu



/**
 * @brief LPUARTx check for the 
 * 
 * @details this flag asserts when there is no data currently in the receive
 *  buffer. it is recommended to assign the full contents of the LPUARTx_DATA
 *  and check this flag before looping to drain the FIFO.
 */
#define DATA_EMPTY_CONDITION (d & LPUART_DATA_RXEMPT)


// ----------------------------------------------------------------------------
//
//  PRIVATE BUFFER STORAGE
//
// ----------------------------------------------------------------------------

/** Raw staging buffer — holds UART8_BUFFER_SIZE bytes during pattern search */
static volatile char  UART8_buffer[UART8_BUFFER_SIZE] = {0};


// ----------------------------------------------------------------------------
//
//  PORT INJECTION
//
// ----------------------------------------------------------------------------

void LPUART8_SetPort(HardwareSerial* port) {
    _serial = port;
}


// ----------------------------------------------------------------------------
//
//  BARE-METAL LPUART8 RX ISR
//
// ----------------------------------------------------------------------------

// Strategy: call Serial1.begin(LPUART8_BAUD_RATE) to let HardwareSerial configure
// the baud rate, I/O pins, FIFO, and CTRL enable bits, then overwrite the
// single function-pointer HardwareSerial installed in _VectorsRam with our
// own handler.  The hardware configuration is identical — we only change
// who is called on each interrupt.
//
// FIFO drain: the iMXRT1062 LPUART8 has a 4-byte RX FIFO.  LPUART8_AttachISR
// sets RXWATER=0 so RDRF fires on every single byte — this prevents short
// frames (e.g. "!E\r\n") from losing their last byte to a stalled FIFO.
// The loop drains all available bytes in one ISR invocation.

FASTRUN void LPUART8_RX_ISR(void)
{

    uint32_t d = 0;

    // ---- Overrun detection -------------------------------------------------
    // STAT.OR (bit 19, W1C) is set when the 4-byte RX FIFO filled and the
    // next incoming byte was discarded.  That lost byte causes the FSM byte
    // counter to drift by one, misaligning every subsequent packet for the
    // entire frame.  We detect and count overruns so anomaly frames can be
    // correlated; clearing OR via |= may also clear other co-asserted W1C
    // flags (NF, FE, PF) — acceptable because the affected bytes are already
    // in (or lost from) the FIFO and cannot be recovered.
    if (IMXRT_LPUART8.STAT & LPUART_STAT_OR) {
        IMXRT_LPUART8.STAT |= LPUART_STAT_OR;
    }

    // ---- FIFO drain --------------------------------------------------------
    // Read the first DATA word atomically: LPUART_DATA_RXEMPT (bit 12) is
    // set when the receive FIFO buffer was already empty. With
    // ILIE disabled, this ISR fires only due to RDRF, so the first read
    // almost always returns a valid byte; the counter is just a safety net.
    d   = IMXRT_LPUART8.DATA;


    while ( !DATA_EMPTY_CONDITION ) {

        LPUART8_ProcessByte( (uint8_t)(d & BYTE_0_MASK) );
        d   = IMXRT_LPUART8.DATA;

    }

}


// ----------------------------------------------------------------------------
//
//  UART LIFECYCLE
//
// ----------------------------------------------------------------------------

void LPUART8_Init(uint32_t baud_rate)
{
    if (_serial == nullptr) return;

    // Let HardwareSerial configure baud, pins, FIFO, and CTRL.RIE/ILIE
    _serial->begin(baud_rate);

}


void LPUART8_AttachISR(LPUART_ISR_Task task)
{
    
    if (_serial == nullptr) return;

    // assigns the function pointer in the caller's struct to the address of the function passed in as an argument, so that the caller can call the function via this pointer when needed
    task_function       = task;

    // Point the module-level pointers at the static buffers
    RX_POINTER          = &UART8_buffer[0];
    uart_buffer_pointer = RX_POINTER;


    // Wait-blocks for TX FIFO to empty (WATER[19:16] = TXCOUNT)
    //  - `7u`: this register holds 0b111 bits
    // @note this optimization is such that it does not need to shift the WATER register to get the TXCOUNT bits; masking only leaves us interested in whether or not TXCOUNT, and the WATER register by extension, equals 0
    while (IMXRT_LPUART8.WATER & LPUART_WATER_TXCOUNT(7u));

    // Then wait for shift register to finish
    while (!(IMXRT_LPUART8.STAT & LPUART_STAT_TC));


    /** -----------------------------------------------------------------------
     * @brief Disable the transmitter and receiver before making other CTRL
     *  changes
     */
    IMXRT_LPUART8.CTRL &= ~(    LPUART_CTRL_TE
                            |   LPUART_CTRL_RE);

    // Explicitly clear TX interrupt enables AND the idle line interrupt enable (ILIE) so no spurious interrupt can fire. HardwareSerial enables ILIE to detect end-of-packet which is not needed for this application. Leaving it enabled causes ~2 400 useless ISR invocations per 100 ms frame (one per inter-packet idle gap at 4,090 packets/sec). 
    // Worse, the IDLE ISR path performs a read-modify-write on STAT to clear LPUART_STAT_IDLE; if STAT.OR (receive overrun) happens to be set at that moment, |= silently clears it — hiding the evidence of the very byte-drop that causes FSM drift. Our application only needs the RXINT so the FSM can properly count bytes internally.
    IMXRT_LPUART8.CTRL &= ~(    LPUART_CTRL_TIE
                             |  LPUART_CTRL_TCIE
                             |  LPUART_CTRL_ILIE);

    // Clear any IDLE and OR flags left pending from the init-command phase.
    IMXRT_LPUART8.STAT |=  (    LPUART_STAT_IDLE
                             |  LPUART_STAT_OR);


    /** -----------------------------------------------------------------------
     * @brief after every changes made, it is safe to re-enable the transmitter
     *  and receiver
     */
    IMXRT_LPUART8.CTRL |=  (    LPUART_CTRL_TE
                             |  LPUART_CTRL_RE);


    // changing RX FIFO watermark `RXWATER` affects invocations to the ISR. Setting RXWATER to a set byte causes the ISR to only be invoked when the FIFO has at least that many bytes, which can cause short frames to lose their last byte to a stalled FIFO. setting RXWATER to 0 means the ISR is invoked on every single byte, which prevents this issue and allows the loop in the ISR to drain all available bytes
    IMXRT_LPUART8.WATER = IMXRT_LPUART8.WATER | LPUART_WATER_RXWATER(1u);


    // From this point all LPUART8 interrupts are handled by LPUART8_RX_ISR
    // with typical priority 64 (HardwareSerial default). 0 is highest, 255 is lowest; 60 is just below the default 64 for HardwareSerial so it can preempt if needed
    attachInterruptVector(IRQ_LPUART8, LPUART8_RX_ISR);
    NVIC_SET_PRIORITY(IRQ_LPUART8, 60); 
    NVIC_ENABLE_IRQ(IRQ_LPUART8);

}


void LPUART8_Stop(void)
{
    // Flush any pending TX data, then close the peripheral.
    // Note: calling end() and begin() rapidly can cause a brief glitch on the
    // RX line; prefer using the Start_Record / IDLING state guard instead.
    if (_serial == nullptr) return;
    _serial->end();
}


void LPUART8_Restart(void)
{
    if (_serial == nullptr) return;
    _serial->begin(LPUART8_BAUD_RATE);
}


// ----------------------------------------------------------------------------
//
//  BYTE-LEVEL I/O
//
// ----------------------------------------------------------------------------

void LPUART8_OutChar(uint8_t byte)
{

    // wait for TX data register empty
    while (!(IMXRT_LPUART8.STAT & LPUART_STAT_TDRE));

    // directly assign to the hardware DATA register
    IMXRT_LPUART8.DATA = (uint32_t)(byte & BYTE_0_MASK);

}


uint8_t LPUART8_InChar(void)
{
    if (_serial == nullptr) return 0;
    while (!_serial->available());
    return (uint8_t)_serial->read();
}


void LPUART8_OutString(const char* str)
{
    // str is a pointer to a null-terminated string, so we can loop until we
    // hit the null terminator
    while (*str) {
        LPUART8_OutChar(*str);
        str++;
    }
}

void LPUART8_InString(uint8_t* buffer, size_t length)
{
    while (length--) {
        *buffer++ = LPUART8_InChar();
    }
}


// ----------------------------------------------------------------------------
//
//  BYTE PROCESSOR
//
// ----------------------------------------------------------------------------

void LPUART8_ProcessByte(uint8_t b)
{
    static int32_t length = 0;


        
    // Check if the received character is a button command from the EUSCI_A2
    if (b == '!') {

        uart_buffer_pointer = RX_POINTER;
        length = 4; // max command length is 4 (e.g. "!E\r\n")

    } else if (b == '#') {

        uart_buffer_pointer = RX_POINTER;
        length = 2 + sizeof(state_se2_t); // 14 bytes for the pose struct

    }
    

    // otherwise, add the character to the buffer and increment the pointer
    *(uart_buffer_pointer++)    = b;
    length--;

#ifdef DEBUG_OUTPUT
    Serial.printf("b=0x%02X, l=%u\n", b, length);
#endif

    // early return if the message is incomplete e.g. nonzero length
    // if (!length)
    if (length > 0)
        return;

#ifdef DEBUG_OUTPUT
    Serial.printf("msg=%14s\n", UART8_buffer);
#endif

    (*task_function)(RX_POINTER);

    memset((void*)UART8_buffer, 0, UART8_BUFFER_SIZE);
    uart_buffer_pointer = RX_POINTER;
    length = 0;

}

state_se2_t Get_State_Request(void) {

    state_se2_t state = {0.0f, 0.0f, 0.0f};

    // alternatively, assign the first 2 bytes which are the command prefix "#S"
    // state_se2_t state = *((state_se2_t*)(UART8_buffer + 2)); 
    memcpy(&state, (const void*)(&UART8_buffer[0] + 2), sizeof(state_se2_t));

    return state;
}


// ----------------------------------------------------------------------------
//
//  UART COMMON FUNCTIONS
//
// ----------------------------------------------------------------------------

uint8_t Check_UART_Data(
        volatile char  UART_Data_Buffer[],
           const char* data_string)
{
    if (strstr((const char*)UART_Data_Buffer, data_string) != NULL) {

        return 0x01;

    }

    return 0x00;

}
