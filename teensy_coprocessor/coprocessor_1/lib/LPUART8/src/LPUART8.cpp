/**
 * @file RPLiDAR_UART.cpp
 * @brief Teensy HAL implementation for the RPLiDAR C1 UART driver.
 *
 * @details This is a direct port of RPLiDAR_A2_UART.c from the MSP432
 *  FW_RPLiDAR_C1 project.  The FSM action functions (Hold_Action,
 *  Find_Pattern_Action, Add_Offset_Action, Skip_Action, Record_Action) and
 *  all helper functions are lifted verbatim — they contain zero
 *  hardware-specific code.
 *
 * @author Gian Fajardo
 */

#include "LPUART8.h"


// ----------------------------------------------------------------------------
//
//  MODULE-LEVEL STATE
//
// ----------------------------------------------------------------------------

/** Bound serial port — set by LPUART8_SetPort() */
static HardwareSerial* _serial = nullptr;

/** Pointer into the raw 5-byte staging buffer */
uint8_t*  RX_POINTER    = nullptr;

/** Pointer into the packed (angle << 16 | distance) buffer */
uint32_t* INTERM_POINTER = nullptr;

/** Flag set by Record_Action when a valid packed word is ready for filtering */
static uint32_t process_data_flag = 0;

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


// ----------------------------------------------------------------------------
//
//  PRIVATE BUFFER STORAGE
//
// ----------------------------------------------------------------------------

/** Raw staging buffer — holds FIND_INDEX bytes during pattern search */
static uint8_t  UART_buffer[FIND_INDEX];


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
// FIFO drain: the iMXRT1062 LPUART8 has a 4-byte RX FIFO.  HardwareSerial
// configures the watermark to 2, so RDRF fires when ≥2 bytes are waiting.
// Reading (WATER >> 24) & 0x7 gives the exact count so we drain all
// available bytes in one ISR invocation — equivalent to the MSP432 scheme
// where every byte triggers EUSCIA2_IRQHandler individually, but without
// the extra interrupt overhead per byte.

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
    if (IMXRT_LPUART6.STAT & LPUART_STAT_OR) {
        IMXRT_LPUART6.STAT |= LPUART_STAT_OR;
    }

    // ---- FIFO drain --------------------------------------------------------
    // Read the first DATA word atomically: LPUART_DATA_RXEMPT (bit 12) is
    // set when the FIFO was already empty at the time of the read.  With
    // ILIE disabled, this ISR fires only due to RDRF, so the first read
    // almost always returns a valid byte; the counter is just a safety net.
    d   = IMXRT_LPUART6.DATA;

    // this condition 
    #define DATA_EMPTY_CONDITION (d & LPUART_DATA_RXEMPT)

    
    if ( !DATA_EMPTY_CONDITION ) {

        // process each byte sequentially from the FIFO and then reassign d to the next DATA word atomically, so the loop can exit when the FIFO is empty via reading the RXEMPT bit
        do {

            LPUART8_ProcessByte( (uint8_t)(d & BYTE_0_MASK) );
            d   = IMXRT_LPUART6.DATA;

        } while ( !DATA_EMPTY_CONDITION );

    }

}


// ----------------------------------------------------------------------------
//
//  UART LIFECYCLE
//
// ----------------------------------------------------------------------------

void LPUART8_Init(void)
{
    if (_serial == nullptr) return;

    // Let HardwareSerial configure baud, pins, FIFO, and CTRL.RIE/ILIE.
    // Do NOT replace the IRQ vector here — the protocol init commands
    // (STOP / RESET / GET_HEALTH / SCAN) are transmitted after this call
    // using HardwareSerial's TX path, which needs its own ISR to drain the
    // TX ring buffer.  Call LPUART8_AttachISR() from setup() once all
    // init commands have been sent.
    _serial->begin(LPUART8_BAUD);
}


void LPUART8_AttachISR(void)
{
    if (_serial == nullptr) return;

    // _serial->flush();  // wait for any pending TX to finish before hijacking the vector

    // Wait-blocks for TX FIFO to empty (WATER[19:16] = TXCOUNT)
    //  - `7u`: this register holds 0b111 bits
    while (IMXRT_LPUART6.WATER & LPUART_WATER_TXCOUNT(7u));

    // Then wait for shift register to finish
    while (!(IMXRT_LPUART6.STAT & LPUART_STAT_TC));


    /** -----------------------------------------------------------------------
     * @brief Disable the transmitter and receiver before making other CTRL
     *  changes
     */
    IMXRT_LPUART6.CTRL &= ~(    LPUART_CTRL_TE
                            |   LPUART_CTRL_RE);

    // Explicitly clear TX interrupt enables AND the idle line interrupt enable (ILIE) so no spurious interrupt can fire. HardwareSerial enables ILIE to detect end-of-packet which is not needed for this application. Leaving it enabled causes ~2 400 useless ISR invocations per 100 ms frame (one per inter-packet idle gap at 4 090 packets/sec). Worse, the IDLE ISR path performs a read-modify-write on STAT to clear LPUART_STAT_IDLE; if STAT.OR (receive overrun) happens to be set at that moment, |= silently clears it — hiding the evidence of the very byte-drop that causes FSM drift. Our application only needs the RXINT so the FSM can properly count bytes internally.
    IMXRT_LPUART6.CTRL &= ~(    LPUART_CTRL_TIE
                            |   LPUART_CTRL_TCIE
                            |   LPUART_CTRL_ILIE);

    // Clear any IDLE and OR flags left pending from the init-command phase.
    IMXRT_LPUART6.STAT |= (     LPUART_STAT_IDLE
                            |   LPUART_STAT_OR);


    /** -----------------------------------------------------------------------
     * @brief after every changes made, it is safe to re-enable the receiver
     */
    IMXRT_LPUART6.CTRL |=       LPUART_CTRL_RE;

    // From this point all LPUART6 interrupts are handled by LPUART6_RX_ISR
    // with priority 64 (HardwareSerial default). 
    attachInterruptVector(IRQ_LPUART6, LPUART6_RX_ISR);
    NVIC_SET_PRIORITY(IRQ_LPUART6, 64);  // match HardwareSerial default
    NVIC_ENABLE_IRQ(IRQ_LPUART6);

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
    _serial->begin(LPUART8_BAUD);
}


// ----------------------------------------------------------------------------
//
//  BYTE-LEVEL I/O
//
// ----------------------------------------------------------------------------

void LPUART8_OutChar(uint8_t byte)
{
    if (_serial == nullptr) return;
    _serial->write(byte);
}


uint8_t LPUART8_InChar(void)
{
    if (_serial == nullptr) return 0;
    while (!_serial->available());      // spin — same as the MSP432 IFG poll
    return (uint8_t)_serial->read();
}


// ----------------------------------------------------------------------------
//
//  BYTE PROCESSOR
//
// ----------------------------------------------------------------------------

void LPUART8_ProcessByte(uint8_t b)
{
    // Check if the received character is a button command from the EUSCI_A2
    if (b == '!') {

        uart_buffer_pointer = RX_POINTER;

    }

    // otherwise, add the character to the buffer and increment the pointer
    *(uart_buffer_pointer++)    = b;


    // early return if the message is incomplete
    if (!Check_UART_Data(RX_POINTER, "\r\n"))
        return;

    
    (*task_function)(RX_POINTER);

}


// ----------------------------------------------------------------------------
//
//  PRIVATE HELPER IMPLEMENTATIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Check whether msg_ptr points to a valid RPLiDAR C1 5-byte packet
 *        start.
 *
 * @details From the RPLiDAR C1 Interface Protocol (p. 15):
 *  - Byte 0, bits [1:0]: start flag bits — must be 01 or 10 (complements).
 *  - Byte 1, bit  [0]  : check flag — must be 1.
 */
static inline uint8_t pattern(const uint8_t* msg_ptr)
{
    uint8_t start_bits = msg_ptr[0] & 0x03;

    return     ((start_bits == 0x01) || (start_bits == 0x02))
            && ((msg_ptr[1] & 0x01) == 0x01);
}

