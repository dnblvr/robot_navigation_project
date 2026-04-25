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
 *  The only things that changed from the MSP432 version:
 *
 *   | MSP432                        | This file                          |
 *   | :---------------------------: | :--------------------------------: |
 *   | EUSCI_A2 register writes      | HardwareSerial* _serial            |
 *   | NVIC / EUSCIA2_IRQHandler()   | RPLiDAR_ProcessByte(uint8_t b)     |
 *   | Clock_Delay1ms(n)             | delay(n)                           |
 *   | Timer_A1_Ignore()             | _timer_ignore_stub()  (no-op)      |
 *   | Timer_A1_Acknowledge()        | _timer_ack_stub()     (no-op)      |
 *
 * @author Gian Fajardo
 */

#include "RPLiDAR_UART.h"

// Forward-declaration: defined in Timer_A1_Tasks.h (included by main via
// rplidar_impl.h).  RPLiDAR_UART.cpp manipulates this flag from inside
// the ISR without pulling in the full scheduler header, which would cause
// duplicate-definition link errors.
extern volatile uint8_t timer_ignore_flag;


// ----------------------------------------------------------------------------
//
//  MODULE-LEVEL STATE
//
// ----------------------------------------------------------------------------

/** Bound serial port — set by RPLiDAR_UART_SetPort() */
static HardwareSerial* _serial = nullptr;

/** Non-owning pointer to the caller's C1_States struct */
C1_States* config       = nullptr;

/** Pointer into the raw 5-byte staging buffer */
uint8_t*  RX_POINTER    = nullptr;

/** Pointer into the packed (angle << 16 | distance) buffer */
uint32_t* INTERM_POINTER = nullptr;

/** Flag set by Record_Action when a valid packed word is ready for filtering */
static uint32_t process_data_flag = 0;

/**
 * @brief Current raw byte being processed by the FSM.
 *
 * @details In the MSP432 version this was set in EUSCIA2_IRQHandler() from
 *  RXBUF.  Here it is set inside RPLiDAR_ProcessByte() before dispatching.
 */
static uint32_t data = 0;

/**
 * @brief Byte-offset correction found during FIND_PATTERN.
 *
 * @details Non-zero when the 5-byte packet boundary is misaligned; Add_Offset
 *  discards exactly `offset` bytes to re-align.
 */
static uint32_t offset = 0;

/**
 * @brief   byte-mask for the LPUART DATA register
 * @details has other accessible bits stored which must be masked to get the FIFO-drainable byte value  
 */
#define BYTE_0_MASK 0xFFu


// ----------------------------------------------------------------------------
//
//  PRIVATE BUFFER STORAGE
//
// ----------------------------------------------------------------------------

/** Raw staging buffer — holds FIND_INDEX bytes during pattern search */
static uint8_t  lidar_uart_buffer[FIND_INDEX];

/** Packed angle-distance buffer — holds up to PROCESS_BUFFER_SIZE words */
static uint32_t angle_distance_buffer[PROCESS_BUFFER_SIZE];


// ----------------------------------------------------------------------------
//
//  PRIVATE HELPER PROTOTYPES
//
// ----------------------------------------------------------------------------

static inline uint8_t  pattern(const uint8_t* msg_ptr);
static inline uint32_t Compact_Data(volatile uint8_t* msg_ptr);
static void            Reset_State(void);
static void            End_Record(void);


// ----------------------------------------------------------------------------
//
//  TIMER STUBS
//
// ----------------------------------------------------------------------------

// These write timer_ignore_flag, which is read by Task_Selector() (in the
// IntervalTimer ISR) to suppress task_flag assignments while the FSM is
// mid-recording.  A volatile byte write is safe from interrupt context —
// no need to stop/restart the IntervalTimer itself.

static inline void _timer_ignore(void)      { timer_ignore_flag = 1; }
static inline void _timer_acknowledge(void) { timer_ignore_flag = 0; }


// ----------------------------------------------------------------------------
//
//  DEFAULT ANGLE FILTER
//
// ----------------------------------------------------------------------------

uint8_t Scan_All(uint32_t /*data*/) { return 1; }


// ----------------------------------------------------------------------------
//
//  PORT INJECTION
//
// ----------------------------------------------------------------------------

void RPLiDAR_UART_SetPort(HardwareSerial* port) {
    _serial = port;
}


// ----------------------------------------------------------------------------
//
//  PUBLIC CONFIGURATION FUNCTIONS
//
// ----------------------------------------------------------------------------

void Configure_RPLiDAR_Struct(const C1_States* input_config)
{
    // Store a non-const pointer from the const input pointer.
    // Caller must pass a long-lived writable C1_States instance.
    config = const_cast<C1_States*>(input_config);

    // Point the module-level pointers at the static buffers
    RX_POINTER      = lidar_uart_buffer;
    INTERM_POINTER  = angle_distance_buffer;

    Reset_State();

    config->isr_counter     = 0;
    process_data_flag       = 0;

}


void Start_Record(Angle_Filter filter)
{
    // Install the angle filter (NULL --> accept all)
    config->angle_filter = (filter == nullptr) ? &Scan_All : filter;

    // Only arm if we are currently idling
    if (config->current_state == IDLING)
        config->current_state = READY;
}


// ----------------------------------------------------------------------------
//
//  BARE-METAL LPUART6 RX ISR
//
// ----------------------------------------------------------------------------

// Strategy: call Serial1.begin(RPLIDAR_BAUD) to let HardwareSerial configure
// the baud rate, I/O pins, FIFO, and CTRL enable bits, then overwrite the
// single function-pointer HardwareSerial installed in _VectorsRam with our
// own handler.  The hardware configuration is identical — we only change
// who is called on each interrupt.
//
// FIFO drain: the iMXRT1062 LPUART6 has a 4-byte RX FIFO.  HardwareSerial
// configures the watermark to 2, so RDRF fires when ≥2 bytes are waiting.
// Reading (WATER >> 24) & 0x7 gives the exact count so we drain all
// available bytes in one ISR invocation — equivalent to the MSP432 scheme
// where every byte triggers EUSCIA2_IRQHandler individually, but without
// the extra interrupt overhead per byte.

FASTRUN void LPUART6_RX_ISR(void)
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
    

    while ( !DATA_EMPTY_CONDITION ) {

        RPLiDAR_ProcessByte( (uint8_t)(d & BYTE_0_MASK) );
        d   = IMXRT_LPUART6.DATA;

    }

}

// ----------------------------------------------------------------------------
//
//  UART LIFECYCLE
//
// ----------------------------------------------------------------------------

void RPLiDAR_UART_Init(void)
{
    if (_serial == nullptr) return;

    // Let HardwareSerial configure baud, pins, FIFO, and CTRL.RIE/ILIE.
    // Do NOT replace the IRQ vector here — the protocol init commands
    // (STOP / RESET / GET_HEALTH / SCAN) are transmitted after this call
    // using HardwareSerial's TX path, which needs its own ISR to drain the
    // TX ring buffer.  Call RPLiDAR_UART_AttachISR() from setup() once all
    // init commands have been sent.
    _serial->begin(RPLIDAR_BAUD);
}


void RPLiDAR_UART_AttachISR(void)
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


void RPLiDAR_UART_Stop(void)
{
    // Flush any pending TX data, then close the peripheral.
    // Note: calling end() and begin() rapidly can cause a brief glitch on the
    // RX line; prefer using the Start_Record / IDLING state guard instead.
    if (_serial == nullptr) return;
    _serial->end();
}


void RPLiDAR_UART_Restart(void)
{
    if (_serial == nullptr) return;
    _serial->begin(RPLIDAR_BAUD);
}


// ----------------------------------------------------------------------------
//
//  BYTE-LEVEL I/O
//
// ----------------------------------------------------------------------------

void RPLiDAR_UART_OutChar(uint8_t byte)
{
    if (_serial == nullptr) return;
    _serial->write(byte);
}


uint8_t RPLiDAR_UART_InChar(void)
{
    if (_serial == nullptr) return 0;
    while (!_serial->available());
    return (uint8_t)_serial->read();
}


// ----------------------------------------------------------------------------
//
//  FSM ACTION FUNCTIONS
//
// ----------------------------------------------------------------------------

static void Hold_Action(void)
{
    // early-return if isr_counter has not yet reached `WAIT_INDEX`
    config->isr_counter++;
    if (config->isr_counter < WAIT_INDEX)   return;

    // otherwise, transition to FIND_PATTERN and prepare for the search
    config->isr_counter = 0;

    if (config->current_state == READY) {

        _timer_ignore();

        config->current_state   = RECORDING;
        config->  limit_status  = FIND_PATTERN;
        config-> buffer_pointer = RX_POINTER;
    }
}


static void Find_Pattern_Action(void)
{
    // instantiate the pattern-matching variables
    uint32_t found = 0;


    // Write the incoming byte into the staging buffer
    *(config->buffer_pointer++) = (uint8_t)data;


    // early-return if isr_counter does not exceed `FIND_INDEX` 
    config->isr_counter++;
    if (config->isr_counter < FIND_INDEX)   return;


    // otherwise, reset the counter and search 
    config->isr_counter = 0;


    // pattern-search
    for (offset = 0; offset < MSG_LENGTH; offset++) {

        // Check all 4 packet starts within the 20-byte window
        if ( pattern(RX_POINTER + MSG_LENGTH*0 + offset) ) {
            found =     pattern(RX_POINTER + MSG_LENGTH*1 + offset)
                     && pattern(RX_POINTER + MSG_LENGTH*2 + offset)
                     && pattern(RX_POINTER + MSG_LENGTH*3 + offset);
        }

        if (!found) continue;

        // Assuming `offset` is found at 0, it transitions to `RECORD` as the
        // pattern is already aligned; otherwise it transitions to `ADD_OFFSET`
        // to discard the first `offset` bytes.
        config->limit_status = (offset == 0) ? RECORD : ADD_OFFSET;
        break;
    }

    // Prepare the common states for all three branches
    config->buffer_pointer          = RX_POINTER;
    config->interm_buffer_pointer   = INTERM_POINTER;
    config->interm_buffer_counter   = 0;
}


static void Add_Offset_Action(void)
{
    // early-return if isr_counter has not yet reached `offset`
    config->isr_counter++;
    if (config->isr_counter < offset)   return;

    // otherwise, reset the counter and begin recording
    config->isr_counter  = 0;
    config->limit_status = RECORD;
}


static void Skip_Action(void)
{
    // In-motion processing: min/max angle rejection
    if (process_data_flag) {

        process_data_flag = 0;

        if ( config->angle_filter(*config->interm_buffer_pointer) ) {
            config->interm_buffer_pointer++;
            config->interm_buffer_counter++;
        }
    }

    // early-return if isr_counter has not yet reached `SKIP_INDEX`
    config->isr_counter++;
    if (config->isr_counter < SKIP_INDEX)   return;

    // otherwise, reset the counter and wait, or process the frame if full
    config->isr_counter = 0;


    if (config->interm_buffer_counter > PROCESS_BUFFER_SIZE) {

        // Scan frame is full — hand off to application

        End_Record();
        _timer_acknowledge();               // MSP432: Timer_A1_Acknowledge()

    } else {

        config->limit_status = RECORD;
    }
}


static void Record_Action(void)
{
    // Write the incoming byte into the staging buffer
    *(config->buffer_pointer++) = (uint8_t)data;

    // early-return if isr_counter has not yet reached `MSG_LENGTH`
    config->isr_counter++;
    if (config->isr_counter < MSG_LENGTH)   return;

    // otherwise, reset the counter and prepare for the next packet
    config->isr_counter     = 0;
    config->buffer_pointer  = RX_POINTER;
    config->limit_status    = SKIP;


    // Zero-distance filter: both distance bytes must be non-zero
    if (config->buffer_pointer[4] && config->buffer_pointer[3]) {

        *(config->interm_buffer_pointer) = Compact_Data(config->buffer_pointer);

        process_data_flag = 1;
    }
}


// ----------------------------------------------------------------------------
//
//  FSM DISPATCH TABLE
//
// ----------------------------------------------------------------------------

struct GPS_State { void (* const action)(void); };
typedef const struct GPS_State GPS_State_t;

static GPS_State_t FSM_Table[5] = {
    { &Hold_Action          },  // HOLD
    { &Find_Pattern_Action  },  // FIND_PATTERN
    { &Add_Offset_Action    },  // ADD_OFFSET
    { &Skip_Action          },  // SKIP
    { &Record_Action        }   // RECORD
};


// ----------------------------------------------------------------------------
//
//  BYTE PROCESSOR
//
// ----------------------------------------------------------------------------

void RPLiDAR_ProcessByte(uint8_t b)
{
    if (config == nullptr) return;

    data = b;
    FSM_Table[config->limit_status].action();
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


/**
 * @brief Pack a 5-byte scan message into one 32-bit angle-distance word.
 *
 * @details Encoding (Q fixed-point):
 *  - Bits [31:16]: Q9.6  angle   (raw, 1/64 degree LSB)
 *  - Bits [15: 0]: Q14.2 distance (raw, 0.25 mm LSB)
 *
 * @note The same encoding is decoded in Process_RPLiDAR_Data().
 */
static inline uint32_t Compact_Data(volatile uint8_t* msg_ptr)
{
    return   ( (uint32_t) msg_ptr[2]         << 23 )
           | ( (uint32_t)(msg_ptr[1] & 0x7F) << 16 )
           | ( (uint32_t) msg_ptr[4]         <<  8 )
           | ( (uint32_t) msg_ptr[3]         <<  0 );
}


static void Reset_State(void)
{
    config->current_state           = IDLING;
    config->  limit_status          = HOLD;
    config-> buffer_pointer         = RX_POINTER;
    config-> interm_buffer_pointer  = INTERM_POINTER;
    config-> interm_buffer_counter  = 0;
}


static void End_Record(void)
{
    config->current_state   = PROCESSING;
    config->  limit_status  = HOLD;
    config-> buffer_pointer = RX_POINTER;
}
