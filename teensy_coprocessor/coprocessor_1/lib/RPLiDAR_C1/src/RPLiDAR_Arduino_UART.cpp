/**
 * @file RPLiDAR_Arduino_UART.cpp
 * @brief Arduino/Teensy HAL implementation for the RPLiDAR C1 UART driver.
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
 *   |-------------------------------|------------------------------------|
 *   | EUSCI_A2 register writes      | HardwareSerial* _serial            |
 *   | NVIC / EUSCIA2_IRQHandler()   | RPLiDAR_ProcessByte(uint8_t b)     |
 *   | Clock_Delay1ms(n)             | delay(n)          (not used here)  |
 *   | Timer_A1_Ignore()             | _timer_ignore_stub()  (no-op)      |
 *   | Timer_A1_Acknowledge()        | _timer_ack_stub()     (no-op)      |
 *
 * @author Gian Fajardo
 */

#include "RPLiDAR_Arduino_UART.h"

// Forward-declaration: defined in Timer_A1_Tasks.h (included by main via
// rplidar_impl.h).  RPLiDAR_Arduino_UART.cpp manipulates this flag from inside
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
C1_States* config = nullptr;

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


// ----------------------------------------------------------------------------
//
//  PRIVATE BUFFER STORAGE
//
// ----------------------------------------------------------------------------

/** Raw staging buffer — holds FIND_INDEX bytes during pattern search */
static uint8_t  lidar_uart_buffer[FIND_INDEX];

/** Packed angle-distance buffer — holds up to PROCESS_BUFFER_SIZE words */
static uint32_t angle_distance_buffer[PROCESS_BUFFER_SIZE];


// ============================================================================
//
//  DEBUG INSTRUMENTATION  (compiled only when RPLIDAR_DEBUG is defined)
//
// ============================================================================
//
// All counters are written exclusively from ISR context so plain volatile
// uint32_t increments are race-free (ISR can't preempt itself).
// loop() reads them only while the FSM is in the PROCESSING state — during
// that window the ISR is counting HOLD bytes and does NOT touch these
// counters.  Call RPLiDAR_ResetDebugStats() immediately after printing.

#ifdef RPLIDAR_DEBUG

/** ISR fired but RXEMPT was set on the very first DATA read (FIFO empty on entry). */
volatile uint32_t dbg_isr_empty_entry  = 0;

/** Find_Pattern_Action exhausted all 5 candidate offsets without a match. */
volatile uint32_t dbg_find_fail        = 0;

/** Pattern found at byte-offset N within the 20-byte search window (index = N). */
volatile uint32_t dbg_find_offsets[5]  = {0, 0, 0, 0, 0};

/** Record_Action received a first byte whose start-flag bits [1:0] were neither 0x01 nor 0x02. */
volatile uint32_t dbg_bad_start_bit    = 0;

/** Raw bytes [0..4] of the first packet that triggered dbg_bad_start_bit. */
volatile uint8_t  dbg_bad_packet[5]    = {0, 0, 0, 0, 0};

/** Non-zero once dbg_bad_packet holds a valid capture; reset by RPLiDAR_ResetDebugStats(). */
volatile uint8_t  dbg_bad_packet_valid = 0;

/** Value of interm_buffer_counter at the moment End_Record was last called. */
volatile uint32_t dbg_last_interm      = 0;

/**
 * STAT.OR (receive overrun) was set when the ISR entered — one byte was
 * silently dropped from the FIFO.  Each increment = one lost byte = one
 * potential frame-level misalignment event.
 */
volatile uint32_t dbg_stat_or         = 0;

void RPLiDAR_ResetDebugStats(void)
{
    dbg_isr_empty_entry  = 0;
    dbg_find_fail        = 0;
    for (uint8_t _i = 0; _i < 5; _i++)
        dbg_find_offsets[_i] = 0;
    dbg_bad_start_bit    = 0;
    dbg_bad_packet_valid = 0;
    dbg_last_interm      = 0;
    dbg_stat_or          = 0;

}

#endif  // RPLIDAR_DEBUG


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
//
// On MSP432, Timer_A1_Ignore() pauses the 10 Hz task-scheduler ISR while the
// LiDAR is recording so that Process_RPLiDAR_Data() is not called mid-scan.
// On Teensy we poll `cfg.current_state == PROCESSING` from loop(), so no
// timer management is needed here.  The stubs are defined to satisfy the
// linker if any template code references them.
//
// These write timer_ignore_flag, which is read by Task_Selector() (in the
// IntervalTimer ISR) to suppress task_flag assignments while the FSM is
// mid-recording.  A volatile byte write is safe from interrupt context —
// no need to stop/restart the IntervalTimer itself.

static inline void _timer_ignore(void)      { timer_ignore_flag = 1; }
static inline void _timer_acknowledge(void) { timer_ignore_flag = 0; }


// ============================================================================
//
//  DEFAULT ANGLE FILTER
//
// ============================================================================

uint8_t Scan_All(uint32_t /*data*/) { return 1; }


// ============================================================================
//
//  PORT INJECTION
//
// ============================================================================

void RPLiDAR_UART_SetPort(HardwareSerial* port) {
    _serial = port;
}


// ============================================================================
//
//  PUBLIC CONFIGURATION FUNCTIONS
//
// ============================================================================

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
    // Install the angle filter (NULL → accept all)
    config->angle_filter = (filter == nullptr) ? &Scan_All : filter;

    // Only arm if we are currently idling
    if (config->current_state == IDLING)
        config->current_state = READY;
}


// ============================================================================
//
//  BARE-METAL LPUART6 RX ISR
//  (replaces EUSCIA2_IRQHandler on MSP432)
//
// ============================================================================
//
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
    // ---- Overrun detection -------------------------------------------------
    // STAT.OR (bit 19, W1C) is set when the 4-byte RX FIFO filled and the
    // next incoming byte was discarded.  That lost byte causes the FSM byte
    // counter to drift by one, misaligning every subsequent packet for the
    // entire frame.  We detect and count overruns so anomaly frames can be
    // correlated; clearing OR via |= may also clear other co-asserted W1C
    // flags (NF, FE, PF) — acceptable because the affected bytes are already
    // in (or lost from) the FIFO and cannot be recovered.
    //
    // ILIE has been disabled in RPLiDAR_UART_AttachISR() so the idle-line
    // flag can no longer trigger this ISR; no IDLE handling is needed here.
#ifdef RPLIDAR_DEBUG
    if (IMXRT_LPUART6.STAT & LPUART_STAT_OR) {
        dbg_stat_or++;
        IMXRT_LPUART6.STAT |= LPUART_STAT_OR;
    }
#else
    if (IMXRT_LPUART6.STAT & LPUART_STAT_OR) {
        IMXRT_LPUART6.STAT |= LPUART_STAT_OR;
    }
#endif

    // ---- FIFO drain --------------------------------------------------------
    // Read the first DATA word atomically: LPUART_DATA_RXEMPT (bit 12) is
    // set when the FIFO was already empty at the time of the read.  With
    // ILIE disabled, this ISR fires only due to RDRF, so the first read
    // almost always returns a valid byte; the counter is just a safety net.
    uint32_t d = IMXRT_LPUART6.DATA;

    

#ifdef RPLIDAR_DEBUG
    if (d & LPUART_DATA_RXEMPT) {

        // FIFO empty on entry despite RDRF — rare timing edge case.
        dbg_isr_empty_entry++;

    } else {

        RPLiDAR_ProcessByte((uint8_t)(d & 0xFFu));
        while (!((d = IMXRT_LPUART6.DATA) & LPUART_DATA_RXEMPT)) {
            RPLiDAR_ProcessByte((uint8_t)(d & 0xFFu));
        }
        
    }
#else
    if (!(d & LPUART_DATA_RXEMPT)) {
        RPLiDAR_ProcessByte((uint8_t)(d & 0xFFu));
        while (!((d = IMXRT_LPUART6.DATA) & LPUART_DATA_RXEMPT)) {
            RPLiDAR_ProcessByte((uint8_t)(d & 0xFFu));
        }
    }
#endif


}


// ============================================================================
//
//  UART LIFECYCLE
//
// ============================================================================

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

    // Block until the TX ring buffer is empty and the LPUART shift register
    // is idle.  After flush() returns, HardwareSerial has already cleared
    // CTRL.TIE, so no TX interrupt is pending.
    _serial->flush();

    // Disable the transmitter and receiver before making other CTRL changes
    IMXRT_LPUART6.CTRL &= ~(    LPUART_CTRL_TE
                            |   LPUART_CTRL_RE);

    // Belt-and-suspenders: explicitly clear TX interrupt enables AND the idle
    // line interrupt enable (ILIE) so no spurious interrupt can fire.
    //
    // Why disable ILIE:
    //   HardwareSerial enables ILIE to detect end-of-packet.  We don't need
    //   it: the FSM counts bytes internally.  Leaving it enabled causes
    //   ~2 400 useless ISR invocations per 100 ms frame (one per inter-packet
    //   idle gap at 4 090 packets/sec).  Worse, the IDLE ISR path performs a
    //   read-modify-write on STAT to clear LPUART_STAT_IDLE; if STAT.OR
    //   (receive overrun) happens to be set at that moment, |= silently clears
    //   it — hiding the evidence of the very byte-drop that causes FSM drift.
    IMXRT_LPUART6.CTRL &= ~(    LPUART_CTRL_TIE
                            |   LPUART_CTRL_TCIE
                            |   LPUART_CTRL_ILIE);

    // Clear any IDLE and OR flags left pending from the init-command phase.
    IMXRT_LPUART6.STAT |=       LPUART_STAT_IDLE
                            |   LPUART_STAT_OR;


    // after every changes made, it is safe to re-enable the receiver
    IMXRT_LPUART6.CTRL |=       LPUART_CTRL_RE;

    // Safe to replace the vector now.  From this point all LPUART6
    // interrupts are handled by LPUART6_RX_ISR.
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


// ============================================================================
//
//  BYTE-LEVEL I/O
//
// ============================================================================

void RPLiDAR_UART_OutChar(uint8_t byte)
{
    if (_serial == nullptr) return;
    _serial->write(byte);
}


uint8_t RPLiDAR_UART_InChar(void)
{
    if (_serial == nullptr) return 0;
    while (!_serial->available());      // spin — same as the MSP432 IFG poll
    return (uint8_t)_serial->read();
}


// ============================================================================
//
//  FSM ACTION FUNCTIONS   (lifted verbatim from RPLiDAR_A2_UART.c)
//
// ============================================================================

static void Hold_Action(void)
{
    config->isr_counter++;

    if (config->isr_counter < WAIT_INDEX)   return;

    config->isr_counter = 0;

    if (config->current_state == READY) {

        _timer_ignore();                        // MSP432: Timer_A1_Ignore()

        config->current_state   = RECORDING;
        config->  limit_status  = FIND_PATTERN;
        config-> buffer_pointer = RX_POINTER;
    }
}


static void Find_Pattern_Action(void)
{
    *(config->buffer_pointer++) = (uint8_t)data;

    config->isr_counter++;

    if (config->isr_counter < FIND_INDEX)   return;

    config->isr_counter = 0;

    uint32_t found = 0;

    for (offset = 0; offset < MSG_LENGTH; offset++) {

        if ( pattern(RX_POINTER + MSG_LENGTH*0 + offset) )
            // Check all 4 packet starts within the 20-byte window
            found =     pattern(RX_POINTER + MSG_LENGTH*1 + offset)
                     && pattern(RX_POINTER + MSG_LENGTH*2 + offset)
                     && pattern(RX_POINTER + MSG_LENGTH*3 + offset);

        if (!found) continue;

        config->limit_status = (offset == 0) ? RECORD : ADD_OFFSET;
        break;
    }

#ifdef RPLIDAR_DEBUG
    
    // If no pattern was found at any offset, count this as a find failure for post-mortem correlation with anomaly frames.
    if (!found) {
        dbg_find_fail++;
    // otherwise, record at which offset this was found at
    } else if (offset < (uint32_t)MSG_LENGTH) {
        dbg_find_offsets[offset]++;
    }
#endif

    config->buffer_pointer          = RX_POINTER;
    config->interm_buffer_pointer   = INTERM_POINTER;
    config->interm_buffer_counter   = 0;
}


static void Add_Offset_Action(void)
{
    config->isr_counter++;

    if (config->isr_counter < offset)   return;

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

    config->isr_counter++;

    if (config->isr_counter < SKIP_INDEX)   return;

    config->isr_counter = 0;

    if (config->interm_buffer_counter > PROCESS_BUFFER_SIZE) {

        // Scan frame is full — hand off to application
#ifdef RPLIDAR_DEBUG
        dbg_last_interm = config->interm_buffer_counter;
#endif
        End_Record();
        _timer_acknowledge();               // MSP432: Timer_A1_Acknowledge()

    } else {

        config->limit_status = RECORD;
    }
}


static void Record_Action(void)
{
    *(config->buffer_pointer++) = (uint8_t)data;

    config->isr_counter++;

    if (config->isr_counter < MSG_LENGTH)   return;

    config->isr_counter     = 0;
    config->buffer_pointer  = RX_POINTER;
    config->limit_status    = SKIP;

#ifdef RPLIDAR_DEBUG
    // Validate the start-flag bits of byte 0 of the accumulated packet.
    // A valid RPLiDAR C1 packet has bits [1:0] of byte 0 == 0x01 or 0x02
    // (alternating complement pair).  Any other value means the FSM byte
    // counter was misaligned — we are recording from the wrong position.
    {
        uint8_t sb = RX_POINTER[0] & 0x03u;
        if (sb != 0x01u && sb != 0x02u) {
            dbg_bad_start_bit++;
            if (!dbg_bad_packet_valid) {
                // Capture first offending packet verbatim for post-mortem.
                for (uint8_t _i = 0; _i < MSG_LENGTH; _i++)
                    dbg_bad_packet[_i] = RX_POINTER[_i];
                dbg_bad_packet_valid = 1;
            }
        }
    }
#endif

    // Zero-distance filter: both distance bytes must be non-zero
    if (config->buffer_pointer[4] && config->buffer_pointer[3]) {

        *(config->interm_buffer_pointer) = Compact_Data(config->buffer_pointer);

        process_data_flag = 1;
    }
}


// ----------------------------------------------------------------------------
//
//  FSM DISPATCH TABLE   (mirrors MSP432 FSM_Table[])
//
// ----------------------------------------------------------------------------

struct RPLiDAR_State { void (* const action)(void); };
typedef const struct RPLiDAR_State RPLiDAR_State_t;

static RPLiDAR_State_t FSM_Table[5] = {
    { &Hold_Action          },  // HOLD
    { &Find_Pattern_Action  },  // FIND_PATTERN
    { &Add_Offset_Action    },  // ADD_OFFSET
    { &Skip_Action          },  // SKIP
    { &Record_Action        }   // RECORD
};


// ============================================================================
//
//  BYTE PROCESSOR   (replaces EUSCIA2_IRQHandler)
//
// ============================================================================

void RPLiDAR_ProcessByte(uint8_t b)
{
    if (config == nullptr) return;

    data = b;                                       // MSP432: data = RXBUF
    FSM_Table[config->limit_status].action();       // same dispatch as ISR
}


// ============================================================================
//
//  PRIVATE HELPER IMPLEMENTATIONS
//
// ============================================================================

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
