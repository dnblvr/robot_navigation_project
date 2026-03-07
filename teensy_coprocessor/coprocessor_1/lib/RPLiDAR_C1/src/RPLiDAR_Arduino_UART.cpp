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
// Replace these with IntervalTimer calls if you later add a timer.

static inline void _timer_ignore(void)      { /* no-op — poll from loop() */ }
static inline void _timer_acknowledge(void) { /* no-op — poll from loop() */ }


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
//  UART LIFECYCLE
//
// ============================================================================

void RPLiDAR_UART_Init(void)
{
    if (_serial == nullptr) return;
    _serial->begin(RPLIDAR_BAUD);
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
            found =     pattern(RX_POINTER + MSG_LENGTH*1 + offset)
                     && pattern(RX_POINTER + MSG_LENGTH*2 + offset);

        if (!found) continue;

        config->limit_status = (offset == 0) ? RECORD : ADD_OFFSET;
        break;
    }

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
