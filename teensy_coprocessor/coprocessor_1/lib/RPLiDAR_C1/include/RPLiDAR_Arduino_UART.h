/**
 * @file RPLiDAR_Arduino_UART.h
 * @brief Arduino/Teensy HAL for the RPLiDAR C1 UART driver.
 *
 * @details This is a direct port of RPLiDAR_A2_UART.h from the MSP432
 *  FW_RPLiDAR_C1 project. The public API surface is intentionally kept
 *  identical so that higher-level code (RPLiDAR_C1.cpp) requires zero
 *  changes. Only the hardware seam is swapped:
 *
 *      MSP432 eUSCI_A2 register writes  →  HardwareSerial reference
 *      NVIC / EUSCIA2_IRQHandler        →  RPLiDAR_ProcessByte(uint8_t)
 *      Clock_Delay1ms(n)                →  delay(n)
 *      Timer_A1_Ignore / Acknowledge    →  no-op stubs (poll cfg state
 *                                          from loop() instead)
 *
 * @note Call RPLiDAR_UART_SetPort() before anything else, then
 *       RPLiDAR_UART_Init() to start the serial port at 460 800 bps.
 *
 * @note From loop() or a serialEventN() callback, feed every incoming byte
 *       to RPLiDAR_ProcessByte(b).  The FSM will handle the rest.
 *
 * @author Gian Fajardo
 */

#ifndef __RPLIDAR_ARDUINO_UART_H__
#define __RPLIDAR_ARDUINO_UART_H__


#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "RPLiDAR_Config.h"
#include "data_structures.h"


// ----------------------------------------------------------------------------
//
//  TYPES
//
// ----------------------------------------------------------------------------

/**
 * @brief Angle-rejection filter function pointer type.
 *
 * @details Receives a packed 32-bit angle-distance word (same encoding used
 *  by Compact_Data) and returns 1 to keep the point, 0 to discard.
 */
typedef uint8_t (*Angle_Filter)(uint32_t data);


/**
 * @brief Default angle filter — accepts every point.
 */
uint8_t Scan_All(uint32_t data);


// ----------------------------------------------------------------------------
//
//  STATE MACHINE ENUMERATIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Top-level RPLiDAR operational state.
 * @details Hierarchical outer layer; the inner FSM (Record_States) only runs
 *          when this is RECORDING.
 */
typedef enum {
    IDLING      = 0,
    READY,          ///< Start_Record() has been called; waiting for WAIT_INDEX
    RECORDING,      ///< Actively collecting scan bytes
    PROCESSING      ///< Scan frame complete; ready for Process_RPLiDAR_Data()
} RPLiDAR_States;


/**
 * @brief Inner byte-level FSM states for the scan acquisition pipeline.
 */
typedef enum {
    HOLD            = 0,    ///< waiting for WAIT_INDEX bytes before searching
    FIND_PATTERN,           ///< aligning to the 5-byte packet boundary
    ADD_OFFSET,             ///< correcting misalignment by discarding N bytes
    SKIP,                   ///< decimation skip phase
    RECORD                  ///< recording MSG_LENGTH bytes into the buffer
} Record_States;


// ----------------------------------------------------------------------------
//
//  DRIVER STATE STRUCTURE
//
// ----------------------------------------------------------------------------

/**
 * @brief Complete runtime state for the RPLiDAR C1 acquisition pipeline.
 *
 * @note  One global instance is created inside RPLiDAR_Arduino_UART.cpp and
 *        exposed via the config pointer.  User code should treat this opaque
 *        and interact through the public API only — except for reading
 *        `current_state` to know when a frame is ready.
 */
typedef struct {

    /** Angle-rejection callback — set by Start_Record() */
    Angle_Filter            angle_filter;

    /** Top-level state visible to application code */
    volatile RPLiDAR_States current_state;

    /** Inner FSM state, consulted by each byte handler */
    Record_States           limit_status;

    /** Byte counter used by each FSM action */
    volatile uint32_t       isr_counter;

    /** Write cursor into the raw 5-byte staging buffer */
    volatile uint8_t*       buffer_pointer;

    /** Write cursor into the packed angle-distance buffer */
    volatile uint32_t*      interm_buffer_pointer;

    /** Number of valid entries currently in the packed buffer */
    volatile uint32_t       interm_buffer_counter;

} C1_States;


// ----------------------------------------------------------------------------
//
//  PORT INJECTION
//
// ----------------------------------------------------------------------------

/**
 * @brief Bind a HardwareSerial port to the driver.
 *
 * @details Must be called before RPLiDAR_UART_Init().  The driver stores a
 *  pointer to the supplied port; the caller is responsible for keeping the
 *  object alive (e.g. pass &Serial1 which is a forever-live global).
 *
 * @param port  Pointer to any HardwareSerial instance (Serial1…Serial8 on
 *              Teensy 4.x).
 */
void RPLiDAR_UART_SetPort(HardwareSerial* port);


// ----------------------------------------------------------------------------
//
//  PUBLIC CONFIGURATION FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Initialise the C1_States struct and internal buffer pointers.
 *
 * @param input_config  Caller-allocated C1_States instance.  The driver takes
 *                      a non-owning pointer to it.
 */
void Configure_RPLiDAR_Struct(const C1_States* input_config);


/**
 * @brief Transition from IDLING → READY, arming the acquisition FSM.
 *
 * @param filter    Optional angle-rejection callback.  Pass NULL for Scan_All.
 */
void Start_Record(Angle_Filter filter);


// ----------------------------------------------------------------------------
//
//  UART LIFECYCLE  (analogous to EUSCI_A2_UART_Init/Stop/Restart)
//
// ----------------------------------------------------------------------------

/**
 * @brief Open the serial port at RPLIDAR_BAUD (460 800).
 */
void RPLiDAR_UART_Init(void);


/**
 * @brief Close the serial port.
 *
 * @note  On Teensy, HardwareSerial::end() flushes TX and disables the LPUART
 *        peripheral.  The RPLiDAR C1 motor will keep spinning.
 */
void RPLiDAR_UART_Stop(void);


/**
 * @brief Re-open the serial port at RPLIDAR_BAUD after a Stop().
 */
void RPLiDAR_UART_Restart(void);


// ----------------------------------------------------------------------------
//
//  BYTE-LEVEL I/O  (analogous to EUSCI_A2_UART_OutChar / InChar)
//
// ----------------------------------------------------------------------------

/**
 * @brief Blocking transmit of one byte to the RPLiDAR C1.
 *
 * @param data   Byte to send.
 */
void    RPLiDAR_UART_OutChar(uint8_t data);


/**
 * @brief Blocking receive of one byte from the RPLiDAR C1.
 *
 * @return Received byte.
 */
uint8_t RPLiDAR_UART_InChar(void);


// ----------------------------------------------------------------------------
//
//  BYTE PROCESSOR  (replaces EUSCIA2_IRQHandler)
//
// ----------------------------------------------------------------------------

/**
 * @brief Feed one received byte into the acquisition FSM.
 *
 * @details This is the direct replacement for EUSCIA2_IRQHandler().  Call it
 *  from a serialEventN() callback or from a polling loop:
 *
 *  @code
 *  // Option A – serialEvent (called automatically after loop())
 *  void serialEvent1() {
 *      while (Serial1.available())
 *          RPLiDAR_ProcessByte(Serial1.read());
 *  }
 *
 *  // Option B – polling from loop()
 *  while (Serial1.available())
 *      RPLiDAR_ProcessByte(Serial1.read());
 *  @endcode
 *
 * @param b  Byte freshly read from the serial port.
 */
void RPLiDAR_ProcessByte(uint8_t b);


// ----------------------------------------------------------------------------
//
//  INTERNAL DECLARATIONS  (private — not for direct use)
//
// ----------------------------------------------------------------------------

/** Raw 5-byte staging buffer and packed-word buffer (defined in .cpp) */
extern uint8_t*  RX_POINTER;
extern uint32_t* INTERM_POINTER;

/** Global config pointer dereferenced by every FSM action */
extern C1_States* config;


#endif /* __RPLIDAR_ARDUINO_UART_H__ */
