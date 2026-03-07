/**
 * @file RPLiDAR_C1.h
 * @brief Higher-level RPLiDAR C1 API — platform-agnostic.
 *
 * @details This header is a direct port of the MSP432 RPLiDAR_C1.h.  It
 *  provides the same public interface: initialise the scanner, issue protocol
 *  commands, and convert raw scan data into a PointCloud.
 *
 *  All hardware I/O is delegated to RPLiDAR_Arduino_UART, so this layer
 *  contains zero platform-specific code.
 *
 * @author Gian Fajardo
 */

#ifndef __INC_RPLIDAR_C1_H__
#define __INC_RPLIDAR_C1_H__


#include "RPLiDAR_Config.h"
#include "RPLiDAR_Arduino_UART.h"
#include "data_structures.h"

#include <stdint.h>
#include <math.h>

#ifdef RPLIDAR_DEBUG
#include <Arduino.h>    // Serial.printf for debug output
#endif


// ----------------------------------------------------------------------------
//
//  COMMAND DESCRIPTOR TYPES
//  (identical to the MSP432 version)
//
// ----------------------------------------------------------------------------

/**
 * @brief Descriptor for a Single-Request, No-Response command.
 *
 * @param command       Command byte sent after the 0xA5 start flag.
 * @param byte_length   Unused payload length (kept for API parity).
 * @param time          Post-command delay in milliseconds.
 */
typedef struct {
    uint8_t  command;
    uint8_t  byte_length;
    uint16_t time;
} No_Response;


/**
 * @brief Descriptor for a Single-Request, Single/Multiple-Response command.
 *
 * @param command       Command byte sent after the 0xA5 start flag.
 * @param byte_length   Number of response-descriptor bytes to read back.
 */
typedef struct {
    uint8_t command;
    uint8_t byte_length;
} Single_Response;


// ----------------------------------------------------------------------------
//
//  INITIALISATION
//
// ----------------------------------------------------------------------------

/**
 * @brief Initialise the RPLiDAR C1.
 *
 * @details Sends STOP → RESET → GET_HEALTH → SCAN in sequence, then returns.
 *  The scanner is left continuously streaming scan data.  Feed incoming bytes
 *  to RPLiDAR_ProcessByte() (from serialEventN or a polling loop) to capture
 *  them.  Call Start_Record() to arm the FSM.
 *
 *  Initialisation sequence:
 *   1. Configure the C1_States struct via Configure_RPLiDAR_Struct().
 *   2. Open UART: RPLiDAR_UART_Init().
 *   3. Issue STOP + RESET (no-response commands with guard delays).
 *   4. Issue GET_HEALTH  (single response — verifies comms).
 *   5. Issue SCAN        (begins the continuous scan data stream).
 *
 * @param config  Caller-allocated C1_States instance (must outlive the driver).
 */
void Initialize_RPLiDAR_C1(const C1_States* config);


/**
 * @brief Convert one buffered scan frame into a PointCloud.
 *
 * @details Reads from the packed angle-distance buffer populated by the FSM,
 *  optionally sorts by angle, applies SKIP_FACTOR downsampling, and computes
 *  Cartesian (x, y) coordinates.  Call only when
 *  `cfg.current_state == PROCESSING`.
 *
 * @param[out] output  Destination PointCloud.  `output->num_pts` is set to
 *                     the number of valid points written.
 */
void Process_RPLiDAR_Data(PointCloud* output);


// ----------------------------------------------------------------------------
//
//  PROTOCOL HELPERS
//
// ----------------------------------------------------------------------------

/**
 * @brief Send a single-request, no-response command and wait.
 *
 * @param cmd   Pointer to No_Response descriptor.
 */
void Single_Request_No_Response(const No_Response* cmd);


/**
 * @brief Send a command and read its response descriptor.
 *
 * @param cmd             Pointer to Single_Response descriptor.
 * @param RX_DATA_BUFFER  Caller-supplied buffer; must be >= cmd->byte_length.
 * @return 1 on success (correct 0xA5 0x5A prefix seen), 0 on failure.
 */
uint8_t Single_Request_Multiple_Response(
        const Single_Response*  cmd,
              uint8_t           RX_DATA_BUFFER[]);


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

#if (SKIP_FACTOR != 1)

/**
 * @brief In-place binary insertion sort of packed angle-distance words.
 *
 * @details Angle occupies the most-significant 16 bits, so sorting u32 by
 *  value sorts by angle.
 *
 * @param polar_data    Array of packed 32-bit words.
 * @param point_count   Number of valid entries.
 */
void binary_insertion_u32(uint32_t polar_data[], uint32_t point_count);

#endif /* SKIP_FACTOR != 1 */


#ifdef DEBUG_OUTPUT

/**
 * @brief Conditionally print the intermediary u32 buffer over Serial.
 */
void print_buffer_u32(uint8_t boolean, uint32_t limits);

#endif


#endif /* __INC_RPLIDAR_C1_H__ */
