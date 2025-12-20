/**
 * @brief RPLiDAR_C1.h
 * @details Header file for the RPLiDAR C1 driver API.
 */

#ifndef __INC_RPLIDAR_C1_H__
#define __INC_RPLIDAR_C1_H__

#include "Project_Config.h"
#include "RPLiDAR_A2_UART.h"
#include "coordinate_transform.h"

#include <stdint.h>

#ifdef DEBUG_OUTPUT
#include <stdio.h>          // for printf() debugging
#include "inc/Profiler.h"
#endif


/**
 * @brief Command definitions for the RPLiDAR C1
 * @details These commands are used to control the RPLiDAR C1 and retrieve
 *          status and information.
 *
 *  - For the single-request, no-response commands, the format is in:
 *      {command, byte-length, time}
 */
typedef struct {
    uint8_t command,
            byte_length;
    uint16_t time;
} No_Response;


/**
 * @brief Command definitions for the RPLiDAR C1
 * @details These commands are used to control the RPLiDAR C1 and retrieve
 *          status and information.
 *
 *  - For the single-request, single-response commands, the format is in:
 *      {command, byte-length}
 */
typedef struct {
    uint8_t command,
            byte_length;
} Single_Response;


// ----------------------------------------------------------------------------
//
//  INITIALIZATION
//
// ----------------------------------------------------------------------------

/**
 * @brief Initialize the RPLiDAR C1.
 *
 * This function initializes the RPLiDAR C1 by sending the appropriate commands
 *      to the device and configuring the UART A2 interface.
 *
 * @details The process for using SCAN command:
 *  1. turn on the UART TX/RX interrupt enable
 *  2. set up C1 using the RESET, GET_HEALTH, and SCAN commands for continuous
 *      scan
 *  3. read characters at a set time determined by an external timer
 *  4. record our out-characters onto an array until it fills up
 *  5. process the data
 *  6. go to step 3
 *
 * @return None
 */
void Initialize_RPLiDAR_C1(const RPLiDAR_Config*    config);


/**
 * @brief processes all the raw 5-byte messages incoming from the RPLiDAR data
 */
void Process_RPLiDAR_Data(PointCloud*   output);

// ----------------------------------------------------------------------------
//
//  HIGHER-LEVEL RPLiDAR FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Sends a command to the RPLiDAR and waits for a single response.
 *
 * @param[in]   No_Response command
 */
void Single_Request_No_Response(const No_Response*  cmd);


/**
 * @brief Sends a command to the RPLiDAR and waits for a single response.
 *
 * @param[in]   Single_Response command
 * @param[in]   RX_DATA_BUFFER The buffer to store the received response.
 * @return uint8_t status
 */
uint8_t Single_Request_Single_Response(
        const Single_Response*   cmd,
              uint8_t RX_DATA_BUFFER[]);


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

#if (SKIP_FACTOR != 1)

/**
 * @brief binary insertion sort algorithm
 *
 * @note Good for nearly-sorted data or small datasets.
 *
 * @param[in] polar_data    32-bit concatenated angle-distance pairs
 * @param[in] point_count   Number of points to sort
 */
void binary_insertion_u32(
        uint32_t    polar_data[],
        uint32_t    point_count);

#endif  // #if (SKIP_FACTOR != 1)


#ifdef DEBUG_OUTPUT
/**
 * @brief print current u32 output from the intermediary buffer
 *
 * @param[in]	as
 * @param[in]	as
 */
void print_buffer_u32(uint8_t boolean, uint32_t limits);

#endif


#endif /* __INC_RPLIDAR_C1_H__ */
