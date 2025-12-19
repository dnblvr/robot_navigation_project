/**
 * RPLiDAR_C1.h
 *
 */

#ifndef __INC_RPLIDAR_C1_H__
#define __INC_RPLIDAR_C1_H__

#include "Project_Config.h"
#include "RPLiDAR_A2_UART.h"
#include "coordinate_transform.h"
#include "data_structures.h"

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
 *
 *  - For the single-request, single-response commands, the format is in:
 *      {command, byte-length}
 */
typedef struct {
    uint8_t command,
            byte_length;
    uint16_t time;
} No_Response;

typedef struct {
    uint8_t command,
            byte_length;
} Single_Response;


// -------------------------------------------------------------------------------------
//
//  INITIALIZATION
//
// -------------------------------------------------------------------------------------


/**
 * @brief Initialize the RPLiDAR C1.
 *
 * This function initializes the RPLiDAR C1 by sending the appropriate commands
 *      to the device and configuring the UART A2 interface.
 *
 * @return None
 */

void Initialize_RPLiDAR_C1(
        const RPLiDAR_Config*   config);


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
 * @param[in] command No_Response address.
 */
void Single_Request_No_Response(
        const No_Response*  cmd);


/**
 * @brief Sends a command to the RPLiDAR and waits for a single response.
 *
 * @param command The command to send.
 * @param RX_DATA_BUFFER The buffer to store the received response.
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

/**
 * @brief binary insertion sort algorithm
 *
 * @note Good for nearly-sorted data or small datasets.
 *
 * @param polar_data array 32-bit concatenated angle-distance pairs
 * @param point_count Number of points to sort
 */
void binary_insertion_uint(
        uint32_t    polar_data[],
        uint32_t    point_count);


#endif /* __INC_RPLIDAR_C1_H__ */
