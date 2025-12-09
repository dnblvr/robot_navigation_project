/**
 * RPLiDAR_C1.h
 *
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
 *  - For the single-request, single-response commands, the format is in:
 *      {command, byte-length}
 *
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
        const RPLiDAR_Config*   config,
        uint8_t*                RPLiDAR_RX_Data);


/**
 * @brief processes all the raw 5-byte messages incoming from the RPLiDAR data
 */
void Process_RPLiDAR_Data(
        const uint8_t   RX_DATA[RPLiDAR_UART_BUFFER_SIZE],
        float           out[OUTPUT_BUFFER][3],
        uint32_t*       point_count);




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
        uint8_t RX_DATA_BUFFER[RPLiDAR_UART_BUFFER_SIZE]);


/**
 * @brief       Get data from the RPLiDAR C1 without interrupts
 *
 * @param scan_confirmation Confirmation flag for the scan
 * @return None
 *
 * @note not needed for this particular application
 */
//void Gather_LiDAR_Data(
//        RPLiDAR_Config       *cfg,
//
//        uint8_t scan_confirmation,
//        uint8_t           RX_Data[RPLiDAR_UART_BUFFER_SIZE],
//
//        float                 out[OUTPUT_BUFFER][3]);


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Converts the raw data from the RPLiDAR to angle and distance values.
 *
 * @note The declaration/definition is in the source file.
 *
 * @param[in] base_ptr          Pointer to the raw data buffer.
 * @param[out] distance_angle   Array to store the converted distance and
 *      angle values (radians).
 */
static uint8_t to_distance_angle(
        const uint8_t*  msg_ptr,
        float           distance_angle[2]);


/**
 * @brief Insertion sort algorithm
 *
 * @note Good for nearly-sorted data or small datasets.
 *
 * @todo might be used in a more efficient implementation of the LiDAR scanner
 *
 * @param polar_data Array of [distance, angle] pairs
 * @param point_count Number of points to sort
 */
void insertion(float polar_data[][2], int point_count);


/**
 * @brief Partition function for quicksort
 */
int partition(float polar_data[][2], int low, int high);


/**
 * @brief Quick sort algorithm
 *
 * Efficient for large datasets with O(n log n) average time complexity.
 * Best choice for sorting 300 LiDAR points.
 *
 * @param polar_data Array of [distance, angle] pairs
 * @param low Starting index
 * @param high Ending index
 *
 * @note requires that partition() is active
 */
void quicksort(float polar_data[][2], int low, int high);


#endif /* __INC_RPLIDAR_C1_H__ */
