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
 * @param base_ptr       Pointer to the raw data buffer.
 * @param distance_angle Array to store the converted distance and angle values (radians).
 */
//static uint8_t to_distance_angle(
//        uint8_t    *base_ptr,
//        float       distance_angle[2]);

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


#endif /* __INC_RPLIDAR_C1_H__ */
