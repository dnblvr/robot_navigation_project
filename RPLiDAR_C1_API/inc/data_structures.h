/**
 * @file data_structures.h
 * @author your name (you@domain.com)
 * 
 * @brief Establish shared data structures for GraphSLAM in the MSP432
 *      environment.
 */

#ifndef __INC_DATA_STRUCTURES_H__
#define __INC_DATA_STRUCTURES_H__


#include <stdbool.h>
#include <stdint.h>
#include <assert.h> // for stati_assert()


// ----------------------------------------------------------------------------
//
//  CONSTANTS
//
// ----------------------------------------------------------------------------


/**
 * @brief intra-buffer properties
 */
#define MSG_LENGTH                  5
#define SKIP_FACTOR                 2


/**
 * @brief Buffer lengths for UART communication, processing, etc
 */

// final output buffer size
#define OUTPUT_BUFFER               100

// float buffer after sorting
#define INTERMEDIARY_BUFFER         SKIP_FACTOR*OUTPUT_BUFFER // 200

// data filtered for
#define RPLiDAR_UART_BUFFER_SIZE    INTERMEDIARY_BUFFER*MSG_LENGTH


// check at compile-time to see if buffer size is a multiple of MSG_LENGTH
static_assert(RPLiDAR_UART_BUFFER_SIZE % MSG_LENGTH == 0, 
    "RPLiDAR_UART_BUFFER_SIZE must be a multiple of MSG_LENGTH");

// ----------------------------------------------------------------------------
//
//  DATA STRUCTURES
//
// ----------------------------------------------------------------------------


/**
 * @brief 2D point representation
 */
typedef struct {
    float   x,
            y;
} Point2D;


/**
 * @brief Pose state representation (x, y, theta)
 */
typedef struct {
    float   x,
            y,
            theta;
            
    uint32_t    timestamp;
} Pose;


/**
 * @brief Point cloud data structure
 */
typedef struct {
    Point2D     points[OUTPUT_BUFFER];
    uint32_t    num_pts;
} PointCloud;


/**
 * @brief ICP alignment result
 */
typedef struct {
    float   dx;         // Translation in x
    float   dy;         // Translation in y
    float   dtheta;     // Rotation change
    float   confidence; // Match quality [0-1]
    bool    valid;      // Whether result is valid
} ICPResult;


#endif // __INC_DATA_STRUCTURES_H__
