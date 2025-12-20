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


// ----------------------------------------------------------------------------
//
//  CONSTANTS
//
// ----------------------------------------------------------------------------

/**
 * @brief final output buffer size for processing, etc
 */
#define OUTPUT_BUFFER               100


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
