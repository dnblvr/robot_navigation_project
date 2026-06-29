/**
 * @file data_structures.h
 * @author your name (you@domain.com)
 * 
 * @brief Establish shared data structures for GraphSLAM in the MSP432
 *      environment.
 */
#ifndef __DATA_STRUCTURES_H__
#define __DATA_STRUCTURES_H__


#include <stdbool.h>
#include <stdint.h>
// #include <assert.h> // for static_assert()

#ifdef __IMXRT1062__
  #include <Arduino.h>
  #define PRINTF (Serial.printf)

#else
  #include <stdio.h>
  #define PRINTF (printf)

#endif

#define __FAST_MATH__ 1

#ifdef __cplusplus
extern "C" {
#endif

// ————————————————————————————————————————————————————————————————————————————
//
//  DATA STRUCTURES
//
// ————————————————————————————————————————————————————————————————————————————

/**
 * @brief Buffer lengths for UART communication, processing, etc
 */
#define OUTPUT_BUFFER   200

/**
 * @brief number of `uint32_t` needed to store OUTPUT_BUFFER bits
 * 
 * @note handles the border cases well; if container requires:
 * 
 * -  1 bit ——> the size is 1 `uint32_t`
 * 
 * - 32 bits ——> the size is 1 `uint32_t`
 * 
 * - 33 bits ——> the size is 2 `uint32_t`
 */
#define BOOL_ARRAY_SIZE ((OUTPUT_BUFFER + 31u) / 32u)

// test to ensure `BOOL_ARRAY_SIZE` is sufficient to store `OUTPUT_BUFFER` bits
#if (BOOL_ARRAY_SIZE > OUTPUT_BUFFER)
    #error "`BOOL_ARRAY_SIZE` is larger than `OUTPUT_BUFFER`. Please check the calculation of `BOOL_ARRAY_SIZE`."
#endif

/**
 * @brief 2D point representation
 * 
 * @param x X coordinate in mm
 * @param y Y coordinate in mm
 */
typedef struct {
    float   x,
            y;
} Point2D;

/**
 * @brief `se2_t` state representation (x, y, theta)
 * 
 * @param x change in X plane in mm
 * @param y change in Y plane in mm
 * @param theta change in orientation in radians
 */
typedef struct {
    float   x,
            y,
            theta;
} se2_t;


/**
 * @brief Point cloud data structure
 * 
 * @param points Array of 2D points in the cloud
 * @param num_pts Number of valid points in the cloud
 * @param above_range boolean array indicating points above max range (if needed)
 * 
 * @note uses `BOOL_ARRAY_SIZE` to determine the size of the `above_range` array
 */
typedef struct {
    Point2D     points[OUTPUT_BUFFER];
    uint32_t    num_pts;

    uint32_t    above_range[BOOL_ARRAY_SIZE];  // compressed boolean array to indicate points above max range (if needed)
} PointCloud;

/**
 * @brief ICP alignment result
 * 
 * @param dx Translation in x-axis in mm
 * @param dy Translation in y-axis in mm
 * @param dtheta Rotation change in radians
 * @param confidence Match quality [0-1]
 * @param valid Whether result is valid
 * @param num_iterations Number of ICP iterations taken to converge
 */
typedef struct {
    float   dx;
    float   dy;
    float   dtheta;
    float   confidence;
    uint8_t valid;
    uint8_t num_iterations;
} ICPResult;


// ————————————————————————————————————————————————————————————————————————————
//
//  PUBLIC HELPER FUNCTIONS
//
// ————————————————————————————————————————————————————————————————————————————


/**
 * @brief Helper function to print the current pose and scan in a format
 *  expected by the Processing visualization sketch.
 * 
 * @param pose 
 * @param cloud 
 */
void processing4_print(
        const se2_t         pose, 
        const PointCloud*   cloud);


/**
 * @brief helper function to print the current pose and scan in a C format for debugging.
 * 
 * @param pose The pose to print.
 * @param cloud Pointer to the point cloud to print.
 */
void C_format_print(
        const se2_t         pose, 
        const PointCloud*   cloud);


/**
 * @brief helper function to print the current pose and scan in a numpy format for debugging.
 * 
 * @param pose The pose to print.
 * @param cloud Pointer to the point cloud to print.
 */
void numpy_format_print(
        const se2_t         pose, 
        const PointCloud*   cloud);


#ifdef __cplusplus
}
#endif


#endif // __DATA_STRUCTURES_H__
