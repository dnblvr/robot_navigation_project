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
  #include <arm_math.h>

  #ifdef __cplusplus
    // If compiling as C++, use Serial object
    #define PRINTF(...) (Serial.printf(__VA_ARGS__))
  #else
    // If compiling as plain C, use standard printf (Teensy redirects this to Serial)
    #include <stdio.h>
    #define PRINTF(...) (printf(__VA_ARGS__))
  #endif
  
#else
  #include <stdio.h> 
  #include <math.h>
  
  #define PRINTF(...) (printf(__VA_ARGS__))
  
#endif


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
#define OUTPUT_BUFFER               400

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
 * @brief `SE(2)` state struct for a differential-drive robot with a 2D
 *  magnetometer for heading measurements
 * 
 * @param x x-position in the plane
 * @param y y-position in the plane
 * @param theta heading angle in the plane
 */
typedef struct {
    float   x;
    float   y;
    float   theta;
} se2_t;

/**
 * @brief Point cloud data structure
 * 
 * @param points Array of 2D points in the cloud
 * @param num_pts Number of valid points in the cloud
 */
typedef struct {
    Point2D     points[OUTPUT_BUFFER];
    uint32_t    num_pts;
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
        const se2_t   pose, 
        const PointCloud*   cloud);


/**
 * @brief helper function to print the current pose and scan in a C format for debugging.
 * 
 * @param pose The pose to print.
 * @param cloud Pointer to the point cloud to print.
 */
void C_format_print(
        const se2_t   pose, 
        const PointCloud*   cloud);


void numpy_format_print(
        const se2_t   pose, 
        const PointCloud*   cloud);


// ────────────────────────────────────────────────────────────────────────────
//
//  TIMING HELPERS
//
// ────────────────────────────────────────────────────────────────────────────

#ifdef __IMXRT1062__

/**
 * @brief use the DWT cycle counter on the ARM Cortex-M7 to start a free-
 *  running timer
 * 
 * @todo fix this function so it gets rid of the unneccessary `uint32_t` output
 *  as the timer is restarted already by enabling the cycle counter
 * 
 * @return uint32_t 
 * @retval `start_time` in microseconds
 */
void start_free_running_timer(void);

/**
 * @brief Calculate elapsed time in microseconds since `start_time` using the
 *  DWT cycle counter.
 * 
 * @param[in] start_time The start time returned by `start_free_running_timer()`
 * 
 * @return `uint32_t`
 * @retval `elapsed_us` in microseconds
 * 
 * @note Cast to `uint64_t` before multiplying - `uint32_t` overflows at ~7 µs
 *  real elapsed time (4295 cycles × 1,000,000 > 2^32).
 * 
 * @note Must call `start_free_running_timer()` first to initialize DWT and
 *  avoid overflow issues.
 */
uint32_t get_elapsed_time_us();


#endif // __IMXRT1062__


#ifdef __cplusplus
}
#endif


#endif // __DATA_STRUCTURES_H__
