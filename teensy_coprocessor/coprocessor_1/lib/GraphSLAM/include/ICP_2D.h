/**
 * @file        ICP_2D.h
 * @brief  
 */

#ifndef __ICP_2D_H__
#define __ICP_2D_H__


#include "data_structures.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// 
//  CONSTANTS
// 
// ----------------------------------------------------------------------------


// #define PRINTF (Serial.printf)
// #include <Arduino.h>
#define PRINTF (printf)


// ----------------------------------------------------------------------------
// 
//  ICP HELPER FUNCTIONS
// 
// ----------------------------------------------------------------------------


/**
 * @brief Find the index of the closest point in target for a given source
 *      point.
 * @param src           source *point*
 * @param target        pointer to target point *array*
 * @param target_size
 * @param out_dist_sq   output: squared distance to closest point
 * @return index of closest point in target
 */
int Find_Closest_Point(
        const Point2D   src,
        const Point2D*  target,
        const uint16_t  target_size,
        float*          out_dist_sq);


/**
 * @brief helper function to compute centroid of a point set
 * 
 * @param[in] pts 
 * @param[in] n 
 * @param[out] centroid 
 */
void Compute_Centroid(
        Point2D*    pts,
        uint16_t    n,
        Point2D*    centroid);


/**
 * @brief main ICP function for 2D point clouds
 * 
 * @param[in] source        pointer to source point array
 * @param[in] source_size   number of points in source array
 * @param[in] target        pointer to target point array
 * @param[in] target_size   number of points in target array
 * @param[in] max_iteration maximum number of ICP iterations
 * @param[in] tolerance     convergence tolerance
 * 
 * @param[out] out_R 
 * @param[out] out_t 
 */
void ICP_2D(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t  max_iteration,
        float   tolerance,

        // 2x2 rotation matrix (row-major)
        float*  out_R,

        // 2x1 translation vector
        float*  out_t);


void ICP_2D_i(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t  max_iteration,
        float   tolerance,

        // 2x2 rotation matrix (row-major)
        float*  out_R,

        // 2x1 translation vector
        float*  out_t);

#ifdef __cplusplus
}
#endif


#endif // __ICP_2D_H__
