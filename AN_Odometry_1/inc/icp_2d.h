/**
 * @file        ICP_2D.h
 * @brief  
 */

#ifndef __INC_ICP_2D_H__
#define __INC_ICP_2D_H__


#include "data_structures.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>


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
        const int       target_size,
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
        int         n,
        Point2D*    centroid);


/**
 * @brief main ICP function for 2D point clouds
 * 
 * @param[in] source 
 * @param[in] source_size 
 * @param[in] target 
 * @param[in] target_size 
 * @param[in] max_iterations 
 * @param[in] tolerance 
 * @param[out] out_R 
 * @param[out] out_t 
 */
void ICP_2d(
        Point2D* source, int source_size,
        Point2D* target, int target_size,
        int     max_iterations,
        float   tolerance,

        // 2x2 rotation matrix (row-major)
        float*  out_R,

        // 2x1 translation vector
        float*  out_t);


#endif // __INC_ICP_2D_H__
