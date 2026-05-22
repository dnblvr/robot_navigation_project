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
#include <string.h>
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



// check condition
#ifdef DEBUG_OUTPUT
  #define PRINT_CHECK (iter >= 0)
#endif


/**
 * @brief // Maximum number of points ICP should handle
 */
#define ICP_MAX_POINTS  OUTPUT_BUFFER

/**
 * @brief max unsigned integer value of an index in the `icp_correspondences` buffer, which is statically allocated with type `index_t`. This is used to check that the type can hold all possible indices for the target cloud.
 */
#define CORRESPONDENCE_MAX_INDEX ((1ULL << (8*sizeof(index_t))) - 1)

/**
 * @brief Maximum correspondence quality expressed in distance (mm) - pairs
 *  further than this are rejected
 */
#define ICP_MAX_CORR_DIST       200.0f

/**
 * @brief Maximum correspondence quality expressed in distance squared (mm^2)
 */
#define ICP_MAX_CORR_DIST_SQ    (ICP_MAX_CORR_DIST * ICP_MAX_CORR_DIST)


/**
 * @brief max range for points to be included in ICP processing
 */
#define MAX_ICP_RANGE 2500.f 



typedef struct {

    Point2D  icp_src_trans[ICP_MAX_POINTS];
    uint8_t icp_correspondences[ICP_MAX_POINTS];
    float icp_corr_dist_sq[ICP_MAX_POINTS];

} ICP_CorrespondenceCache;


// ----------------------------------------------------------------------------
// 
//  ICP HELPER FUNCTIONS
// 
// ----------------------------------------------------------------------------


/**
 * @brief 
 */
ICP_CorrespondenceCache* ICP_get_cache();


/**
 * @brief Find the index of the closest point in target for a given source
 *      point.
 * @param[in] src           source *point*
 * @param[in] target        pointer to target point *array*
 * @param[in] target_size   number of points in target array
 * @param[out] out_dist_sq  output: squared distance to closest point
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
 * @param[in] pts       pointer to point array
 * @param[in] n         number of points in array
 * @param[out] centroid output: computed centroid
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
 * @param[out] out_R    2x2 rotation matrix (row-major)
 * @param[out] out_t    2x1 translation vector
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


/**
 * @brief main *improved* ICP function for 2D point clouds
 * 
 * @param[in] source        pointer to source point array
 * @param[in] source_size   number of points in source array
 * @param[in] target        pointer to target point array
 * @param[in] target_size   number of points in target array
 * @param[in] max_iteration maximum number of ICP iterations
 * @param[in] tolerance     convergence tolerance
 * 
 * @param[out] out_R    2x2 rotation matrix (row-major)
 * @param[out] out_t    2x1 translation vector
 */
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
