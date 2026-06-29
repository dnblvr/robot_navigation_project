/**
 * @file ICP_2D.h
 * @author Gian Fajardo (gianfajardo.prim@gmail.com)
 * @brief 
 * @version 0.1
 * 
 */

#ifndef __ICP_2D_H__
#define __ICP_2D_H__


#include <state_se2.h>
#include <data_structures.h>

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <assert.h>


#ifdef __IMXRT1062__
  #include <Arduino.h>
  #include <arm_math.h>

  #define PRINTF (Serial.printf)

#else
  #include <math.h>

  #define PRINTF (printf)

#endif


#ifdef __cplusplus
extern "C" {
#endif

// ────────────────────────────────────────────────────────────────────────────
// 
//  CONSTANTS
// 
// ────────────────────────────────────────────────────────────────────────────


// check condition
#ifdef DEBUG_OUTPUT
  #define PRINT_CHECK (iter >= 15)
#endif

/**
 * @brief Maximum number of points that ICP should handle
 */
#define ICP_MAX_POINTS  OUTPUT_BUFFER

/**
 * @brief Maximum range for points to be included in ICP convergence processing
 */
#define MAX_ICP_RANGE   2500.f

/**
 * @brief Maximum correspondence quality expressed in distance (mm)
 */
#define ICP_MAX_CORR_DIST       400.f


/**
 * @brief Maximum correspondence quality expressed in distance squared (mm^2)
 */
#define ICP_MAX_CORR_DIST_SQ    (ICP_MAX_CORR_DIST * ICP_MAX_CORR_DIST)


// ────────────────────────────────────────────────────────────────────────────
// 
//  DATA STRUCTURES
// 
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief this type is used for storing correspondences in the ICP
 *  algorithm. Must be able to hold an index for any point in the target cloud,
 *  which has at most `ICP_MAX_POINTS` points.
 */
typedef uint8_t index_t;

/**
 * @brief max unsigned integer value of an index in the `icp_correspondences`
 *  buffer, which is statically allocated with type `index_t`. This is used to
 *  check that the type can hold all possible indices for the target cloud.
 */
#define CORRESPONDENCE_MAX_INDEX    ( (1ULL << (8*sizeof(index_t))) - 1 )

// static assertion to check that the type used for correspondences can hold
// all possible indices for the target cloud
static_assert(
        CORRESPONDENCE_MAX_INDEX > (ICP_MAX_POINTS - 1),
        "current type used for each icp_correspondences entry with value must be able to hold an index for ICP_MAX_POINTS");


// ────────────────────────────────────────────────────────────────────────────
// 
//  ICP HELPER FUNCTIONS
// 
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief function which returns a pointer to the internal cache of the ICP module, which includes the `icp_corr_dist_sq` buffer which stores the squared distances for each correspondence. This is used to speed up confidence computation in `slam_compute_icp_confidence_i()` by avoiding redundant nearest neighbor search.
 * 
 * @note `ICP_2D_i()` must be able to run first before retrieving the latest cache of correspondence distances, as it is the function which does post-processing of far-range points
 * 
 * @return `float` array to the `icp_corr_dist_sq` buffer
 */
float* ICP_get_cache();


/**
 * @brief helper function that finds the index of the closest point in target 
 *  given a source point.
 * 
 * @param[in] src           point of type `Point2D`
 * @param[in] target        target array of `Point2D` to search through
 * @param[in] target_size   number of points in the target array
 * 
 * @param[out] out_index    output: index of closest point in target
 * @param[out] out_dist_sq  output: squared distance to closest point
 */
void Find_Closest_Point(
        const Point2D   src,
        const Point2D*  target,
        const uint16_t  target_size,

              index_t*  out_index,
              float*    out_dist_sq);


/**
 * @brief helper function to compute centroid of a `Point2D` array set
 * 
 * @param[in] pts   `Point2D` array set
 * @param[in] n     number of points in `Point2D` array
 * 
 * @param[out] centroid computed centroid
 */
void Compute_Centroid(
        Point2D*    pts,
        uint16_t    n,
        Point2D*    centroid);


// ────────────────────────────────────────────────────────────────────────────
// 
//  PL-ICP HELPERS
// 
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief helper function that, like `Find_Closest_Point()` finds the index of the closest point in target given a source point *in addition* to finding the surface normal direction.
 * 
 * @note this function finds two nearest points, which is close enough to assume is a which is used to compute the perpendicular 
 * 
 * @param[in] src           point of type `Point2D`
 * @param[in] target        target array of `Point2D` to search through
 * @param[in] target_size   number of points in the target array
 * 
 * @param[out] q1       point that is the closest point in target to src
 * @param[out] n_hat    surface normal estimate at the closest point in target to src
 * @param[out] dist_sq  squared distance to closest point
 * @return indices of closest point in target
 */
void Find_Closest_Points(
        const Point2D   src,
        const Point2D*  target,
        const uint16_t  target_size,

        int*      index,
        Point2D*  q1,
        float*    n_hat);


/**
 * @brief Copies source to static buffer while sorting this array such that near
 * points are filled in downwards while far points are filled in from the
 * end, upward.
 * 
 * @param[in] source        array of `Point2D` to be sorted
 * @param[in] source_size   number of points in `source` array
 * @param[out] icp_src_trans        sorted output array 
 * @param[out] valid_range_limit    counter for number of points within
 *  `MAX_ICP_RANGE` in `icp_src_trans`
 */
void range_sort(
              Point2D*  source, 
        const uint16_t  source_size, 
              Point2D*  icp_src_trans,
              uint16_t* valid_range_limit);


/**
 * @brief computes the available match distance for a given iteration
 * 
 * @param[in] iteration current iteration of the ICP algorithm
 * @return `float` available match distance squared
 * 
 * @note - the `DECAYING_MATCH_DISTANCE` setting determines whether the match
 *  difference threshold will decay with each iteration
 */
inline float icp_match_distance_sq(uint8_t iteration)
{
    // #define DECAYING_MATCH_DISTANCE 1

#if defined(DECAYING_MATCH_DISTANCE) // ───────────────────────────────────────

    float range = ICP_MAX_CORR_DIST - 30.f*iteration;
    range   = fmaxf(range, 40.f);   // clamp to a minimum range

    return range*range;

#else // ──────────────────────────────────────────────────────────────────────

    return ICP_MAX_CORR_DIST_SQ;

#endif
}


/**
 * @brief 
 * 
 * @note - `valid` returns a boolean array indicating which correspondences are valid based on the distance threshold `ICP_MAX_CORR_DIST`, which is used for post-processing. instead, I will likely sort the correspondences based on distance, same as is done in `ICP_2D_i()` and use the global metric variable `valid_range`
 * 
 * @note - here, I will modify the original implementation so it will only perform the accumulation of `A^T A` and `A^T b` using centered source coordinates, hopefully in order to avoid wildly divergent rotation transformations
 * 
 * @note - for the accumulation of `AT_A`, I will use `matmul_3x1_1x3()`
 * @note - for the accumulation of `AT_b`, I will use `matmul_3x1_1x3()`
 * 
 * @param[in] src 
 * @param[in] src_centroid 
 * @param[in] num_pts 
 * @param[in] q1 
 * @param[in] n_hat 
 * 
 * @param[out] icp_corr 
 * @param[out] AT_A 
 * @param[out] AT_b 
 */
void accumulate_PL_ICP(
        const Point2D*  src,
        const Point2D   src_centroid,
        const uint16_t  src_size,
        const Point2D*  target,
        const uint16_t  target_size,

              index_t   icp_corr[ICP_MAX_POINTS],
              float     AT_A[TOTAL],
              float     AT_b[DIMS]);


/**
 * @brief 
 * 
 * @param delta_centered 
 * @param centroid 
 * @param delta_true 
 */
void denormalize_delta(
        const float     delta_centered[DIMS],
        const Point2D   centroid,
    
              float     delta_true[DIMS]);


// ────────────────────────────────────────────────────────────────────────────
// 
//  ICP MAIN FUNCTION
// 
// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief main ICP function to find the best-fit transform between two 2D point clouds `source` and `target`
 * 
 * @param[in] source        source `Point2D` array
 * @param[in] source_size   number of points in source array
 * @param[in] target        target `Point2D` array
 * @param[in] target_size   number of points in target array
 * @param[in] max_iteration maximum number of ICP iterations
 * @param[in] tolerance     tolerance amount that considers that convergence
 *  has been reached
 * 
 * @param[out] out_R_t    2x2 rotation matrix (row-major) and 2x1 translation vector
 */
__attribute__(( section(".fastrun") ))
void ICP_2D(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,

        uint8_t* num_iter,
        float    out_R_t[TOTAL]);


/**
 * @brief main *improved* ICP function for 2D point clouds.
 * 
 * This version splits points within a defined `MAX_ICP_RANGE`, one to be used for ICP convergence, and the other to be post-processed after ICP convergence to find the best possible correspondences.
 * 
 * @note Note that the Tensy 4.x has a dual-issue superscalar, along with a combined Instruction and Data Tightly Coupled Memory (TCM) setup. Each are very likely to operate at its best when the code and data are TCM and not cached in FLASHMEM. This is partly contributing to a significant speedup of this improved version of ICP, citing a 54.4 times speedup over the unimproved version in hardware-in-the-loop Teensy 4.0 simulation.
 * 
 * In light of these improvements, I will include an attribute that places this function in the `.fastrun` section, which is configured in the linker script to be placed in TCM. Though most functions are already in `.fastrun` supposedly, this attribute makes guarantees that ICP is indeed placed in TCM.
 *  
 * @param[in] source        pointer to source point array
 * @param[in] source_size   number of points in source array
 * @param[in] target        pointer to target point array
 * @param[in] target_size   number of points in target array
 * @param[in] max_iteration maximum number of ICP iterations
 * @param[in] tolerance     convergence tolerance
 * 
 * @param[out] num_iter     output: number of iterations until convergence
 * @param[out] out_R_t    2x2 rotation matrix (row-major) and 2x1 translation vector
 */
__attribute__ (( section(".fastrun") ))
void ICP_2D_i(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,

        uint8_t* num_iter,
        float    out_R_t[TOTAL]);





















/**
 * @brief 
 * 
 * @note it is recommended that the source points are the ones to be
 *  transformed while the target points are fixed in place. it is also recommended that 
 * 
 * @param source 
 * @param source_size 

 * @param target 
 * @param target_size 

 * @param max_iteration 
 * @param tolerance 
 * @param num_iter 
 * @param out_R_t 
 */
void ICP_2D_play(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,

        uint8_t* num_iter,
        float    out_R_t[TOTAL]);


#ifdef __cplusplus
}
#endif


#endif // __ICP_2D_H__
