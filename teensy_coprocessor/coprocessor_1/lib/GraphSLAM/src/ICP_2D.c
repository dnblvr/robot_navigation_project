/**
 * @file ICP_2D.c
 */

#include <ICP_2D.h>

// ----------------------------------------------------------------------------
// 
//  CONSTANTS
// 
// ----------------------------------------------------------------------------

// check condition
#ifdef DEBUG_OUTPUT
  #define PRINT_CHECK (iter >= 0)
#endif


#ifndef M_PI
  #define M_PI 3.14159265359f
#endif


// Maximum number of points ICP should handle
#define ICP_MAX_POINTS  OUTPUT_BUFFER

/**
 * @brief max unsigned integer value of an index in the `icp_correspondences` buffer, which is statically allocated with type `index_t`. This is used to check that the type can hold all possible indices for the target cloud.
 */
#define CORRESPONDENCE_MAX_INDEX ((1ULL << (8*sizeof(index_t))) - 1)


// Maximum correspondence quality expressed in distance (mm) - pairs further than this are rejected
#define ICP_MAX_CORR_DIST       200.0f
#define ICP_MAX_CORR_DIST_SQ    (ICP_MAX_CORR_DIST * ICP_MAX_CORR_DIST)



// max range for points to be included in ICP processing
#define MAX_ICP_RANGE 2500.f 


// ----------------------------------------------------------------------------
// 
//  DATA STRUCTURES
// 
// ----------------------------------------------------------------------------

/**
 * @brief this is the type used for storing correspondences in the ICP
 *  algorithm. Must be able to hold an index for any point in the target cloud,
 *  which has at most `ICP_MAX_POINTS` points.
 */
typedef uint8_t index_t;

/**
 * @brief Statically-allocated buffers declared in the source file for ICP
 *  processing
 */

// stores transformed source points for processing
static Point2D  icp_src_trans[ICP_MAX_POINTS];

// stores the index of the closest point in target for each source point
static index_t  icp_correspondences[ICP_MAX_POINTS];

// stores the squared distance for each correspondence
static float    icp_corr_dist_sq[ICP_MAX_POINTS];

// static assertion to check that the type used for correspondences can hold all possible indices for the target cloud
static_assert(
        CORRESPONDENCE_MAX_INDEX > (ICP_MAX_POINTS - 1),
        "current type used for each icp_correspondences entry with value must be able to hold an index for ICP_MAX_POINTS");


// Cache variables for ICP to speed up repeated calls with the same input sizes
static uint16_t icp_cache_valid_range = 0;
static uint16_t icp_cache_source_size = 0;


// ----------------------------------------------------------------------------
// 
//  ICP HELPER FUNCTIONS
// 
// ----------------------------------------------------------------------------1

int Find_Closest_Point(
        const Point2D 	src,
        const Point2D*	target,
        const uint16_t  target_size,
              float*    out_dist_sq)
{
    // counter
    uint16_t     i;

    // properties of closest point
    float   min_dist    = FLT_MAX;
    uint16_t     min_idx     = 0;

    // iterates through target points to find the closest one
    // via brute-force search
    for (i = 0; i < target_size; i++) {

        float dx    = src.x - target[i].x;
        float dy    = src.y - target[i].y;
        float dist  = dx*dx + dy*dy;

        if (dist < min_dist) {
            min_dist    = dist;
            min_idx     = i;
        }

    }

    // Return squared distance via output parameter
    *out_dist_sq = min_dist;

    return min_idx;
}


void Compute_Centroid(
        Point2D* pts,
        uint16_t n,
        Point2D* centroid)
{
    // counter
    uint16_t i;

    // initializes centroid to (0,0)
    centroid->x = 0.0f;
    centroid->y = 0.0f;


    // add all points together and find their average
    for (i = 0; i < n; i++) {
        centroid->x += pts[i].x;
        centroid->y += pts[i].y;
    }

    centroid->x /= (float)n;
    centroid->y /= (float)n;

}


void ICP_2D(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t  max_iteration,
        float   tolerance,

        // 2x2 rotation matrix (row-major)
        float*  out_R,

        // 2x1 translation vector
        float*  out_t)
{

    // counter variables
    uint16_t iter, i;
    int valid_count;        // number of correspondences within max distance
    float prev_error;

    // Initialize transformation matrices
    float R[2][2]   = { {1, 0},
                        {0, 1} };
    float t[2]      =   {0, 0};


    // Validate input size against static buffer limits
    if (    source_size > ICP_MAX_POINTS
         || source_size <= 0)
    {

#ifdef DEBUG_OUTPUT
        PRINTF("\tICP: source_size %d exceeds max %d\n",
               source_size,
               ICP_MAX_POINTS);
#endif

        // Return identity transform
        out_R[0] = 1.0f; out_R[1] = 0.0f;
        out_R[2] = 0.0f; out_R[3] = 1.0f;

        out_t[0] = 0.0f; out_t[1] = 0.0f;
        return;
    }

    // Copy source to static buffer
    for (i = 0; i < source_size; i++) {
        icp_src_trans[i] = source[i];
    }

    /**
     * The goal is for the source points to converge to the target points
     *  iteratively.
     */
    prev_error = FLT_MAX;
    for (iter = 0; iter < max_iteration; iter++)
    {

        Point2D centroid_src, centroid_tgt;
        float S_xx, S_xy, S_yx, S_yy;
        float theta, cos_theta, sin_theta;
        float R_iter[2][2];
        float t_iter[2];
        float mean_error;
        float R_new[2][2];
        float t_new[2];
        float x, y;
        

        // print first few source and target points on first iteration
#ifdef DEBUG_OUTPUT
        if (PRINT_CHECK) {
            PRINTF("ICP iter %u: "
                   "src[0]=(%.2f,%.2f) src[1]=(%.2f,%.2f) "
                   "tgt[0]=(%.2f,%.2f) tgt[1]=(%.2f,%.2f)\n",

                   iter,
                   icp_src_trans[0].x, icp_src_trans[0].y,
                   icp_src_trans[1].x, icp_src_trans[1].y,
                   target[0].x, target[0].y,
                   target[1].x, target[1].y);
        }
#endif

        // 1. Find correspondences between each source pt. and
        //      all target points in the `target` array.
        //      Also record the squared distance for filtering.
        for (i = 0; i < source_size; i++) {
            
            icp_correspondences[i] = Find_Closest_Point(
                    icp_src_trans[i],
                    target,
                    target_size,
                    &icp_corr_dist_sq[i]);
                    
        }


        // 2. Compute centroids using only valid correspondences
        //    (distance < ICP_MAX_CORR_DIST)
        centroid_src.x  = 0;    centroid_src.y  = 0;
        centroid_tgt.x  = 0;    centroid_tgt.y  = 0;
        valid_count     = 0;

        for (i = 0; i < source_size; i++) {

            if (icp_corr_dist_sq[i] < ICP_MAX_CORR_DIST_SQ) {
                centroid_src.x += icp_src_trans[i].x;
                centroid_src.y += icp_src_trans[i].y;
                centroid_tgt.x += target[icp_correspondences[i]].x;
                centroid_tgt.y += target[icp_correspondences[i]].y;
                valid_count++;
            }

        }

        // Need at least 3 valid correspondences to compute transformation
        if (valid_count < 3) {
#ifdef DEBUG_OUTPUT
            PRINTF("ICP iter %d: only %d valid correspondences, stopping\n",
                    iter,
                    valid_count);
#endif
            break;
        }

        centroid_src.x /= valid_count;
        centroid_src.y /= valid_count;
        centroid_tgt.x /= valid_count;
        centroid_tgt.y /= valid_count;


        // 3. Compute cross-covariance matrix using only valid correspondences
        S_xx = 0;   S_xy = 0;
        S_yx = 0;   S_yy = 0;
        for (i = 0; i < source_size; i++) {

            // early-continue away from pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) {
                continue;
            }


            float x_s   = icp_src_trans[i].x - centroid_src.x;
            float y_s   = icp_src_trans[i].y - centroid_src.y;
            float x_t   = target[icp_correspondences[i]].x - centroid_tgt.x;
            float y_t   = target[icp_correspondences[i]].y - centroid_tgt.y;

            /**
             * {{S_xx, S_xy},
             *  {S_yx, S_yy}}
             */
            S_xx   += x_s * x_t;
            S_xy   += x_s * y_t;
            S_yx   += y_s * x_t;
            S_yy   += y_s * y_t;
        }


        // 4. Compute rotation (using SVD for 2x2)
        theta       = atan2f(S_xy - S_yx, S_xx + S_yy);
        
        #ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u: S_xx=%.2f S_xy=%.2f S_yx=%.2f S_yy=%.2f -> theta=%.3f rad (%.1f deg)\n",
        //         iter,
        //            S_xx, S_xy, S_yx, S_yy, theta, theta * 57.2958f);
        // }
        #endif
        
        cos_theta   = cosf(theta);
        sin_theta   = sinf(theta);

        R_iter[0][0]    =  cos_theta;
        R_iter[0][1]    = -sin_theta;
        R_iter[1][0]    =  sin_theta;
        R_iter[1][1]    =  cos_theta;


        // 5. Compute translation
        t_iter[0]   =   centroid_tgt.x
                      - (   R_iter[0][0]*centroid_src.x
                          + R_iter[0][1]*centroid_src.y);

        t_iter[1]   =   centroid_tgt.y
                      - (   R_iter[1][0]*centroid_src.x
                          + R_iter[1][1]*centroid_src.y);


        // Update transformation (compose) before transforming points
        // R_new = R_iter * R_old, t_new = R_iter * t_old + t_iter
        R_new[0][0] = R_iter[0][0]*R[0][0] + R_iter[0][1]*R[1][0];
        R_new[0][1] = R_iter[0][0]*R[0][1] + R_iter[0][1]*R[1][1];
        R_new[1][0] = R_iter[1][0]*R[0][0] + R_iter[1][1]*R[1][0];
        R_new[1][1] = R_iter[1][0]*R[0][1] + R_iter[1][1]*R[1][1];

        t_new[0]    = R_iter[0][0]*t[0] + R_iter[0][1]*t[1] + t_iter[0];
        t_new[1]    = R_iter[1][0]*t[0] + R_iter[1][1]*t[1] + t_iter[1];

        R[0][0] = R_new[0][0]; R[0][1] = R_new[0][1];
        R[1][0] = R_new[1][0]; R[1][1] = R_new[1][1];
        t[0]    = t_new[0];
        t[1]    = t_new[1];


        // 6. Transform source points using accumulated transformation
        for (i = 0; i < source_size; i++) {
            x   = source[i].x;
            y   = source[i].y;
            icp_src_trans[i].x = R[0][0]*x + R[0][1]*y + t[0];
            icp_src_trans[i].y = R[1][0]*x + R[1][1]*y + t[1];
        }


        // 7. Check error (only on valid correspondences)
        mean_error  = 0.0f;
        valid_count = 0;
        for (i = 0; i < source_size; i++) {

            // Skip pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) {
                continue;
            }

            float dx, dy;

            dx = icp_src_trans[i].x - target[icp_correspondences[i]].x;
            dy = icp_src_trans[i].y - target[icp_correspondences[i]].y;
            mean_error += sqrtf(dx*dx + dy*dy);
            valid_count++;
        }

        if (valid_count > 0) {
            mean_error /= valid_count;
        }

        // if tolerance threshold is met, consider it converged
        if (fabsf(prev_error - mean_error) < tolerance) {
            break;
        }

        prev_error = mean_error;
    }

    // Output
    out_R[0] = R[0][0]; out_R[1] = R[0][1];
    out_R[2] = R[1][0]; out_R[3] = R[1][1];

    out_t[0] = t[0];
    out_t[1] = t[1];

#ifdef DEBUG_OUTPUT
    PRINTF("ICP result: t=(%.3f, %.3f)  theta=%.4f rad (%.2f deg)\n",
           t[0],
           t[1],
           atan2f(R[1][0], R[0][0]),
           atan2f(R[1][0], R[0][0]) * 57.2958f);
#endif
}


void ICP_2D_i(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t  max_iteration,
        float   tolerance,

        // 2x2 rotation matrix (row-major)
        float*  out_R,

        // 2x1 translation vector
        float*  out_t)
{

    // counter variables
    uint16_t iter, i;

    // limit variables
    uint16_t valid_range;
    uint16_t valid_count;   // number of correspondences within max distance

    // iterative error for convergence check
    float prev_error;

    // Initialize transformation matrices
    float R[2][2]   = { {1, 0},
                        {0, 1} };
    float t[2]      =   {0, 0};


    // Validate input size against static buffer limits
    if (    source_size > ICP_MAX_POINTS
         || source_size <= 0)
    {

#ifdef DEBUG_OUTPUT
        PRINTF("\tICP: source_size %d exceeds max %d\n",
               source_size,
               ICP_MAX_POINTS);
#endif

        // Return identity transform
        out_R[0] = 1.0f; out_R[1] = 0.0f;
        out_R[2] = 0.0f; out_R[3] = 1.0f;

        out_t[0] = 0.0f; out_t[1] = 0.0f;

        return;
    }

    // Copy source to static buffer while filtering out points beyond max range
    // we will sort this array in-place such that near points are filled in downwards while far points are filled in from the end, upward.
    icp_cache_valid_range = 0;
    Point2D* near   = icp_src_trans;
    Point2D* far    = icp_src_trans + ICP_MAX_POINTS - 1;
    for (i = 0; i < source_size; i++) {

        float range_sq  =   source[i].x*source[i].x \
                          + source[i].y*source[i].y;

        if (range_sq < (MAX_ICP_RANGE * MAX_ICP_RANGE)) {

            // near[icp_cache_valid_range] = source[i];
            *(near++)   = source[i];
            icp_cache_valid_range++;

        } else {

            *(far--)    = source[i];

        }

    }

#ifdef DEBUG_OUTPUT
    PRINTF("ICP_2D_i: filtered source from %u to %u points based on max range\n",
          source_size,
          icp_cache_valid_range);
#endif

    /**
     * The goal is for the source points to converge to the target points
     *  iteratively.
     */
    prev_error = FLT_MAX;
    for (iter = 0; iter < max_iteration; iter++)
    {

        Point2D centroid_src, centroid_tgt;
        float S_xx, S_xy, S_yx, S_yy;
        float theta, c, s;
        float R_iter[2][2];
        float t_iter[2];
        float mean_error;
        float R_new[2][2];
        float t_new[2];
        

        // print first few source and target points on first iteration
#ifdef DEBUG_OUTPUT
        if (PRINT_CHECK) {
            PRINTF("ICP iter %u: "
                   "src[0]=(%.2f,%.2f) src[1]=(%.2f,%.2f) "
                   "tgt[0]=(%.2f,%.2f) tgt[1]=(%.2f,%.2f)\n",

                   iter,
                   icp_src_trans[0].x, icp_src_trans[0].y,
                   icp_src_trans[1].x, icp_src_trans[1].y,
                   target[0].x, target[0].y,
                   target[1].x, target[1].y);
        }
#endif

        // 1. Find correspondences between each source pt. and
        //      all target points in the `target` array.
        //      Also record the squared distance for filtering.
        for (i = 0; i < icp_cache_valid_range; i++) {
            
            icp_correspondences[i] = Find_Closest_Point(
                    icp_src_trans[i],
                    target,
                    target_size,
                    &icp_corr_dist_sq[i]);
                    
        }


        // 2. Compute centroids using only valid correspondences
        //    (distance < ICP_MAX_CORR_DIST)
        centroid_src.x  = 0;    centroid_src.y  = 0;
        centroid_tgt.x  = 0;    centroid_tgt.y  = 0;
        valid_count     = 0;

        // float w_sum = 0.0f;
        for (i = 0; i < icp_cache_valid_range; i++) {

            // alternatively, add this as a Huber loss type penalty instead of hard cutoff
            // float w = (icp_corr_dist_sq[i] < ICP_MAX_CORR_DIST_SQ)
            //             ? 1.0f
            //             : ICP_MAX_CORR_DIST_SQ / icp_corr_dist_sq[i];
                        
            // w_sum += w;
            // centroid_src.x += w*icp_src_trans[i].x;
            // centroid_src.y += w*icp_src_trans[i].y;
            // centroid_tgt.x += w*target[icp_correspondences[i]].x;
            // centroid_tgt.y += w*target[icp_correspondences[i]].y;

            // if (w > 0.0f)
            //     valid_count++;

            if (icp_corr_dist_sq[i] < ICP_MAX_CORR_DIST_SQ) {
                centroid_src.x += icp_src_trans[i].x;
                centroid_src.y += icp_src_trans[i].y;
                centroid_tgt.x += target[icp_correspondences[i]].x;
                centroid_tgt.y += target[icp_correspondences[i]].y;
                valid_count++;
            }

        }

        // Need at least 3 valid correspondences to compute transformation
        if (valid_count < 3) {
#ifdef DEBUG_OUTPUT
            PRINTF("ICP iter %d: only %d valid correspondences, stopping\n",
                    iter,
                    valid_count);
#endif
            break;
        }

        centroid_src.x /= valid_count;
        centroid_src.y /= valid_count;
        centroid_tgt.x /= valid_count;
        centroid_tgt.y /= valid_count;


        // 3. Compute cross-covariance matrix using only valid correspondences
        S_xx = 0;   S_xy = 0;
        S_yx = 0;   S_yy = 0;
        
        for (i = 0; i < icp_cache_valid_range; i++) {

            // // alternatively, add this as a Huber loss type penalty instead of hard cutoff
            // float w = (icp_corr_dist_sq[i] < ICP_MAX_CORR_DIST_SQ)
            //             ? 1.0f
            //             : ICP_MAX_CORR_DIST_SQ / icp_corr_dist_sq[i];
            
            // float x_s   = icp_src_trans[i].x - centroid_src.x;
            // float y_s   = icp_src_trans[i].y - centroid_src.y;
            // float x_t   = target[icp_correspondences[i]].x - centroid_tgt.x;
            // float y_t   = target[icp_correspondences[i]].y - centroid_tgt.y;
            
            // /**
            //  * {{S_xx, S_xy},
            //  *  {S_yx, S_yy}}
            //  */
            // S_xx   += w * x_s * x_t;
            // S_xy   += w * x_s * y_t;
            // S_yx   += w * y_s * x_t;
            // S_yy   += w * y_s * y_t;
            
            
            // early-continue away from pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) {
                continue;
            }
            
            
            float x_s   = icp_src_trans[i].x - centroid_src.x;
            float y_s   = icp_src_trans[i].y - centroid_src.y;
            float x_t   = target[icp_correspondences[i]].x - centroid_tgt.x;
            float y_t   = target[icp_correspondences[i]].y - centroid_tgt.y;

            S_xx   += x_s * x_t;
            S_xy   += x_s * y_t;
            S_yx   += y_s * x_t;
            S_yy   += y_s * y_t;
        }


        // 4. Compute rotation (using SVD for 2x2)
        theta       = atan2f(S_xy - S_yx, S_xx + S_yy);
        
        #ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u: S_xx=%.2f S_xy=%.2f S_yx=%.2f S_yy=%.2f -> theta=%.3f rad (%.1f deg)\n",
        //         iter,
        //            S_xx, S_xy, S_yx, S_yy, theta, theta * 57.2958f);
        // }
        #endif
        
        c   = cosf(theta);
        s   = sinf(theta);

        R_iter[0][0]    =  c;
        R_iter[0][1]    = -s;
        R_iter[1][0]    =  s;
        R_iter[1][1]    =  c;


        // 5. Compute translation
        t_iter[0]   =   centroid_tgt.x
                      - (   R_iter[0][0]*centroid_src.x
                          + R_iter[0][1]*centroid_src.y);

        t_iter[1]   =   centroid_tgt.y
                      - (   R_iter[1][0]*centroid_src.x
                          + R_iter[1][1]*centroid_src.y);


        // Update transformation (compose) before transforming points
        // R_new = R_iter * R_old, t_new = R_iter * t_old + t_iter
        R_new[0][0] = R_iter[0][0]*R[0][0] + R_iter[0][1]*R[1][0];
        R_new[0][1] = R_iter[0][0]*R[0][1] + R_iter[0][1]*R[1][1];
        R_new[1][0] = R_iter[1][0]*R[0][0] + R_iter[1][1]*R[1][0];
        R_new[1][1] = R_iter[1][0]*R[0][1] + R_iter[1][1]*R[1][1];

        t_new[0]    = R_iter[0][0]*t[0] + R_iter[0][1]*t[1] + t_iter[0];
        t_new[1]    = R_iter[1][0]*t[0] + R_iter[1][1]*t[1] + t_iter[1];

        R[0][0] = R_new[0][0]; R[0][1] = R_new[0][1];
        R[1][0] = R_new[1][0]; R[1][1] = R_new[1][1];
        t[0]    = t_new[0];
        t[1]    = t_new[1];


        // 6. Transform source points using accumulated transformation
        for (i = 0; i < icp_cache_valid_range; i++) {

            float x = icp_src_trans[i].x;
            float y = icp_src_trans[i].y;

            icp_src_trans[i].x = R_iter[0][0]*x + R_iter[0][1]*y + t_iter[0];
            icp_src_trans[i].y = R_iter[1][0]*x + R_iter[1][1]*y + t_iter[1];
        }


        // 7. Check error (only on valid correspondences)
        mean_error  = 0.0f;
        valid_count = 0;
        for (i = 0; i < icp_cache_valid_range; i++) {

            // Skip pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) {
                continue;
            }

            float dx = icp_src_trans[i].x - target[icp_correspondences[i]].x;
            float dy = icp_src_trans[i].y - target[icp_correspondences[i]].y;


            // update two places to speed up work
            icp_corr_dist_sq[i] = dx*dx + dy*dy;
            mean_error         += icp_corr_dist_sq[i];

            valid_count++;
        }

        // 
        if (valid_count > 0) {
            mean_error  = sqrtf(mean_error / valid_count);
        }

        // if tolerance threshold is met, consider it converged
        if (fabsf(prev_error - mean_error) < tolerance) {
            break;
        }

        prev_error = mean_error;

        // 8. repeat with new source positions and same target until we see convergence or max iterations

    }

    // 9. post-processing for the far-range post-filtered points




    // 10. return final transformation R,t
    out_R[0] = R[0][0]; out_R[1] = R[0][1];
    out_R[2] = R[1][0]; out_R[3] = R[1][1];

    out_t[0] = t[0];
    out_t[1] = t[1];

#ifdef DEBUG_OUTPUT
    PRINTF("ICP result: t=(%.3f, %.3f)  theta=%.4f rad (%.2f deg)\n",
           t[0],
           t[1],
           atan2f(R[1][0], R[0][0]),
           atan2f(R[1][0], R[0][0]) * 57.2958f);
#endif
}

