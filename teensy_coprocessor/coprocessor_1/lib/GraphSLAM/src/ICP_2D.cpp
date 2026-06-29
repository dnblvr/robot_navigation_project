/**
 * @file ICP_2D.c
 * @author Gian Fajardo (gianfajardo.prim@gmail.com)
 * @brief Implementation of 2D Iterative Closest Point (ICP) algorithm
 * @version 0.1
 */

#include <ICP_2D.h>


// ----------------------------------------------------------------------------
// 
//  DATA STRUCTURES
// 
// ----------------------------------------------------------------------------

/**
 * @brief Statically-allocated buffer for storing transformed source points
 *  for processing
 */
static Point2D  icp_src_trans[ICP_MAX_POINTS];

/**
 * @brief buffer which stores each index of its closest point in target for
 *  each source point; index of source is linked to an index of the target cloud
 */
static index_t  icp_correspondences[ICP_MAX_POINTS];

/**
 * @brief buffer which stores the squared distance for each correspondence
 */
static float    icp_corr_dist_sq[ICP_MAX_POINTS];

/**
 * @brief Cache variables for ICP to speed up repeated calls with the same
 *  input sizes
 */
static uint16_t icp_valid_range = 0;


// ────────────────────────────────────────────────────────────────────────────
// 
//  ICP HELPER FUNCTIONS
// 
// ────────────────────────────────────────────────────────────────────────────

float* ICP_get_cache() {
    return &icp_corr_dist_sq[0];
}


void Find_Closest_Point(
        const Point2D   src,
        const Point2D*  target,
        const uint16_t  target_size,

              index_t*  out_index,
              float*    out_dist_sq)
{
    // counter
    uint16_t    i;

    // properties of closest point
    float    min_dist   = FLT_MAX;
    uint16_t min_idx    = 0;

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
    *out_index   = min_idx;
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


// ────────────────────────────────────────────────────────────────────────────
// 
//  PL-ICP HELPER FUNCTIONS
// 
// ────────────────────────────────────────────────────────────────────────────

void Find_Closest_Points(
        const Point2D   src,
        const Point2D*  target,
        const uint16_t  target_size,

              int*      index,
              Point2D*  q1,
              float*    n_hat)
{
    
    float v_x, v_y, norm;
    
    // iterates through target points to find the closest point via brute-force search
    {
        uint16_t i;                 // counter
        *q1 = (Point2D){0.f, 0.f};  // second closest point, used for normal estimation
        Point2D q2 = {0.f, 0.f};    // second closest point, used for normal estimation

        float closest_dist_sq[2] = {FLT_MAX, FLT_MAX};
        int indices[2] = {-1, -1};

        for (i = 0; i < target_size; i += 1)
        {
            float dx    = src.x - target[i].x;
            float dy    = src.y - target[i].y;
            float d_sq  = dx*dx + dy*dy;
    
            // if point is closer than the closest point, it is implicit that it is also closer than the second closest point. so checking for this filter first allows us to avoid an extra comparison
            if (d_sq < closest_dist_sq[0]) {

                closest_dist_sq[1]  = closest_dist_sq[0];
                indices[1]          = indices[0];
    
                closest_dist_sq[0]  = d_sq;
                indices[0]          = i;
    
            } else if (d_sq < closest_dist_sq[1]) {
    
                closest_dist_sq[1]  = d_sq;
                indices[1]          = i;
            }
        }

#ifdef DEBUG_OUTPUT
        // PRINTF("indices: %d, %d\n", indices[0], indices[1]);
#endif

        // calculate normal vector from the two closest points, which is expected to form a 
        *q1 = target[indices[0]];
        q2  = target[indices[1]];
        
        // return closest point index via output parameter
        *index = indices[0];

        // return `v_x` and `v_y` for normal estimation later
        v_x = q2.x - q1->x;
        v_y = q2.y - q1->y;
    }


    norm = sqrtf(v_x*v_x + v_y*v_y);

    if (norm > 1e-6) {

        // compute normal vector 
        n_hat[0] = -v_y / norm;
        n_hat[1] =  v_x / norm;


    // default normal vector if the two points q1 and q2 are too similar and thus have an unreliable normal direction. hopefully this becomes an outlier enough to not appear in a significant way
    } else {

        n_hat[0] = 1.f;
        n_hat[1] = 0.f;

    }

}


void range_sort(
              Point2D*  source, 
        const uint16_t  source_size, 
              Point2D*  icp_src_trans,
              uint16_t* valid_range_limit)
{
    uint32_t i;
    Point2D* near   = icp_src_trans;
    Point2D* far    = icp_src_trans + (source_size - 1);
    
    *valid_range_limit = 0;
    for (i = 0; i < source_size; i++) {

        float range_sq  =     source[i].x*source[i].x \
                            + source[i].y*source[i].y;

        if (range_sq < (MAX_ICP_RANGE*MAX_ICP_RANGE)) {

            *(near++)   = source[i];
            (*valid_range_limit)++;

        } else {

            *(far--)    = source[i];
        }
    }
}


void accumulate_PL_ICP(
        const Point2D*  src,
        const Point2D   src_centroid,
        const uint16_t  src_size,
        const Point2D*  target,
        const uint16_t  target_size,

              index_t   icp_corr[ICP_MAX_POINTS],
              float     AT_A[TOTAL],
              float     AT_b[DIMS])
{
    int i;
    float n_hat[2] = {0};
    
    memset(AT_A, 0.f, sizeof(float)*TOTAL);
    memset(AT_b, 0.f, sizeof(float)*DIMS);
    // correlation distance_squared array declared elsewhere
    
    
    // it takes this form of this overdetermined system which tracks every correspondence `i` to the following equality:
    // `residual_i = [n_xi, n_yi, c_i] * [dx, dy, dtheta]^T - d_i`, set to 0 for all `i`, the point is to reduce the residual error across all correspondences by solving for least square solution which will eventually take the form `A*x = b`
    for (i = 0; i < src_size; i++) {
        
        int idx;
        Point2D q1;         // closest point in target to src[i]
        float a[DIMS];      // coefficients for one correspondence
        float aT_a[TOTAL];  // system coefficients for one correspondence

    #ifdef DEBUG_OUTPUT
        // PRINTF("src[%d]: (%.1f, %.1f)\n",   i, src[i].x, src[i].y);
    #endif

        Find_Closest_Points(src[i], target, target_size,

                            &idx,   &q1,    n_hat);

        // compute distance of its closest point via subtracting the target q from the source aka p; in other words, the equation is `x = p.x - q.x` and `y = p.y - q.y`
        float x = src[i].x - q1.x;
        float y = src[i].y - q1.y;
        icp_corr_dist_sq[i] = x*x  +  y*y;


        // store the icp correspondences for later post-processing
        icp_corr[i] = idx;


        // check if it's a valid correspondence fit for post-processing
        if (icp_corr_dist_sq[i] > icp_match_distance_sq(i))
            continue;


        // calculate cross-section term `ci` and diagonal terms `di` using the dot product dot_2(q1 - p, n_hat) to determine translational movement; negate x and y distance from previous calcs fit for reuse
        x = -x; y = -y;

        float ci    =   (src[i].x - src_centroid.x)*n_hat[1]
                      - (src[i].y - src_centroid.y)*n_hat[0];
        // float ci    =   (src[i].x)*n_hat[1]
        //               - (src[i].y)*n_hat[0];
        float di    = x*n_hat[0] + y*n_hat[1];

#ifdef DEBUG_OUTPUT
        // Phase 3: per-correspondence sanity sample (first 3 valid corrs only)
        // if (i < 3) {
        //     PRINTF("  corr[%d]: src=(%.1f,%.1f) q1=(%.1f,%.1f) n=(%.3f,%.3f) ci=%.2f di=%.3f\n",
        //            i, src[i].x, src[i].y, q1.x, q1.y, n_hat[0], n_hat[1], ci, di);
        // }
#endif

        // assign one piece of the overdetermined vector `a` and 
        a[0]  = n_hat[0];   a[1]  = n_hat[1];   a[2]  = ci;


        // —— Accumulate `aT_a` into `AT_A` ... ———————————————————————————————
        matmul_3_1x1_3(a, a, aT_a);
        matadd_3x3(AT_A, aT_a, AT_A);   // method 1

        // for (k = 0; k < TOTAL; k++)     // method 2
        //     AT_A[k] += aT_a[k];


        // —— ... and `aT_b` into `AT_b` for each correspondence. —————————————

        // AT_b only requires scalar multiplication since `a` is already 3x1
        AT_b[0]    += a[0]*di;
        AT_b[1]    += a[1]*di;
        AT_b[2]    += a[2]*di;
      
    }
    
}

void denormalize_delta(
        const float     delta_centered[DIMS],
        const Point2D   centroid,
    
              float     delta_true[DIMS])
{

    /**
     * p'_x = px + (dx_c - dθ·py) + dθ·μy
     * p'_y = py + (dy_c + dθ·px) − dθ·μx
     */
    memcpy(delta_true, delta_centered, sizeof(float)*DIMS);

    delta_true[0] += delta_centered[2]*centroid.y;
    delta_true[1] -= delta_centered[2]*centroid.x;
    // angle component is unaffected because this is a translation of the center of rotation
}


// ────────────────────────────────────────────────────────────────────────────
// 
//  ICP FUNCTION
// 
// ────────────────────────────────────────────────────────────────────────────

__attribute__ (( section(".fastrun") ))
void ICP_2D(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,


        uint8_t* num_iter,
        float    out_R_t[TOTAL])
{

    // Validate input size against static buffer limits
    if (    source_size > ICP_MAX_POINTS
         || source_size <= 0)
    {

#ifdef DEBUG_OUTPUT
        // PRINTF("\tICP: source_size %d exceeds max %d\n",
        //        source_size,
        //        ICP_MAX_POINTS);
#endif

        // Return identity transform
        
        out_R_t[R_00] = 1.0f;  out_R_t[R_01] = 0.0f;  out_R_t[T_x_] = 0.0f;
        out_R_t[R_10] = 0.0f;  out_R_t[R_11] = 1.0f;  out_R_t[T_y_] = 0.0f;
        return;
    }

    
    // counter variables
    uint16_t iter, i;
    int valid_count;        // number of correspondences within max distance
    float prev_error;

    // Initialize transformation matrices
    float R[2][2]   = { {1, 0},
                        {0, 1} };
    float t[2]      =   {0, 0};


    // Copy source to static buffer
    for (i = 0; i < source_size; i++) {
        icp_src_trans[i] = source[i];
    }

    // print first few target points on first iteration
#ifdef DEBUG_OUTPUT
    // if (PRINT_CHECK) {
    //     PRINTF("ICP targets:  tgt[0]=(%+4.2f,%+4.2f) tgt[1]=(%+4.2f,%+4.2f)\n",
    //            target[0].x, target[0].y,
    //            target[1].x, target[1].y);
    // }
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
        float theta, cos_theta, sin_theta;
        float R_iter[2][2];
        float t_iter[2];
        float mean_error;
        float R_new[2][2];
        float t_new[2];
        float x, y;
        

        // print first few source points on first iteration
#ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %2u: "
        //            "src[0]=(%+4.2f,%+4.2f) src[1]=(%+4.2f,%+4.2f)\n",

        //            iter,
        //            icp_src_trans[0].x, icp_src_trans[0].y,
        //            icp_src_trans[1].x, icp_src_trans[1].y);
        // }
#endif

        // 1. Find correspondences between each source pt. and
        //      all target points in the `target` array.
        //      Also record the squared distance for filtering.
        for (i = 0; i < source_size; i++) {
            
            Find_Closest_Point(
                    icp_src_trans[i],
                    target,
                    target_size,
                    &icp_correspondences[i],
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
            S_xx += x_s * x_t;      S_xy += x_s * y_t;
            S_yx += y_s * x_t;      S_yy += y_s * y_t;
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
        
        cos_theta = cosf(theta);    sin_theta = sinf(theta);
        R_iter[0][0] =  cos_theta;  R_iter[0][1] = -sin_theta;
        R_iter[1][0] =  sin_theta;  R_iter[1][1] =  cos_theta;


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
                icp_corr_dist_sq[i] = 0.0f;
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

    // Output final transformation
    out_R_t[R_00] = R[0][0];  out_R_t[R_01] = R[0][1];  out_R_t[T_x_] = t[0];
    out_R_t[R_10] = R[1][0];  out_R_t[R_11] = R[1][1];  out_R_t[T_y_] = t[1];


#ifdef DEBUG_OUTPUT
    PRINTF("ICP result: t=(%.3f, %.3f)  theta=%.4f rad (%.2f deg)\n",
           t[0],
           t[1],
           atan2f(R[1][0], R[0][0]),
           atan2f(R[1][0], R[0][0]) * 57.2958f);
#endif
}


__attribute__ (( section(".fastrun") ))
void ICP_2D_i(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,

        uint8_t* num_iter,
        float    out_R_t[TOTAL])
{


    // early-return to validate input size against static buffer limits
    if (    (source_size > ICP_MAX_POINTS)
         || (source_size <= 0))
    {

#ifdef DEBUG_OUTPUT
        PRINTF("\tICP: source_size %d exceeds max %d\n",
               source_size,
               ICP_MAX_POINTS);
#endif

        // Assign default identity transform        
        out_R_t[R_00] = 1.0f;  out_R_t[R_01] = 0.0f;  out_R_t[T_x_] = 0.0f;
        out_R_t[R_10] = 0.0f;  out_R_t[R_11] = 1.0f;  out_R_t[T_y_] = 0.0f;

        return;
    }

    // counter variables
    uint16_t    iter, i;
    int         j;

    // iterative error for convergence check
    float prev_error;

    // Initialize transformation matrices
    float R_t[TOTAL]   = I_3x3;

    float R[2][2]   = { {1, 0},
                        {0, 1} };
    float t[2]      =   {0, 0};


    // reset cache variables for this ICP run
    memset(&icp_src_trans[0],       0, sizeof(Point2D)*ICP_MAX_POINTS);
    memset(&icp_correspondences[0], 0, sizeof(index_t)*ICP_MAX_POINTS);
    memset(&icp_corr_dist_sq[0],    0,   sizeof(float)*ICP_MAX_POINTS);


    // Copy source to static buffer while sorting this array such that near
    // points are filled in downwards while far points are filled in from the
    // end, upward.
    {
        icp_valid_range = 0;
        Point2D* near   = icp_src_trans;
        Point2D* far    = icp_src_trans + source_size - 1;
        for (i = 0; i < source_size; i++) {

            float range_sq  =   source[i].x*source[i].x \
                              + source[i].y*source[i].y;

            if (range_sq < (MAX_ICP_RANGE*MAX_ICP_RANGE)) {

                *(near++)   = source[i];
                icp_valid_range++;

            } else {

                *(far--)    = source[i];
            }
        }
    }

#ifdef DEBUG_OUTPUT
    PRINTF("ICP_2D_i: `ICP_MAX_RANGE` filtered source from %u to %u points\n",
          source_size,
          icp_valid_range);

    if (PRINT_CHECK) {
        PRINTF("ICP tgts:    tgt[0]=(%.2f,%.2f)  tgt[1]=(%.2f,%.2f)\n",
               target[0].x, target[0].y,
               target[1].x, target[1].y);
    }
#endif
    /**
     * The goal is for the source points to converge to the target points
     *  iteratively.
     */
    prev_error  = FLT_MAX;
    for (iter = 0; iter < max_iteration; iter++)
    {

        Point2D centroid_src, centroid_tgt;
        float theta, c, s;
        float R_iter[2][2];
        float t_iter[2];
        float mean_error;
        float R_new[2][2];
        float t_new[2];

        // number of correspondences within max distance
        uint16_t valid_count;
        

        // print first few source and target points on first iteration
#ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u:  src[0]=(%.2f,%.2f)  src[1]=(%.2f,%.2f)\n",

        //            iter,
        //            icp_src_trans[0].x, icp_src_trans[0].y,
        //            icp_src_trans[1].x, icp_src_trans[1].y);
        // }
#endif

        // 1. Find correspondences between each source pt. and
        //      all target points in the `target` array.
        //      Also record the squared distance for filtering.
        for (i = 0; i < icp_valid_range; i++) {
            
            Find_Closest_Point(
                    icp_src_trans[i],
                    target,
                    target_size,

                    &icp_correspondences[i],
                    &icp_corr_dist_sq[i]);
                    
        }


        // 2. Compute centroids using only valid correspondences
        //    (distance < ICP_MAX_CORR_DIST)
        centroid_src.x  = 0;    centroid_src.y  = 0;
        centroid_tgt.x  = 0;    centroid_tgt.y  = 0;
        valid_count     = 0;

        for (i = 0; i < icp_valid_range; i++) {

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
        // this derivces the optimal rotation matrix that minimizes the mean squared error between the source and target points
        float   S_xx = 0,   S_xy = 0,
                S_yx = 0,   S_yy = 0;
    
        for (i = 0; i < icp_valid_range; i++) {            
            
            // early-continue away from pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) {
                continue;
            }
            
            
            float x_s   = icp_src_trans[i].x - centroid_src.x;
            float y_s   = icp_src_trans[i].y - centroid_src.y;
            float x_t   =   target[icp_correspondences[i]].x
                          - centroid_tgt.x;
            float y_t   =   target[icp_correspondences[i]].y
                          - centroid_tgt.y;

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
        
        c   = cosf(theta);  s   = sinf(theta);

        R_iter[0][0]    =  c;   R_iter[0][1]    = -s;
        R_iter[1][0]    =  s;   R_iter[1][1]    =  c;


        // 5. Compute translation
        t_iter[0]   =   centroid_tgt.x
                      - (   R_iter[0][0]*centroid_src.x
                          + R_iter[0][1]*centroid_src.y);

        t_iter[1]   =   centroid_tgt.y
                      - (   R_iter[1][0]*centroid_src.x
                          + R_iter[1][1]*centroid_src.y);


        float R_t_iter[TOTAL]   = I_3x3;
        float R_t_new[TOTAL]    = I_3x3;
        R_t_iter[R_00] = c;  R_t_iter[R_01] = -s;  
        R_t_iter[R_10] = s;  R_t_iter[R_11] =  c;  

        R_t_iter[T_x_] =    centroid_tgt.x
                        - (   R_iter[0][0]*centroid_src.x
                            + R_iter[0][1]*centroid_src.y);

        R_t_iter[T_y_] =    centroid_tgt.y
                        - (   R_iter[1][0]*centroid_src.x
                            + R_iter[1][1]*centroid_src.y);

        


        // Update transformation (compose) before transforming points
        // R_new = R_iter * R_old, t_new = R_iter * t_old + t_iter
        R_new[0][0] = R_iter[0][0]*R[0][0]  +  R_iter[0][1]*R[1][0];
        R_new[0][1] = R_iter[0][0]*R[0][1]  +  R_iter[0][1]*R[1][1];
        R_new[1][0] = R_iter[1][0]*R[0][0]  +  R_iter[1][1]*R[1][0];
        R_new[1][1] = R_iter[1][0]*R[0][1]  +  R_iter[1][1]*R[1][1];

        t_new[0]    = R_iter[0][0]*t[0]  +  R_iter[0][1]*t[1]  +  t_iter[0];
        t_new[1]    = R_iter[1][0]*t[0]  +  R_iter[1][1]*t[1]  +  t_iter[1];


        // replace with function that does the same operation as above
        matmul_3x3(R_t_iter, R_t, R_t_new);


        // update R, t with new values for next iteration        
        // memcpy(R, R_new, sizeof(float)*4);
        // memcpy(t, t_new, sizeof(float)*2);


        // replace with function that does the same operation as above
        R_t[R_00] = R_t_new[R_00];  R_t[R_01] = R_t_new[R_01];
        R_t[R_10] = R_t_new[R_10];  R_t[R_11] = R_t_new[R_11];
        R_t[T_x_] = R_t_new[0];
        R_t[T_y_] = R_t_new[1];
        
        // memcpy(R_t, R_t_new, sizeof(float)*2);


        // 6. Transform source points using accumulated transformation R, t
        for (i = 0; i < icp_valid_range; i++) {

            // float x = icp_src_trans[i].x;
            // float y = icp_src_trans[i].y;

            // icp_src_trans[i].x = R_iter[0][0]*x + R_iter[0][1]*y + t_iter[0];
            // icp_src_trans[i].y = R_iter[1][0]*x + R_iter[1][1]*y + t_iter[1];

            // replace with function that does the same operation as above
            icp_src_trans[i] = transform_point(&icp_src_trans[i], R_t_iter);
        }


        // 7. Check error (only on valid correspondences)
        mean_error  = 0.0f;
        valid_count = 0;
        for (i = 0; i < icp_valid_range; i++) {

            // Skip pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) {
                icp_corr_dist_sq[i] = 0.0f;
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

        // 8. repeat 1. with new source positions and same target until we see convergence or max iterations
    }


    // 9. perform post-processing of far-range points with final R, t
    int num_far   = (int)(source_size - icp_valid_range);
    int far_start = (int)ICP_MAX_POINTS - num_far;
    for (j = (int)ICP_MAX_POINTS - 1; j >= far_start; j--) {

        // replace with function that does the same operation as above
        icp_src_trans[j] = transform_point(&icp_src_trans[j], R_t);

        // then, find their correspondences for potential use in downstream
        // processing (e.g. loop closure)
        Find_Closest_Point(
                icp_src_trans[j],
                target,
                target_size,

                &icp_correspondences[j],
                &icp_corr_dist_sq[j]);
    }


    // 10. return final transformation R,t
    out_R_t[R_00] = R_t[R_00];  out_R_t[R_01] = R_t[R_01];
    out_R_t[R_10] = R_t[R_10];  out_R_t[R_11] = R_t[R_11];

    out_R_t[T_x_] = R_t[T_x_];
    out_R_t[T_y_] = R_t[T_y_];

    *num_iter = iter;
    

#ifdef DEBUG_OUTPUT
    PRINTF("ICP result: t=(%.3f, %.3f)  theta=%.4f rad (%.2f deg)\n",
           t[0],
           t[1],
           atan2f(R[1][0], R[0][0]),
           atan2f(R[1][0], R[0][0]) * 57.2958f);
#endif

}










































































#define PL_ICP 1

#if defined(PL_ICP)

__attribute__ (( section(".fastrun") ))
void ICP_2D_play(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,

        uint8_t* num_iter,
        float    out_R_t[TOTAL])
{   

    // Initialize transformation matrices
    float R_t[TOTAL] = I_3x3;

    // early-return to validate input size against static buffer limits
    if (source_size > ICP_MAX_POINTS) {

#ifdef DEBUG_OUTPUT
        PRINTF("\tICP: source_size %d exceeds %d\n",
               source_size, ICP_MAX_POINTS);
#endif

        memcpy(out_R_t, R_t, sizeof(float)*TOTAL);

        return;
    }

    // counter variables
    uint16_t    iter, i;
    int         j;

    // iterative error for convergence check
    float prev_error;


    // reset cache variables for this ICP run
    memset(icp_src_trans,       0, sizeof(Point2D)*ICP_MAX_POINTS);
    memset(icp_correspondences, 0, sizeof(index_t)*ICP_MAX_POINTS);
    memset(icp_corr_dist_sq,    0,   sizeof(float)*ICP_MAX_POINTS);


    // Copy source to static buffer while sorting this array such that near
    // points are filled in downwards while far points are filled in from the
    // end, upward.
    range_sort(source, 
               source_size, 
               icp_src_trans,
               &icp_valid_range);

#ifdef DEBUG_OUTPUT
    PRINTF("ICP_2D_i: `ICP_MAX_RANGE` filtered source from %u to %u points\n",
          source_size,
          icp_valid_range);

    if (PRINT_CHECK) {
        PRINTF("ICP tgts:    tgt[0]=(%.2f,%.2f)  tgt[1]=(%.2f,%.2f)\n",
               target[0].x, target[0].y,
               target[1].x, target[1].y);
    }
#endif

    /**
     * The goal is for the source points to converge to the target points
     *  iteratively.
     */
    prev_error  = FLT_MAX;
    for (iter = 0; iter < max_iteration; iter++)
    {

        Point2D src_centroid    = {0.f, 0.f};
        float R_t_iter[TOTAL]   = I_3x3;
        float R_t_new[TOTAL]    = I_3x3;
        float delta[DIMS];
        float theta, c, s;
        float mean_error;

        // number of correspondences within max distance
        uint16_t valid_count;
        

        // print first few source and target points on first iteration
#ifdef DEBUG_OUTPUT
        if (PRINT_CHECK) {
            PRINTF("ICP iter %u:  src[0]=(%.2f,%.2f)  src[1]=(%.2f,%.2f)\n",
                   iter,
                   icp_src_trans[0].x, icp_src_trans[0].y,
                   icp_src_trans[1].x, icp_src_trans[1].y);
        }
#endif

#ifdef PROCESSING4_OUTPUT
        PointCloud dummy_cloud;
        for (i = 0; i < source_size; i++) {
            dummy_cloud.points[i] = icp_src_trans[i];
        }
        dummy_cloud.num_pts = source_size;

        numpy_format_print((se2_t){0,0,0}, &dummy_cloud);
#endif

        {
            float delta_centered[DIMS];
            
            // 3. the PLICP paper suggests that I accumulate the overdetermined system `AT_A` and `AT_b` using known correspondences. then solve the matrix `A x == b`.
            float AT_A[TOTAL]   = {0};
            float AT_b[DIMS]    = {0}; 


            Compute_Centroid(icp_src_trans, icp_valid_range, &src_centroid);

            // here, we accumulate the A and b matrices of points up to limit `icp_valid_range`. This function also calculates the `icp_correspondences` that is needed for further processing in step 7, so we can reuse this buffer for both purposes. hopefully the array from 0 to `icp_valid_range` is fully populated
            accumulate_PL_ICP(icp_src_trans, src_centroid, icp_valid_range,
                              target, target_size,

                              icp_correspondences,
                              AT_A, AT_b);

#ifdef DEBUG_OUTPUT
            // Phase 2: normal equations conditioning
            // PRINTF("  AT_A diag=(%.2f,%.2f,%.2f) AT_b=(%.3f,%.3f,%.3f) det=%.6f\n",
            //        AT_A[M_00], AT_A[M_11], AT_A[M_22],
            //        AT_b[0], AT_b[1], AT_b[2],
            //        determinant_3x3(AT_A));
#endif

            // 4. we use the accumulated `A` and `b` matrices which comes up with the optimal solution `[dx, dy, dtheta]` that minimizes the mean squared error of the source and target.
            solve_3x3_system(AT_A, AT_b, delta_centered);
            denormalize_delta(delta_centered, src_centroid, delta);
        }

        // 5. compute the incremental transformation R_t_iter from the solution vector `delta`
        theta       = delta[2];
        c   = cosf(theta);  s   = sinf(theta);
        R_t_iter[R_00] = c;  R_t_iter[R_01] = -s;  R_t_iter[T_x_] = delta[0];
        R_t_iter[R_10] = s;  R_t_iter[R_11] =  c;  R_t_iter[T_y_] = delta[1];


        // Update transformation (compose) before transforming points using matrix multiplication function
        // R_t_new = R_t_iter * R_t
        matmul_3x3(R_t_iter, R_t, R_t_new);

        // update R_t with new values for next iteration
        memcpy(R_t, R_t_new, sizeof(float)*TOTAL);


        // 6. Transform source points using accumulated transformation R_t_iter using function
        for (i = 0; i < icp_valid_range; i++) {
            icp_src_trans[i] = transform_point(icp_src_trans + i, R_t_iter);
        }

    #ifdef DEBUG_OUTPUT
        // Phase 1: full delta summary (translation + rotation)
        PRINTF("iter %u: delta=(%.3f, %.3f, %.2fdeg)\n",
               iter, delta[0], delta[1], delta[2]*57.2958f);
    #endif


        // 7. Check error (only on valid correspondences)
        mean_error  = 0.0f;
        valid_count = 0;
        for (i = 0; i < icp_valid_range; i++) {

            float dx = icp_src_trans[i].x - target[icp_correspondences[i]].x;
            float dy = icp_src_trans[i].y - target[icp_correspondences[i]].y;

            // update two places to speed up work
            icp_corr_dist_sq[i] = dx*dx + dy*dy;

            // most important part of the alg: skip pairs very far apart
            if (icp_corr_dist_sq[i] >= icp_match_distance_sq(iter)) {
                icp_corr_dist_sq[i] = 0.0f;
                continue;
            }

            mean_error         += icp_corr_dist_sq[i];
            valid_count++;
        }

        if (valid_count > 0) {
            mean_error  = sqrtf(mean_error / valid_count);
        }

#ifdef DEBUG_OUTPUT
        // Phase 1 tail: error + valid correspondence count after step 7
        PRINTF("  err=%.3f valid=%u\n", mean_error, valid_count);
#endif

        // if tolerance threshold is met, consider it converged
        if (fabsf(prev_error - mean_error) < tolerance) {
            break;
        }

        prev_error = mean_error;

        // 8. repeat 1. with new source positions and same target until we see convergence or max iterations
    }


    // 9. perform post-processing of far-range points with final R, t
    int num_far   = (int)(source_size  -  icp_valid_range);
    int far_start = (int)(source_size) -  num_far;
    for (j = (int)source_size - 1; j >= far_start; j--) {
        
        // first, perform an action on the far points with the final transformation using function
        icp_src_trans[j] = transform_point(icp_src_trans + j, R_t);

        // then, find their correspondences for potential use in downstream
        // processing (e.g. loop closure)

        Find_Closest_Point(
                icp_src_trans[j],  target,  target_size,

                &icp_correspondences[j], &icp_corr_dist_sq[j]);
    }


#ifdef DEBUG_OUTPUT
    PRINTF("ICP result: t=(%.3f, %.3f)  theta=%.4f rad (%.2f deg)\n",
           R_t[T_x_],
           R_t[T_y_],
           atan2f(R_t[R_10], R_t[R_00]),
           atan2f(R_t[R_10], R_t[R_00]) * 57.2958f);
#endif

    // 10. return final transformation R,t
    // out_R_t[R_00] = R_t[R_00];  out_R_t[R_01] = R_t[R_01];
    // out_R_t[R_10] = R_t[R_10];  out_R_t[R_11] = R_t[R_11];

    // out_R_t[T_x_] = R_t[T_x_];
    // out_R_t[T_y_] = R_t[T_y_];

    memcpy(out_R_t, R_t, sizeof(float)*TOTAL);

    *num_iter   = iter;
}











#else 













// __attribute__ (( section(".fastrun") ))
void ICP_2D_play(
        Point2D* source, uint16_t source_size,
        Point2D* target, uint16_t target_size,
        uint16_t max_iteration,
        float    tolerance,

        uint8_t* num_iter,
        float    out_R_t[TOTAL])
{   
    // early-return to validate input size against static buffer limits
    if (source_size > ICP_MAX_POINTS) {

#ifdef DEBUG_OUTPUT
        PRINTF("\tICP: source_size %d exceeds ICP_MAX_POINTS\n", source_size);
#endif
        // Assign default identity transform
        // out_R_t[R_00] = 1.f;    out_R_t[R_01] = 0.f;    out_R_t[T_x_] = 0.f;
        // out_R_t[R_10] = 0.f;    out_R_t[R_11] = 1.f;    out_R_t[T_y_] = 0.f;
        memcpy(out_R_t, (float[TOTAL])I_3x3, sizeof(float)*TOTAL);

        return;
    }

    // counter variables
    uint16_t    iter, i;
    int         j;

    // iterative error for convergence check
    float prev_error;
    
    // Initialize transformation matrices
    float R_t[TOTAL]   = I_3x3;

    // reset cache variables for this ICP run
    memset(&icp_src_trans[0],       0, sizeof(Point2D)*ICP_MAX_POINTS);
    memset(&icp_correspondences[0], 0, sizeof(index_t)*ICP_MAX_POINTS);
    memset(&icp_corr_dist_sq[0],    0,   sizeof(float)*ICP_MAX_POINTS);


    // Copy source to static buffer while sorting this array such that near
    // points are filled in downwards while far points are filled in from the
    // end, upward.
    range_sort(source, 
               source_size, 
               icp_src_trans,
               &icp_valid_range);

#ifdef DEBUG_OUTPUT
    PRINTF("ICP_2D_play: `ICP_MAX_RANGE` filtered source from %u to %u points\n",
          source_size,
          icp_valid_range);

    if (PRINT_CHECK) {
        PRINTF("ICP tgts:    tgt[0]=(%.2f,%.2f)  tgt[1]=(%.2f,%.2f)\n",
               target[0].x, target[0].y,
               target[1].x, target[1].y);
    }
#endif

    /**
     * The goal is for the source points to converge to the target points
     *  iteratively.
     */
    prev_error  = FLT_MAX;
    for (iter = 0; iter < max_iteration; iter++)
    {
        Point2D centroid_src, centroid_tgt;
        float theta, c, s;
        float mean_error;
        float R_t_iter[TOTAL]   = I_3x3;
        float R_t_new[TOTAL]    = I_3x3;

        // number of correspondences within max distance
        uint16_t valid_count;
        

        // print first few source and target points on first iteration
#ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u:  src[0]=(%.2f,%.2f)  src[1]=(%.2f,%.2f)\n",
        //            iter,
        //            icp_src_trans[0].x, icp_src_trans[0].y,
        //            icp_src_trans[1].x, icp_src_trans[1].y);
        // }
#endif
#ifdef PROCESSING4_OUTPUT
        // print the whole cloud for later visual debugging
        PointCloud dummy_cloud;

        for (i = 0; i < source_size; i++) {
            dummy_cloud.points[i] = icp_src_trans[i];
        }
        dummy_cloud.num_pts = source_size;

        C_format_print((se2_t){0,0,0}, &dummy_cloud);
#endif

        // 1. Find correspondences between each source pt. and
        //      all target points in the `target` array.
        //      Also record the squared distance for filtering.
        for (i = 0; i < icp_valid_range; i++) {
            
            Find_Closest_Point(
                    icp_src_trans[i],  target,  target_size,

                    &icp_correspondences[i],
                    &icp_corr_dist_sq[i]);
        }


        // 2. Compute centroids using only valid correspondences
        //    (distance < ICP_MAX_CORR_DIST)
        centroid_src.x  = 0;    centroid_src.y  = 0;
        centroid_tgt.x  = 0;    centroid_tgt.y  = 0;
        valid_count     = 0;

        
        for (i = 0; i < icp_valid_range; i++) {

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

        centroid_src.x /= valid_count;      centroid_src.y /= valid_count;
        centroid_tgt.x /= valid_count;      centroid_tgt.y /= valid_count;


        // 3. Compute cross-covariance matrix using only valid correspondences
        // this derivces the optimal rotation matrix that minimizes the mean squared error between the source and target points
        float   S_xx = 0,   S_xy = 0,
                S_yx = 0,   S_yy = 0;
    
        for (i = 0; i < icp_valid_range; i++) {
            
            // early-continue away from pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ)
                continue;
            
            float x_s   = icp_src_trans[i].x - centroid_src.x;
            float y_s   = icp_src_trans[i].y - centroid_src.y;
            float x_t   =   target[icp_correspondences[i]].x
                          - centroid_tgt.x;
            float y_t   =   target[icp_correspondences[i]].y
                          - centroid_tgt.y;

            S_xx += x_s * x_t;  S_xy += x_s * y_t; 
            S_yx += y_s * x_t;  S_yy += y_s * y_t;
        }


        // 4. Compute rotation (using SVD for 2x2)
        theta       = atan2f(S_xy - S_yx, S_xx + S_yy);
        c   = cosf(theta);  s   = sinf(theta);
        
    #ifdef DEBUG_OUTPUT
        // if (PRINT_CHECK) {
        //     PRINTF("ICP iter %u: S_xx=%.2f S_xy=%.2f S_yx=%.2f S_yy=%.2f -> theta=%.3f rad (%.1f deg)\n",
        //         iter,
        //            S_xx, S_xy, S_yx, S_yy, theta, theta * 57.2958f);
        // }
    #endif

        R_t_iter[R_00] = c;  R_t_iter[R_01] = -s;  
        R_t_iter[R_10] = s;  R_t_iter[R_11] =  c;  

        R_t_iter[T_x_] =    centroid_tgt.x
                        - (   R_t_iter[R_00]*centroid_src.x
                            + R_t_iter[R_01]*centroid_src.y);

        R_t_iter[T_y_] =    centroid_tgt.y
                        - (   R_t_iter[R_10]*centroid_src.x
                            + R_t_iter[R_11]*centroid_src.y);
        

        // Update transformation (compose) before transforming points using matrix multiplication function
        // R_t_new = R_t_iter * R_t
        matmul_3x3(R_t_iter, R_t, R_t_new);


        // update R, t with new values for next iteration using functions
        memcpy(R_t, R_t_new, sizeof(float)*TOTAL);


        // 6. Transform source points using accumulated transformation R_t using function
        for (i = 0; i < icp_valid_range; i++) {
            icp_src_trans[i] = transform_point(&icp_src_trans[i], R_t_iter);
        }

    #ifdef DEBUG_OUTPUT
        printf("    movement: t=(%.3f, %.3f)\n", R_t_iter[0], R_t_iter[1]);
    #endif

        // 7. Check error (only on valid correspondences)
        mean_error  = 0.0f;
        valid_count = 0;
        for (i = 0; i < icp_valid_range; i++) {

            // Skip pairs that are too far apart
            if (icp_corr_dist_sq[i] >= ICP_MAX_CORR_DIST_SQ) {
                icp_corr_dist_sq[i] = 0.0f;
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

        // 8. repeat 1. with new source positions and same target until we see convergence or max iterations
    }


    // 9. perform post-processing of far-range points with final R, t
    int num_far   = (int)(source_size - icp_valid_range);
    int far_start = (int)source_size - num_far;
    for (j = (int)source_size - 1; j >= far_start; j--) {
        
        // first, perform an action on the far points with the final transformation using function
        icp_src_trans[j] = transform_point(&icp_src_trans[j], R_t);

        // then, find their correspondences for potential use in downstream
        // processing (e.g. loop closure)
        Find_Closest_Point(
                icp_src_trans[j], target, target_size,

                &icp_correspondences[j],
                &icp_corr_dist_sq[j]);
    }
    

#ifdef DEBUG_OUTPUT
    PRINTF("ICP result: t=(%.3f, %.3f)  theta=%.4f rad (%.2f deg)\n",
           R_t[T_x_],
           R_t[T_y_],
           atan2f(R_t[R_10], R_t[R_00]),
           atan2f(R_t[R_10], R_t[R_00]) * 57.2958f);
#endif


    // 10. return final transformation R_t
    memcpy(out_R_t, R_t, sizeof(float)*TOTAL);

    *num_iter = iter;
}

#endif // PLAY_ICP_IMPLEMENTATION