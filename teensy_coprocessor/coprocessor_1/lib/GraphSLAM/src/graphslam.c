/**
 * @file graphslam.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 */

#include <graphslam.h>


// Forward declaration of internal helper
static void slam_apply_constraint(
        SLAMOptimizer*  optimizer,

        int     pose1_id,
        int     pose2_id,
        float   dx,
        float   dy,
        float   dtheta,
        float   confidence);


// ────────────────────────────────────────────────────────────────────────────
//
//  ICP INTEGRATION
//
// ────────────────────────────────────────────────────────────────────────────

void slam_perform_icp(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const se2_t*         initial_guess,
              ICPResult*    result)
{

    // output variables
    float R_t[TOTAL];  // translation vector
    
    
    // If initial guess is non-zero, pre-transform scan1 so ICP only has to
    // find the small residual correction, then compose the two to recover
    // the full transformation: z_ij = initial_guess + residual
    if (    (initial_guess->x      != 0.f)
         || (initial_guess->y      != 0.f)
         || (initial_guess->theta  != 0.f))
    {

        se2_t icp_residual;
        se2_t full_transform;

        // Pre-transform scan1 with the initial guess
        PointCloud transformed_scan1;
        transform_point_cloud(scan1, initial_guess, &transformed_scan1);
        
        // ICP finds the residual correction between scan1' and scan2
        ICP_2D(transformed_scan1.points, transformed_scan1.num_pts,
               (Point2D*)scan2->points, scan2->num_pts,
               MAX_ICP_ITERATIONS,
               ICP_CONVERGENCE_TOLERANCE,
               &result->num_iterations,
               R_t);

        // Compose initial_guess + icp_residual to get the full transformation.
        // compose_poses(p2, p1, result) computes result = p1 + p2
        icp_residual   = (se2_t){R_t[T_x_],
                                R_t[T_y_],
                                atan2f(R_t[R_10], R_t[R_00])};
        
        compose_poses(&icp_residual, initial_guess, &full_transform);

        result->dx      = full_transform.x;
        result->dy      = full_transform.y;
        result->dtheta  = full_transform.theta;
    

    // ICP finds the full transformation directly
    } else {

        ICP_2D((Point2D*)scan1->points, scan1->num_pts,
               (Point2D*)scan2->points, scan2->num_pts,
               MAX_ICP_ITERATIONS,
               ICP_CONVERGENCE_TOLERANCE,
               &result->num_iterations,
               R_t);

        result->dx      = R_t[T_x_];
        result->dy      = R_t[T_y_];
        result->dtheta  = atan2f(R_t[R_10], R_t[R_00]);
    }

    result->valid   = true;

}


float slam_compute_icp_confidence(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const ICPResult*    result)
{

    int i, j;
    float total_error, mean_error, match_ratio;
    float error_confidence, confidence;
    int matches;
    se2_t icp_transform;
    PointCloud transformed;
    
    
    // Then apply the ICP result transformation
    icp_transform.x     = result->dx;
    icp_transform.y     = result->dy;
    icp_transform.theta = result->dtheta;
    
    transform_point_cloud(scan1, &icp_transform, &transformed);

    // Compute mean correspondence distance
    total_error = 0.0f;
    matches     = 0;
    
    for (i = 0; i < transformed.num_pts; i++) {

        float dx, dy, dist;
        float min_dist = FLT_MAX;
        
        for (j = 0; j < scan2->num_pts; j++) {

            dx   = transformed.points[i].x - scan2->points[j].x;
            dy   = transformed.points[i].y - scan2->points[j].y;
            dist = sqrtf(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        
        // if the threshold for valid match is met, add to the matches
        // Use larger threshold (100mm) to account for LiDAR noise at long range
        if (min_dist < VALID_MATCH_DISTANCE) {
            total_error += min_dist;
            matches++;
        }
        
    }
    
#ifdef DEBUG_OUTPUTS
//    printf("Confidence: %d matches out of %d points, total_error=%.3f\n",
//           matches, transformed.num_pts, total_error);
#endif
    
    if (matches == 0)
        return 0.0f;
    
    mean_error  = total_error / matches;
    match_ratio = (float)matches / transformed.num_pts;


    // Confidence based on error and match ratio
    //      - exp(-error/20): 20mm error --> 0.37 confidence
    error_confidence    = expf( -mean_error / ERROR_CONFIDENCE_SCALE );
    confidence          = error_confidence * match_ratio;

    
#ifdef DEBUG_OUTPUTS
    printf("  mean_error=%.3f, error_conf=%.3f,"
           " match_ratio=%.3f, confidence=%.3f\n",
           mean_error, error_confidence, match_ratio, confidence);
#endif
    
    return confidence;

}

void slam_perform_icp_i(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const se2_t*         initial_guess,
              ICPResult*    result)
{
    // output variables
    float R_t[TOTAL];  // 2x2 rotation matrix (row-major) + translation vector

#ifdef DEBUG_OUTPUT     
    PRINTF("MAX_ICP_RANGE: %.2f\n", MAX_ICP_RANGE);
    PRINTF("ICP_MAX_CORR_DIST: %.2f\n", ICP_MAX_CORR_DIST);
#endif
    
    // If initial guess is non-zero, pre-transform scan1 so ICP only has to
    // find the small residual correction, then compose the two to recover
    // the full transformation: z_ij = initial_guess + residual
    if (    initial_guess->x       != 0.f
         || initial_guess->y       != 0.f
         || initial_guess->theta   != 0.f)
    {
        se2_t icp_residual;
        se2_t full_transform;

        // Pre-transform scan1 with the initial guess
        PointCloud transformed_scan1;
        transform_point_cloud(scan1, initial_guess, &transformed_scan1);
        
        // ICP finds the residual correction between scan1' and scan2
        ICP_2D_i(transformed_scan1.points, transformed_scan1.num_pts,
                 (Point2D*)scan2->points, scan2->num_pts,
                 MAX_ICP_ITERATIONS,
                 ICP_CONVERGENCE_TOLERANCE,
                 &result->num_iterations,
                 R_t);

        // Compose initial_guess + icp_residual to get the full transformation.
        // compose_poses(p2, p1, result) computes result = p1 + p2
        icp_residual   = (se2_t){R_t[T_x_],
                                R_t[T_y_],
                                atan2f(R_t[R_10], R_t[R_00])};
        
        compose_poses(&icp_residual, initial_guess, &full_transform);

        result->dx      = full_transform.x;
        result->dy      = full_transform.y;
        result->dtheta  = full_transform.theta;
    

    // ICP finds the full transformation directly
    } else {

        ICP_2D_i((Point2D*)scan1->points, scan1->num_pts,
                 (Point2D*)scan2->points, scan2->num_pts,
                 MAX_ICP_ITERATIONS,
                 ICP_CONVERGENCE_TOLERANCE, 

                 &result->num_iterations, R_t);

        result->dx      = R_t[T_x_];
        result->dy      = R_t[T_y_];
        result->dtheta  = atan2f(R_t[R_10], R_t[R_00]);
    }

    result->valid   = true;

}

float slam_compute_icp_confidence_i()
{
    int     i;
    float   min_dist;
    float   total_error;

    float*  cache   = ICP_get_cache();

    // Compute mean correspondence distance
    total_error = 0.0f;
    
    for (i = 0; i < ICP_MAX_POINTS; i++) {

        min_dist        = sqrtf( cache[i] );
        total_error    += expf( -min_dist / ERROR_CONFIDENCE_SCALE );
    }
    
    return total_error / ICP_MAX_POINTS;
}


// ────────────────────────────────────────────────────────────────────────────
//
//  CORE SLAM FUNCTIONS
//
// ────────────────────────────────────────────────────────────────────────────

uint8_t slam_initialize(SLAMOptimizer* optimizer)
{
    optimizer->current_pose_count       = 0;
    optimizer->buffer_size              = 0;
    optimizer->num_constraints          = 0;
    optimizer->matrices_initialized     = false;
    optimizer->optimization_requested   = false;
    optimizer->last_optimization_time   = 0;
    

    // Initialize dense H matrix and vectors to zero
    memset(optimizer->H,     0, STATE_SIZE * STATE_SIZE * sizeof(float));
    memset(optimizer->b,     0, STATE_SIZE * sizeof(float));
    memset(optimizer->state, 0, STATE_SIZE * sizeof(float));
    
    optimizer->matrices_initialized = true;

    return SLAM_SUCCESS;

}


uint8_t slam_add_pose(
              SLAMOptimizer*    optimizer,
        const se2_t*            pose,
        const PointCloud*       scan)
{
    // counters and temporary variables
    int idx, i;
    int new_constraint_count;


    // append if buffer is not full
    if (optimizer->buffer_size < MAX_POSES) {
        
        idx = optimizer->buffer_size;
        optimizer->buffer_size++;


    // else if buffer is full, shift everything down (discard oldest)
    } else {
        
        // Shift poses and scans: move 1..MAX_POSES-1 to 0..MAX_POSES-2
        for (i = 0; i < (MAX_POSES - 1); i++) {
            optimizer->pose_pool[i] = optimizer->pose_pool[i + 1];
            optimizer->scan_pool[i] = optimizer->scan_pool[i + 1];
        }
        
        // Shift state vector similarly
        for (i = 0; i < 3*(MAX_POSES - 1); i++)
            optimizer->state[i] = optimizer->state[i + 3];
        
        // Shift constraint IDs and remove any that reference pose 0
        // (pose 0 is being discarded, so its constraints are invalid)
        new_constraint_count = 0;
        for (i = 0; i < optimizer->num_constraints; i++) {
            Constraint* c = &optimizer->constraints[i];
            
            // Skip constraints that reference pose 0 (being discarded)
            if (c->pose1_id == 0 || c->pose2_id == 0)
                continue;
            
            // Decrement pose IDs (since everything shifted down by 1)
            c->pose1_id -= 1;
            c->pose2_id -= 1;
            
            // Keep this constraint (move to front if needed)
            if (new_constraint_count != i)
                optimizer->constraints[new_constraint_count] = *c;

            new_constraint_count++;
        }

        optimizer->num_constraints = new_constraint_count;
        
        // Clear H and b - they will be rebuilt with updated constraint IDs
        memset(optimizer->H, 0, STATE_SIZE * STATE_SIZE * sizeof(float));
        memset(optimizer->b, 0, STATE_SIZE * sizeof(float));
        
        // New pose goes at the last slot
        idx = MAX_POSES - 1;
        
    }


    
    // Store pose and scan at computed index
    optimizer->pose_pool[idx]   = *pose;
    optimizer->scan_pool[idx]   = *scan;
    
    // Update state vector for this new pose
    // IMPORTANT: Initialize state to the provided pose estimate
    // This gives optimization a good starting point, but it will adjust
    // these values to minimize constraint errors
    optimizer->state[idx*3 + 0] = pose->x;
    optimizer->state[idx*3 + 1] = pose->y;
    optimizer->state[idx*3 + 2] = pose->theta;
    
    optimizer->current_pose_count++;
    
    return SLAM_SUCCESS;

}

void slam_add_odometry_constraint(
        SLAMOptimizer*  optimizer,
        int             pose1_id,
        int             pose2_id,
        float           dx,
        float           dy,
        float           dtheta,
        float           confidence)
{
    // Store the constraint for later rebuilding during optimization
    if (optimizer->num_constraints < MAX_CONSTRAINTS) {

        Constraint* c   = &optimizer->constraints[optimizer->num_constraints];

        c->pose1_id     = pose1_id;
        c->pose2_id     = pose2_id;
        c->dx           = dx;
        c->dy           = dy;
        c->dtheta       = dtheta;
        c->confidence   = confidence;
        optimizer->num_constraints++;

    }
    
}


/**
 * @brief Apply a single constraint to H and b matrices (internal helper)
 * 
 * Uses current state estimate (not original pose_pool) for error/Jacobian
 *  calculation. This is critical for Gauss-Newton iteration to converge
 *  properly.
 */
static void slam_apply_constraint(
        SLAMOptimizer*  optimizer,
        int             pose1_id,
        int             pose2_id,
        float           dx,
        float           dy,
        float           dtheta,
        float           confidence)
{
    // counters
    int i, j, k, i1, i2;

    float info_xy, info_theta;
    float z_ij[3];
    float error[3];
    float A[3][3], B[3][3];
    float omega[3];
    float sum;
    se2_t p1, p2;  // Local copies built from state vector
    

    if (!optimizer->matrices_initialized)
        return;

    if (    (pose1_id < 0)  ||  (pose1_id >= optimizer->buffer_size)
         || (pose2_id < 0)  ||  (pose2_id >= optimizer->buffer_size) )
        return;

    // Direct indexing - no circular buffer
    i1  = pose1_id * 3;
    i2  = pose2_id * 3;
    

    // Information matrix weights
    info_xy     = confidence;
    info_theta  = confidence * 0.5f;
    

    // Build poses from CURRENT STATE ESTIMATE (not original pose_pool!)
    // This is essential for Gauss-Newton to work correctly across iterations
    p1.x        = optimizer->state[i1 + 0];
    p1.y        = optimizer->state[i1 + 1];
    p1.theta    = optimizer->state[i1 + 2];
    
    p2.x        = optimizer->state[i2 + 0];
    p2.y        = optimizer->state[i2 + 1];
    p2.theta    = optimizer->state[i2 + 2];
    
    z_ij[0]     = dx;
    z_ij[1]     = dy;
    z_ij[2]     = dtheta;
    evaluate_error_pose_pose(&p1, &p2, z_ij, error);
    
#ifdef DEBUG_OUTPUTS
    // print error magnitude for ALL constraints to diagnose zero-error issue
    float error_mag = sqrtf(    error[0]*error[0]
                              + error[1]*error[1]);

    printf("    Constraint %d->%d: p1=(%.1f,%.1f,%.3f) p2=(%.1f,%.1f,%.3f)"
           " z=(%.1f,%.1f,%.3f) err=(%.2f,%.2f,%.3f) mag=%.2f\n",
           pose1_id, pose2_id, 
           p1.x, p1.y, p1.theta,
           p2.x, p2.y, p2.theta,
           dx, dy, dtheta,
           error[0], error[1], error[2], error_mag);
#endif

    // Compute Jacobians using current state estimate
    compute_jacobian_pose_pose(&p1, &p2, A, B);
    

    // Add to information matrix H and vector b
    //
    // H +=   A^T * Omega * A
    //      + B^T * Omega * B
    //      + A^T * Omega * B
    //      + B^T * Omega * A
    //
    // b +=  -A^T * Omega * error
    //      - B^T * Omega * error
    
    omega[0] = info_xy;
    omega[1] = info_xy;
    omega[2] = info_theta;
    
    // A^T * Omega * error
    for (i = 0; i < 3; i++) {
        sum = 0.0f;
        for (j = 0; j < 3; j++) {
            sum += A[j][i] * omega[j] * error[j];
        }
        optimizer->b[i1 + i] -= sum;
    }
    
    // B^T * Omega * error
    for (i = 0; i < 3; i++) {
        sum = 0.0f;
        for (j = 0; j < 3; j++) {
            sum += B[j][i] * omega[j] * error[j];
        }
        optimizer->b[i2 + i] -= sum;
    }
    
    // A^T * Omega * A
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += A[k][i] * omega[k] * A[k][j];
            }
            optimizer->H[(i1 + i)*STATE_SIZE + (i1 + j)] += sum;
        }
    }
    
    // B^T * Omega * B
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += B[k][i] * omega[k] * B[k][j];
            }
            optimizer->H[(i2 + i)*STATE_SIZE + (i2 + j)] += sum;
        }
    }
    
    // A^T * Omega * B (off-diagonal blocks)
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += A[k][i] * omega[k] * B[k][j];
            }
            optimizer->H[(i1 + i)*STATE_SIZE + (i2 + j)] += sum;
        }
    }
    
    // B^T * Omega * A (off-diagonal blocks)
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += B[k][i] * omega[k] * A[k][j];
            }
            optimizer->H[(i2 + i)*STATE_SIZE + (i1 + j)] += sum;
        }
    }

}

void slam_add_icp_constraint(
        SLAMOptimizer*  optimizer,
        int             pose1_id,
        int             pose2_id,
        const ICPResult* icp_result,
        float           base_confidence)
{

    float adjusted_confidence = base_confidence * icp_result->confidence;

    if (!icp_result->valid)
        return;
    
    slam_add_odometry_constraint(
            optimizer,
            pose1_id,
            pose2_id,
            icp_result->dx,
            icp_result->dy,
            icp_result->dtheta,
            adjusted_confidence);
}

uint8_t slam_detect_loop_closure(
        SLAMOptimizer*  optimizer,
        int             current_pose_id)
{
    
    int candidate;
    float dist, confidence;
    se2_t current_pose, candidate_pose;  // Local copies from state vector
    PointCloud *current_scan, *candidate_scan;
    int idx_curr;
    // int idx_cand;
    

    // early return if `current_pose_id` is not within bounds
    if (    (current_pose_id < MIN_TEMPORAL_GAP)
         || (current_pose_id >= optimizer->buffer_size))
    {
        return SLAM_FAILURE;
    }
    

    // Build current pose from STATE (optimized estimate)
    idx_curr            = current_pose_id * 3;
    current_pose.x      = optimizer->state[idx_curr + 0];
    current_pose.y      = optimizer->state[idx_curr + 1];
    current_pose.theta  = optimizer->state[idx_curr + 2];
    
    current_scan    = &optimizer->scan_pool[current_pose_id];
    

    // Check spatially close poses
    for (   candidate = 0;
            candidate < current_pose_id - MIN_TEMPORAL_GAP;
            candidate++)
    {
        // Build candidate pose from STATE (optimized estimate)
        // idx_cand                = 3*candidate;
        candidate_pose.x        = optimizer->state[3*candidate + 0];
        candidate_pose.y        = optimizer->state[3*candidate + 1];
        candidate_pose.theta    = optimizer->state[3*candidate + 2];
        
        dist = pose_distance(&current_pose, &candidate_pose);
        
#ifdef DEBUG_OUTPUTS
        Serial.printf("  Checking candidate %d (buf_idx %d), dist=%.3f (thresh=%.1f)\n",
                candidate, candidate_buffer_idx, dist, LOOP_DISTANCE_THRESHOLD);
#endif

        if (dist < LOOP_DISTANCE_THRESHOLD) {

            // check scan similarity with ICP alignment
            se2_t initial_guess;
            ICPResult icp_result;

            // Attempt ICP alignment
            candidate_scan  = &optimizer->scan_pool[candidate];

            // Use expected relative pose as initial guess for ICP
            // This helps ICP converge to the correct solution
            relative_pose(&candidate_pose, &current_pose, &initial_guess);

            slam_perform_icp(current_scan,
                             candidate_scan,
                             &initial_guess,
                             &icp_result);
            

            confidence = slam_compute_icp_confidence(
                    current_scan,
                    candidate_scan,
                    &icp_result);


            icp_result.confidence = confidence;
            
            if (confidence > ICP_CONFIDENCE_THRESHOLD) {
                
                // ICP was initialized with expected relative pose (initial_guess)

                // Add loop closure constraint with moderate confidence boost
                slam_add_odometry_constraint(
                        optimizer,
                        candidate,
                        current_pose_id,
                        icp_result.dx,
                        icp_result.dy,
                        icp_result.dtheta,
                        confidence * 5.0f); // Moderate boost over odometry

    #ifdef DEBUG_OUTPUTS
                Serial.printf(" --> *** LOOP CLOSURE DETECTED:"
                              " pose %d matches pose %d! ***\n",
                              current_pose_id, candidate);
                Serial.printf("     initial_guess: (%.2f, %.2f, %.3f)\n", 
                              initial_guess.x, initial_guess.y,
                              initial_guess.theta);
                Serial.printf("     ICP correction: (%.2f, %.2f, %.3f)\n",
                              icp_result.dx, icp_result.dy,
                              icp_result.dtheta);
    #endif
                
                return SLAM_SUCCESS;

            } // if (confidence > ICP_CONFIDENCE_THRESHOLD)

        } // if (dist < LOOP_DISTANCE_THRESHOLD)

    } // for loop over candidates
    
    return SLAM_FAILURE;
}

void slam_optimize_gauss_newton(
        SLAMOptimizer*  optimizer,
        int             max_iterations)
{

    if (!optimizer->matrices_initialized)
        return;
    if (optimizer->buffer_size < 2)
        return;
    

    // Need enough constraints to form a connected graph. At minimum, we need
    // (buffer_size - 1) odometry constraints to connect all poses. Without 
    // enough constraints, H matrix will be singular.
    if (optimizer->num_constraints < optimizer->buffer_size - 1) {

#ifdef DEBUG_OUTPUTS
        Serial.printf("Optimization skipped: only %d constraints for %d poses (need %d)\n",
               optimizer->num_constraints, optimizer->buffer_size, 
               optimizer->buffer_size - 1);
#endif

        return;
    }

    int i, iter, p, c;
    int state_size;
    int8_t result;
    float dx_norm;
    Constraint *con;

    // Static buffers for Gauss-Newton optimization
    // Size: STATE_SIZE = 3 * MAX_POSES = 45
    static float L[STATE_SIZE * STATE_SIZE];
    static float y[STATE_SIZE];
    static float dx[STATE_SIZE];
    
    
    // Gauss-Newton iteration loop
    // Each iteration: rebuild H/b from constraints using current state, solve
    // for update dx, apply update to state

    state_size  = optimizer->buffer_size * 3;
    for (iter = 0; iter < max_iterations; iter++) {

#ifdef DEBUG_OUTPUTS
        Serial.printf("i-%d... ", iter);
#endif

        // Clear H and b, then rebuild from stored constraints. Must be done
        // each iteration because Jacobians depend on current state.
        memset(optimizer->H, 0, STATE_SIZE * STATE_SIZE * sizeof(float));
        memset(optimizer->b, 0, STATE_SIZE * sizeof(float));
        
        // Rebuild H and b from all stored constraints
        for (c = 0; c < optimizer->num_constraints; c++) {
            con = &optimizer->constraints[c];
            
            // Skip constraints that reference poses no longer in buffer
            if (    (con->pose1_id < 0                      )   \
                 || (con->pose1_id >= optimizer->buffer_size)   \
                 || (con->pose2_id < 0                      )   \
                 || (con->pose2_id >= optimizer->buffer_size))
            {
                continue;
            }

                
            slam_apply_constraint(optimizer,
                                  con->pose1_id,
                                  con->pose2_id,
                                  con->dx,
                                  con->dy,
                                  con->dtheta,
                                  con->confidence);
        }
        
        // Clear L buffer
        memset(L, 0, STATE_SIZE*STATE_SIZE * sizeof(float));

        // Fix the first pose (anchor) to prevent drift by adding large values
        // to diagonal of first pose's block. This effectively pins pose 0 at
        // its current location         
        optimizer->H[0*STATE_SIZE + 0] += ANCHOR_WEIGHT;    // x_0
        optimizer->H[1*STATE_SIZE + 1] += ANCHOR_WEIGHT;    // y_0
        optimizer->H[2*STATE_SIZE + 2] += ANCHOR_WEIGHT;    // theta_0


        /**
         * @note perform Cholesky decomposition of H to solve for dx:
         *   H = L * L^T
         * Then solve L * y = b (forward substitution)
         * Then solve L^T * dx = y (backward substitution)
         */
        result = Cholesky_Decompose(
                STATE_SIZE,
                state_size, 
                optimizer->H, 
                L);
        
        // Matrix not positive definite, thus optimization cannot proceed
        if (result != 0) {
            
#ifdef DEBUG_OUTPUTS
            printf("Cholesky failed at iteration %d\n", iter);
#endif
            break;
        }
        
        // Gauss-Newton: solve H * dx = b
        // Where b = -J^T * Omega * error (negative gradient)
        // This gives dx = H^{-1} * (-J^T * Omega * error) = -H^{-1} * gradient
        // Which is the correct Newton descent direction
        
        // Forward substitution: L * y = b
        Cholesky_Forward_Substitution(
                STATE_SIZE,
                state_size, 
                L, 
                optimizer->b, 
                y);
        
        // Backward substitution: L^T * dx = y
        Cholesky_Backward_Substitution(
                STATE_SIZE,
                state_size, 
                L, 
                y, 
                dx);
        
        // Check convergence
        dx_norm = 0.0f;
        for (i = 0; i < state_size; i++) {
            dx_norm += dx[i] * dx[i];
        }
        dx_norm = sqrtf(dx_norm);
        
        // Update state
        for (i = 0; i < state_size; i++) {
            optimizer->state[i] += dx[i];
        }
        
        // Normalize all theta values to [-pi, pi]
        for (p = 0; p < optimizer->buffer_size; p++) {
            optimizer->state[p*3 + 2] = \
                    normalize_angle(optimizer->state[p*3 + 2]);
        }
        
        // NOTE: Do NOT write state back to `pose_pool`!
        // `pose_pool` contains raw input poses for error calculation.
        // state contains optimized poses for output.
        // Use `slam_get_optimized_pose()` to retrieve corrected poses.
        
        if (dx_norm < CONVERGENCE_TOLERANCE) {
#ifdef DEBUG_OUTPUTS
            printf("Gauss-Newton converged at iteration %d, dx_norm=%.6f\n",
                   iter,
                   dx_norm);
#endif
            break;
        }
        
        // Continue to next iteration (H/b will be rebuilt with updated state)
    }
    
#ifdef DEBUG_OUTPUTS
        printf("\n");
#endif

}


// ────────────────────────────────────────────────────────────────────────────
// 
//  GETTER FUNCTIONS
// 
// ────────────────────────────────────────────────────────────────────────────

void slam_get_current_pose(
        const SLAMOptimizer*    optimizer,
              se2_t*             out_pose)
{

    if (optimizer->buffer_size > 0) {

        // Return optimized pose from state vector
        int idx = (optimizer->buffer_size - 1) * 3;
        out_pose->x     = optimizer->state[idx + 0];
        out_pose->y     = optimizer->state[idx + 1];
        out_pose->theta = optimizer->state[idx + 2];
    }
}

uint8_t slam_get_pose(
        const SLAMOptimizer *optimizer,
              int           pose_id,
              se2_t         *out_pose)
{
    int idx;

    if (    (pose_id < 0)
         || (pose_id >= optimizer->buffer_size)) {
        return SLAM_FAILURE;
    }
    
    // Return optimized pose from state vector
    idx = pose_id * 3;
    out_pose->x     = optimizer->state[idx + 0];
    out_pose->y     = optimizer->state[idx + 1];
    out_pose->theta = optimizer->state[idx + 2];

    return SLAM_SUCCESS;
}

uint8_t slam_get_scan(
        const SLAMOptimizer *optimizer,
        int                 pose_id,
        PointCloud          *out_scan)
{
    if (    (pose_id < 0)
         || (pose_id >= optimizer->buffer_size)) {
        return SLAM_FAILURE;
    }
    
    // Direct array access - no circular buffer
    *out_scan = optimizer->scan_pool[pose_id];

    return SLAM_SUCCESS;
}























// #define PL_ICP 1

#ifdef PL_ICP

void slam_perform_icp_play(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const se2_t*        initial_guess,
              ICPResult*    result)
{

    // output variables
    float R_t[TOTAL];  // 2x2 rotation matrix (row-major)

#ifdef DEBUG_OUTPUT     
    PRINTF("MAX_ICP_RANGE: %.2f\n", MAX_ICP_RANGE);
    PRINTF("ICP_MAX_CORR_DIST: %.2f\n", ICP_MAX_CORR_DIST);
#endif

    se2_t init_guess = *initial_guess;    // starter guess 
    se2_t full_transform;
    
    
    // If initial guess is non-zero, pre-transform scan1 so ICP only has to
    // find the small residual correction, then compose the two to recover
    // the full transformation: `z_ij` = initial_guess + residual
    if (    (fabsf(init_guess.x    ) > 1e-3f)
         || (fabsf(init_guess.y    ) > 1e-3f)
         || (fabsf(init_guess.theta) > 1e-3f))
    {
        se2_t icp_residual;
        
        // tune the initial guess for better convergence, less overshoot, and, within limiting factors, the lowest amount of iterations required for convergence. increasing this ratio doesn't necessarily lessen iterations
        init_guess.x       *=   0.8f;
        init_guess.y       *=   0.8f;
        init_guess.theta   *=   1.f;
        
        // Pre-transform scan1 with the initial guess
        PointCloud transformed_scan1;
        transform_point_cloud(scan1, &init_guess, &transformed_scan1);

#ifdef PROCESSING4_OUTPUT
        numpy_format_print((se2_t){0.f, 0.f, 0.f},
                           &transformed_scan1);
#endif
        
        // ICP finds the residual correction between scan1' and scan2
        ICP_2D_play(transformed_scan1.points, transformed_scan1.num_pts,
                    (Point2D*)scan2->points, scan2->num_pts,
                    MAX_ICP_ITERATIONS,
                    ICP_CONVERGENCE_TOLERANCE,
                    
                    &result->num_iterations,
                    R_t);

        // Compose initial_guess + icp_residual to get the full transformation.
        // compose_poses(p2, p1, result) computes result = p1 + p2
        icp_residual    = (se2_t){R_t[T_x_],
                                  R_t[T_y_],
                                  atan2f(R_t[R_10], R_t[R_00])};
        
        compose_poses(&init_guess, &icp_residual, &full_transform);
    

    // ICP finds the full transformation directly
    } else {

        ICP_2D_play((Point2D*)scan1->points, scan1->num_pts,
                    (Point2D*)scan2->points, scan2->num_pts,
                    MAX_ICP_ITERATIONS,
                    ICP_CONVERGENCE_TOLERANCE,
    
                    &result->num_iterations,
                    R_t);

        full_transform  = (se2_t){R_t[T_x_],
                                  R_t[T_y_],
                                  atan2f(R_t[R_10], R_t[R_00])};
    }

    result->dx      = full_transform.x;
    result->dy      = full_transform.y;
    result->dtheta  = full_transform.theta;
    result->valid   = true;

}





#else 






void slam_perform_icp_play(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const se2_t*        initial_guess,
              ICPResult*    result)
{

    // output variables
    float R_t[TOTAL];  // 2x2 rotation matrix (row-major)

#ifdef DEBUG_OUTPUT     
    PRINTF("MAX_ICP_RANGE: %.2f\n", MAX_ICP_RANGE);
    PRINTF("ICP_MAX_CORR_DIST: %.2f\n", ICP_MAX_CORR_DIST);
#endif


    se2_t init_guess_aligned;    // starter guess 
    se2_t uninverted_transform;

    // invert_pose(initial_guess, &init_guess_aligned);

    init_guess_aligned = *initial_guess;
    
    // If initial guess is non-zero, pre-transform scan1 so ICP only has to
    // find the small residual correction, then compose the two to recover
    // the full transformation: `z_ij` = initial_guess + residual
    if (    (fabsf(init_guess_aligned.x    ) > 1e-3f)
         || (fabsf(init_guess_aligned.y    ) > 1e-3f)
         || (fabsf(init_guess_aligned.theta) > 1e-3f))
    {
        se2_t icp_residual;
        se2_t full_transform;
        
        // tune the initial guess for better convergence, less overshoot, and, within limiting factors, the lowest amount of iterations required for convergence. increasing this ratio doesn't necessarily lessen iterations
        init_guess_aligned.x       *=  0.8f;
        init_guess_aligned.y       *=  0.8f;
        init_guess_aligned.theta   *=  1.f; 
        
        // Pre-transform scan1 with the initial guess
        PointCloud transformed_scan1;
        transform_point_cloud(scan1, &init_guess_aligned, &transformed_scan1);

#ifdef PROCESSING4_OUTPUT
    C_format_print((se2_t){0.f, 0.f, 0.f},
                   &transformed_scan1);
#endif
        
        
        // ICP finds the residual correction between scan1' and scan2
        ICP_2D_play(transformed_scan1.points, transformed_scan1.num_pts,
                    (Point2D*)scan2->points, scan2->num_pts,
                    MAX_ICP_ITERATIONS,
                    ICP_CONVERGENCE_TOLERANCE,
                    
                    &result->num_iterations,
                    R_t);

        // Compose initial_guess + icp_residual to get the full transformation.
        // compose_poses(p2, p1, result) computes result = p1 + p2
        icp_residual   = (se2_t){R_t[T_x_],
                                 R_t[T_y_],
                                 atan2f(R_t[R_10], R_t[R_00])};
        
        compose_poses(&icp_residual, &init_guess_aligned, &full_transform);

        invert_pose(&full_transform, &uninverted_transform);
    

    // ICP finds the full transformation directly
    } else {

        ICP_2D_play((Point2D*)scan1->points, scan1->num_pts,
                    (Point2D*)scan2->points, scan2->num_pts,
                    MAX_ICP_ITERATIONS,
                    ICP_CONVERGENCE_TOLERANCE,
    
                    &result->num_iterations,
                    R_t);

        se2_t inverted_guess = {R_t[T_x_],
                                R_t[T_y_],
                                atan2f(R_t[R_10], R_t[R_00])};

        invert_pose(&inverted_guess, &uninverted_transform);
    }

    result->dx      = uninverted_transform.x;
    result->dy      = uninverted_transform.y;
    result->dtheta  = uninverted_transform.theta;
    result->valid   = true;

}

#endif // PL_ICP