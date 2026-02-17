/**
 * @file graphslam.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 */

#include "../inc/graphslam.h"
#include "../inc/Project_Config.h"


// Forward declaration of internal helper
static void slam_apply_constraint(
        SLAMOptimizer*  optimizer,
        int             pose1_id,
        int             pose2_id,
        float           dx,
        float           dy,
        float           dtheta,
        float           confidence);


// ----------------------------------------------------------------------------
//
//  Helper Functions
//
// ----------------------------------------------------------------------------

float normalize_angle(float angle)
{
    while (angle > M_PI)
        angle -= 2.0f * M_PI;

    while (angle < -M_PI)
        angle += 2.0f * M_PI;
        
    return angle;
}

float pose_distance(
        const Pose* pose1,
        const Pose* pose2)
{
    float dx, dy;

    dx  = pose2->x - pose1->x;
    dy  = pose2->y - pose1->y;

    return sqrtf(dx*dx + dy*dy);
}


void transform_point_cloud(
        const PointCloud*   scan,
        const Pose*         pose,
              PointCloud*   out_scan)
{
    // counter
    uint32_t i;

    // Pre-compute cosine and sine of rotation angle
    float c, s;

    c = cosf(pose->theta);
    s = sinf(pose->theta);
    
    out_scan->num_pts   = scan->num_pts;

    // Apply rotation and translation
    for (i = 0; i < scan->num_pts; i++) {

        float x = scan->points[i].x;
        float y = scan->points[i].y;
        
        out_scan->points[i].x   = c * x - s * y + pose->x;
        out_scan->points[i].y   = s * x + c * y + pose->y;
    }
}


void compose_poses(
        const Pose*	p2,
        const Pose*	p1,
              Pose* result)
{
    float c, s;

    c = cosf(p1->theta);
    s = sinf(p1->theta);
    
    result->x       = c * p2->x - s * p2->y + p1->x;
    result->y       = s * p2->x + c * p2->y + p1->y;
    result->theta   = normalize_angle(p1->theta + p2->theta);

}


void relative_pose(
        const Pose  *p1,
        const Pose  *p2,
        Pose        *relative)
{
    // temporary variables
    float   dx, dy,
            c,  s;

    dx  = p2->x - p1->x;
    dy  = p2->y - p1->y;
    c   = cosf(-p1->theta);
    s   = sinf(-p1->theta);
    
    relative->x     = c * dx - s * dy;
    relative->y     = s * dx + c * dy;
    relative->theta = normalize_angle(p2->theta - p1->theta);

}


// ----------------------------------------------------------------------------
//
//  Error and Jacobian Functions
//
// ----------------------------------------------------------------------------

void evaluate_error_pose_pose(
        const Pose* x_i,
        const Pose* x_j,
        const float z_ij[3],
              float error[3])
{

    float dx, dy, dtheta;

    float ci, si;

    float rel_x, rel_y, rel_theta;


    // Compute the relative pose from i to j according to current estimate
    dx      = x_j->x - x_i->x;
    dy      = x_j->y - x_i->y;
    dtheta  = x_j->theta - x_i->theta;
    
    ci  = cosf(-x_i->theta);
    si  = sinf(-x_i->theta);
    
    // Transform difference into frame of x_i
    rel_x       = ci * dx - si * dy;
    rel_y       = si * dx + ci * dy;
    rel_theta   = dtheta;
    
    // Error is difference between predicted and observed
    error[0]    = rel_x   -   z_ij[0];
    error[1]    = rel_y   -   z_ij[1];
    error[2]    = rel_theta - z_ij[2];
    
    // Normalize angle error
    error[2] = normalize_angle(error[2]);
}


void compute_jacobian_pose_pose(
    const Pose* x_i,
    const Pose* x_j,
    float       A[3][3],
    float       B[3][3])
{

    // helper variables
    float si, ci, dx, dy;

    si  = sinf(x_i->theta);
    ci  = cosf(x_i->theta);
    
    dx  = x_j->x - x_i->x;
    dy  = x_j->y - x_i->y;

    /**
     * Jacobian w.r.t. x_i (A)
     * e_x =  cos(θ_i)*dx + sin(θ_i)*dy - z_x
     * e_y = -sin(θ_i)*dx + cos(θ_i)*dy - z_y
     * 
     * row 1: derivatives of e_x
     *      ∂e_x/∂x_i = -cos(θ_i),
     *      ∂e_x/∂y_i = -sin(θ_i),
     */

    A[0][0] = -ci;
    A[0][1] = -si;
    A[0][2] = -si*dx + ci*dy;
    
    A[1][0] =  si;
    A[1][1] = -ci;
    A[1][2] = -ci*dx - si*dy;
    
    A[2][0] =  0.0f;
    A[2][1] =  0.0f;
    A[2][2] = -1.0f;
    

    // Jacobian w.r.t. x_j (B)
    B[0][0] =  ci;
    B[0][1] =  si;
    B[0][2] = 0.0f;
    
    B[1][0] = -si;
    B[1][1] =  ci;
    B[1][2] = 0.0f;
    
    B[2][0] = 0.0f;
    B[2][1] = 0.0f;
    B[2][2] = 1.0f;
}


// ----------------------------------------------------------------------------
//
//  ICP INTEGRATION
//
// ----------------------------------------------------------------------------

void slam_perform_icp(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const Pose*         initial_guess,
              ICPResult*    result)
{

    #define MAX_ICP_ITERATIONS 25

    // output variables
    float R[4];  // 2x2 rotation matrix (row-major)
    float t[2];  // translation vector


    // OPTION A: Use initial guess (transforms scan1 first)
    // This returns the CORRECTION on top of initial guess
    // Total transform = initial_guess ⊕ icp_result
    
    // OPTION B: Don't use initial guess (work on raw scans)  
    // This returns the FULL transformation from scan1 to scan2
    // Measurement z_ij = icp_result directly
    
    // For GraphSLAM constraints, we want OPTION B - the full observation!
    // Initial guess can still help ICP converge, but we need to account for it
    
    PointCloud transformed_scan1;
    
    // If initial guess is non-zero, transform scan1 by it first
    // Then ICP result will be the correction on top
    if (initial_guess->x != 0.0f || initial_guess->y != 0.0f || 
        initial_guess->theta != 0.0f) {
        transform_point_cloud(scan1, initial_guess, &transformed_scan1);
        
        // Run ICP on transformed scan1 vs scan2
        ICP_2d(transformed_scan1.points, transformed_scan1.num_pts,
               (Point2D*)scan2->points, scan2->num_pts,
               MAX_ICP_ITERATIONS,
               1e-4f,
               R, t);
        
        // ICP returned correction on top of initial guess
        // We need to compose to get total transformation
        Pose icp_correction;
        icp_correction.x = t[0];
        icp_correction.y = t[1];
        icp_correction.theta = atan2f(R[2], R[0]);
        
        Pose total_transform;
        compose_poses(&icp_correction, initial_guess, &total_transform);
        
        result->dx = total_transform.x;
        result->dy = total_transform.y;
        result->dtheta = total_transform.theta;
        
    } else {
        // No initial guess - work on raw scans directly
        ICP_2d((Point2D*)scan1->points, scan1->num_pts,
               (Point2D*)scan2->points, scan2->num_pts,
               MAX_ICP_ITERATIONS,
               1e-4f,
               R, t);
        
        // ICP returned the full transformation
        result->dx = t[0];
        result->dy = t[1];
        result->dtheta = atan2f(R[2], R[0]);
    }
    
    result->valid = true;

}


float slam_compute_icp_confidence(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const Pose*         initial_guess,
        const ICPResult*    result)
{

    int i, j;
    float min_dist, dx, dy, dist;
    float total_error, mean_error, match_ratio;
    float error_confidence, confidence;
    int matches;
    Pose icp_transform;
    PointCloud transformed_by_guess, transformed;
    

    // First apply initial_guess to scan1 (same as what ICP saw)
    transform_point_cloud(scan1, initial_guess, &transformed_by_guess);
    
    // Then apply the ICP result transformation
    icp_transform.x     = result->dx;
    icp_transform.y     = result->dy;
    icp_transform.theta = result->dtheta;
    
    transform_point_cloud(&transformed_by_guess, &icp_transform, &transformed);
    

    // Compute mean correspondence distance
    total_error = 0.0f;
    matches     = 0;
    
    for (i = 0; i < transformed.num_pts; i++) {

        min_dist = FLT_MAX;
        
        for (j = 0; j < scan2->num_pts; j++) {
            dx  = transformed.points[i].x - scan2->points[j].x;
            dy  = transformed.points[i].y - scan2->points[j].y;
            dist = sqrtf(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        
        // if the threshold for valid match is met, add to the matches
        // Use larger threshold (100mm) to account for LiDAR noise at long range
        if (min_dist < 100.0f) {
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


    #define ERROR_CONFIDENCE_SCALE 70.0f

    
    // Confidence based on error and match ratio
    //      - exp(-error/20): 20mm error --> 0.37 confidence
    error_confidence    = expf(-mean_error / ERROR_CONFIDENCE_SCALE);
    confidence          = error_confidence * match_ratio;
    
#ifdef DEBUG_OUTPUTS
    printf("  mean_error=%.3f, error_conf=%.3f,"
           " match_ratio=%.3f, confidence=%.3f\n",
           mean_error, error_confidence, match_ratio, confidence);
#endif
    
    return confidence;

}


// -----------------------------------------------------------------------------
//
//  Core SLAM Functions
//
// -----------------------------------------------------------------------------

void slam_initialize(SLAMOptimizer* optimizer)
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

}


uint8_t slam_add_pose(
        SLAMOptimizer*      optimizer,
        const Pose*         pose,
        const PointCloud*   scan)
{
    int idx, i, j;
    int new_constraint_count;
    

    // If buffer is full, shift everything down (discard oldest)
    if (optimizer->buffer_size >= MAX_POSES) {
        
        // Shift poses and scans: move 1..MAX_POSES-1 to 0..MAX_POSES-2
        for (i = 0; i < MAX_POSES - 1; i++) {
            optimizer->pose_pool[i] = optimizer->pose_pool[i + 1];
            optimizer->scan_pool[i] = optimizer->scan_pool[i + 1];
        }
        
        // Shift state vector similarly
        for (i = 0; i < (MAX_POSES - 1) * 3; i++) {
            optimizer->state[i] = optimizer->state[i + 3];
        }
        
        // Shift constraint IDs and remove any that reference pose 0
        // (pose 0 is being discarded, so its constraints are invalid)
        new_constraint_count = 0;
        for (i = 0; i < optimizer->num_constraints; i++) {
            Constraint *c = &optimizer->constraints[i];
            
            // Skip constraints that reference pose 0 (being discarded)
            if (c->pose1_id == 0 || c->pose2_id == 0) {
                continue;
            }
            
            // Decrement pose IDs (since everything shifted down by 1)
            c->pose1_id -= 1;
            c->pose2_id -= 1;
            
            // Keep this constraint (move to front if needed)
            if (new_constraint_count != i) {
                optimizer->constraints[new_constraint_count] = *c;
            }
            new_constraint_count++;
        }
        optimizer->num_constraints = new_constraint_count;
        
        // Clear H and b - they will be rebuilt with updated constraint IDs
        memset(optimizer->H, 0, STATE_SIZE * STATE_SIZE * sizeof(float));
        memset(optimizer->b, 0, STATE_SIZE * sizeof(float));
        
        // New pose goes at the last slot
        idx = MAX_POSES - 1;
        
    } else {
        // Buffer not full - just append
        idx = optimizer->buffer_size;
        optimizer->buffer_size++;
    }
    
    // Store pose and scan at computed index
    optimizer->pose_pool[idx] = *pose;
    optimizer->scan_pool[idx] = *scan;
    
    // Update state vector for this new pose
    // IMPORTANT: Initialize state to the provided pose estimate
    // This gives optimization a good starting point, but it will adjust
    // these values to minimize constraint errors
    optimizer->state[idx * 3 + 0] = pose->x;
    optimizer->state[idx * 3 + 1] = pose->y;
    optimizer->state[idx * 3 + 2] = pose->theta;
    
    optimizer->current_pose_count++;
    
    return 1;
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
        Constraint *c = &optimizer->constraints[optimizer->num_constraints];
        c->pose1_id   = pose1_id;
        c->pose2_id   = pose2_id;
        c->dx         = dx;
        c->dy         = dy;
        c->dtheta     = dtheta;
        c->confidence = confidence;
        optimizer->num_constraints++;
    }
    // Note: H/b are rebuilt from all constraints in slam_optimize_gauss_newton()
}


/**
 * @brief Apply a single constraint to H and b matrices (internal helper)
 * 
 * Uses current state estimate (not original pose_pool) for error/Jacobian calculation.
 * This is critical for Gauss-Newton iteration to converge properly.
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
    Pose p1, p2;  // Local copies built from state vector
    

    if (!optimizer->matrices_initialized)
        return;
    if (pose1_id < 0 || pose1_id >= optimizer->buffer_size)
        return;
    if (pose2_id < 0 || pose2_id >= optimizer->buffer_size)
        return;
    

    // Direct indexing - no circular buffer
    i1  = pose1_id * 3;
    i2  = pose2_id * 3;
    

    // Information matrix weights
    info_xy     = confidence;
    info_theta  = confidence * 0.5f;
    

    // Build poses from CURRENT STATE ESTIMATE (not original pose_pool!)
    // This is essential for Gauss-Newton to work correctly across iterations
    p1.x     = optimizer->state[i1 + 0];
    p1.y     = optimizer->state[i1 + 1];
    p1.theta = optimizer->state[i1 + 2];
    
    p2.x     = optimizer->state[i2 + 0];
    p2.y     = optimizer->state[i2 + 1];
    p2.theta = optimizer->state[i2 + 2];
    
    z_ij[0] = dx;
    z_ij[1] = dy;
    z_ij[2] = dtheta;
    evaluate_error_pose_pose(&p1, &p2, z_ij, error);
    
#ifdef DEBUG_OUTPUTS
    // Debug: print error magnitude for ALL constraints to diagnose zero-error issue
    float error_mag = sqrtf(error[0]*error[0] + error[1]*error[1]);
    printf("    Constraint %d->%d: p1=(%.1f,%.1f,%.3f) p2=(%.1f,%.1f,%.3f) z=(%.1f,%.1f,%.3f) err=(%.2f,%.2f,%.3f) mag=%.2f\n",
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
            optimizer->H[i1 + i][i1 + j] += sum;
        }
    }
    
    // B^T * Omega * B
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += B[k][i] * omega[k] * B[k][j];
            }
            optimizer->H[i2 + i][i2 + j] += sum;
        }
    }
    
    // A^T * Omega * B (off-diagonal blocks)
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += A[k][i] * omega[k] * B[k][j];
            }
            optimizer->H[i1 + i][i2 + j] += sum;
        }
    }
    
    // B^T * Omega * A (off-diagonal blocks)
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += B[k][i] * omega[k] * A[k][j];
            }
            optimizer->H[i2 + i][i1 + j] += sum;
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
    // DISABLED: Loop closure d
    return 0;
    
    int candidate;
    float dist, confidence;
    Pose current_pose, candidate_pose;  // Local copies from state vector
    PointCloud *current_scan, *candidate_scan;
    Pose initial_guess;
    ICPResult icp_result;
    int idx_curr, idx_cand;
    

    // early return
    if (current_pose_id < MIN_TEMPORAL_GAP ||
        current_pose_id >= optimizer->buffer_size) {
        return 0;
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
        idx_cand                = candidate * 3;
        candidate_pose.x        = optimizer->state[idx_cand + 0];
        candidate_pose.y        = optimizer->state[idx_cand + 1];
        candidate_pose.theta    = optimizer->state[idx_cand + 2];
        
        dist = pose_distance(&current_pose, &candidate_pose);
        
#ifdef DEBUG_OUTPUTS
        printf("  Checking candidate %d (buf_idx %d), dist=%.3f (thresh=%.1f)\n",
                candidate, candidate_buffer_idx, dist, LOOP_DISTANCE_THRESHOLD);
#endif

        if (dist < LOOP_DISTANCE_THRESHOLD) {

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
                    &initial_guess,
                    &icp_result);


            icp_result.confidence = confidence;
            
            if (confidence > ICP_CONFIDENCE_THRESHOLD) {
                
                // ICP was initialized with expected relative pose (initial_guess)
                // and returns an incremental correction
                // Total transform = initial_guess composed with icp_result
                Pose icp_correction;
                Pose refined_measurement;
                
                icp_correction.x = icp_result.dx;
                icp_correction.y = icp_result.dy;
                icp_correction.theta = icp_result.dtheta;
                
                // compose: initial_guess ⊕ icp_correction
                compose_poses(&icp_correction, &initial_guess, &refined_measurement);

                // Add loop closure constraint with moderate confidence boost
                slam_add_odometry_constraint(
                        optimizer,
                        candidate,
                        current_pose_id,
                        refined_measurement.x,
                        refined_measurement.y,
                        refined_measurement.theta,
                        confidence * 5.0f  // Moderate boost over odometry
                );

    #ifdef DEBUG_OUTPUTS
                printf(" --> *** LOOP CLOSURE DETECTED: pose %d matches pose %d! ***\n",
                       current_pose_id, candidate);
                printf("     initial_guess: (%.2f, %.2f, %.3f)\n", 
                       initial_guess.x, initial_guess.y, initial_guess.theta);
                printf("     ICP correction: (%.2f, %.2f, %.3f)\n",
                       icp_correction.x, icp_correction.y, icp_correction.theta);
                printf("     refined: (%.2f, %.2f, %.3f)\n",
                       refined_measurement.x, refined_measurement.y, refined_measurement.theta);
    #endif
                
                return 1;
            }
        }
    }
    
    return 0;
}

void slam_optimize_gauss_newton(
        SLAMOptimizer   *optimizer,
        int             max_iterations)
{

    if (!optimizer->matrices_initialized)
        return;
    if (optimizer->buffer_size < 2)
        return;
    
    // Need enough constraints to form a connected graph
    // At minimum, we need (buffer_size - 1) odometry constraints to connect all poses
    // Without enough constraints, H matrix will be singular
    if (optimizer->num_constraints < optimizer->buffer_size - 1) {
#ifdef DEBUG_OUTPUTS
        printf("Optimization skipped: only %d constraints for %d poses (need %d)\n",
               optimizer->num_constraints, optimizer->buffer_size, 
               optimizer->buffer_size - 1);
#endif
        return;
    }

    int i, j, iter, p, idx, c;
    int state_size;
    int8_t result;
    float dx_norm;
    Constraint *con;

    // Static buffers for Gauss-Newton optimization
    // Size: STATE_SIZE = 3 * MAX_POSES = 45
    static float H_dense[STATE_SIZE * STATE_SIZE];
    static float L[STATE_SIZE * STATE_SIZE];
    static float y[STATE_SIZE];
    static float dx[STATE_SIZE];
    
    state_size  = optimizer->buffer_size * 3;
    
    // Gauss-Newton iteration loop
    // Each iteration: rebuild H/b from constraints using current state,
    // solve for update dx, apply update to state
    for (iter = 0; iter < max_iterations; iter++) {

#ifdef DEBUG_OUTPUTS
        printf("i-%d... ", iter);
#endif

        // Clear H and b, then rebuild from stored constraints
        // Must be done each iteration because Jacobians depend on current state
        memset(optimizer->H, 0, STATE_SIZE * STATE_SIZE * sizeof(float));
        memset(optimizer->b, 0, STATE_SIZE * sizeof(float));
        
        // Rebuild H and b from all stored constraints
        for (c = 0; c < optimizer->num_constraints; c++) {
            con = &optimizer->constraints[c];
            
            // Skip constraints that reference poses no longer in buffer
            if (con->pose1_id < 0 || con->pose1_id >= optimizer->buffer_size ||
                con->pose2_id < 0 || con->pose2_id >= optimizer->buffer_size)
                continue;
                
            slam_apply_constraint(optimizer,
                                  con->pose1_id,
                                  con->pose2_id,
                                  con->dx,
                                  con->dy,
                                  con->dtheta,
                                  con->confidence);
        }
        
        // Clear the static buffers
        memset(H_dense, 0, STATE_SIZE * STATE_SIZE * sizeof(float));
        memset(L, 0, STATE_SIZE * STATE_SIZE * sizeof(float));
        
        // Copy H from 2D array to flattened 1D array for Cholesky
        for (i = 0; i < state_size; i++) {
            for (j = 0; j < state_size; j++) {
                H_dense[i * state_size + j] = optimizer->H[i][j];
            }
        }
        
        // Fix the first pose (anchor) to prevent drift
        // Add large values to diagonal of first pose's block
        // This effectively pins pose 0 at its current location
        #define ANCHOR_WEIGHT 1000.0f
        H_dense[0 * state_size + 0] += ANCHOR_WEIGHT;  // x0
        H_dense[1 * state_size + 1] += ANCHOR_WEIGHT;  // y0
        H_dense[2 * state_size + 2] += ANCHOR_WEIGHT;  // theta0

        // Cholesky decomposition
        result = cholesky_decompose(H_dense, L, state_size);
        
        if (result != 0) {
            // Matrix not positive definite
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
        forward_substitution(L, optimizer->b, y, state_size);
        
        // Backward substitution: L^T * dx = y
        backward_substitution(L, y, dx, state_size);
        
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
            optimizer->state[p * 3 + 2] = normalize_angle(optimizer->state[p * 3 + 2]);
        }
        
        // NOTE: Do NOT write state back to pose_pool!
        // pose_pool contains raw input poses for error calculation.
        // state contains optimized poses for output.
        // Use slam_get_optimized_pose() to retrieve corrected poses.
        
        if (dx_norm < CONVERGENCE_TOLERANCE) {
#ifdef DEBUG_OUTPUTS
            printf("Gauss-Newton converged at iteration %d, dx_norm=%.6f\n", iter, dx_norm);
#endif
            break;
        }
        
        // Continue to next iteration (H/b will be rebuilt with updated state)
    }
    
#ifdef DEBUG_OUTPUTS
        printf("\n");
#endif

}

void slam_get_current_pose(
        const SLAMOptimizer *optimizer,
        Pose                *out_pose)
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
        int                  pose_id,
        Pose                *out_pose)
{
    int idx;

    if (pose_id < 0 || pose_id >= optimizer->buffer_size) {
        return 0;
    }
    
    // Return optimized pose from state vector
    idx = pose_id * 3;
    out_pose->x     = optimizer->state[idx + 0];
    out_pose->y     = optimizer->state[idx + 1];
    out_pose->theta = optimizer->state[idx + 2];

    return 1;
}

uint8_t slam_get_scan(
        const SLAMOptimizer *optimizer,
        int                 pose_id,
        PointCloud          *out_scan)
{
    if (pose_id < 0 || pose_id >= optimizer->buffer_size) {
        return 0;
    }
    
    // Direct array access - no circular buffer
    *out_scan = optimizer->scan_pool[pose_id];

    return 1;
}
