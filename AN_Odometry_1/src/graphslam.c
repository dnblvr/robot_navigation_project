/**
 * @file graphslam.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "../inc/graphslam.h"
#include "../inc/data_structures.h"
#include <string.h>
#include <float.h>

// ============================================================================
// Sparse Matrix Implementation
// ============================================================================

void sparse_matrix_init(
    SparseMatrix    *mat,
    int             rows,
    int             cols,
    int             estimated_nnz)
{
    mat->rows = rows;
    mat->cols = cols;
    mat->capacity = estimated_nnz;
    mat->nnz = 0;
    
    mat->row_indices = (int*)malloc(estimated_nnz * sizeof(int));
    mat->col_indices = (int*)malloc(estimated_nnz * sizeof(int));
    mat->values = (float*)malloc(estimated_nnz * sizeof(float));
    
    if (!mat->row_indices || !mat->col_indices || !mat->values) {
        // Handle allocation failure
        if (mat->row_indices) free(mat->row_indices);
        if (mat->col_indices) free(mat->col_indices);
        if (mat->values) free(mat->values);
        mat->capacity = 0;
    }
}

void sparse_matrix_free(SparseMatrix *mat)
{
    if (mat->row_indices) free(mat->row_indices);
    if (mat->col_indices) free(mat->col_indices);
    if (mat->values) free(mat->values);
    
    mat->row_indices = NULL;
    mat->col_indices = NULL;
    mat->values = NULL;
    mat->nnz = 0;
    mat->capacity = 0;
}

void sparse_matrix_add(
    SparseMatrix    *mat,
    int             row,
    int             col,
    float           value)
{
    int i;
    
    // Look for existing entry
    for (i = 0; i < mat->nnz; i++) {
        if (mat->row_indices[i] == row && mat->col_indices[i] == col) {
            mat->values[i] += value;
            return;
        }
    }
    
    // Add new entry if capacity available
    if (mat->nnz < mat->capacity) {
        mat->row_indices[mat->nnz] = row;
        mat->col_indices[mat->nnz] = col;
        mat->values[mat->nnz] = value;
        mat->nnz++;
    }
}

float sparse_matrix_get(
    const SparseMatrix  *mat,
    int                 row,
    int                 col)
{
    int i;
    
    for (i = 0; i < mat->nnz; i++) {
        if (mat->row_indices[i] == row && mat->col_indices[i] == col) {
            return mat->values[i];
        }
    }
    return 0.0f;
}

void sparse_matrix_clear(SparseMatrix *mat)
{
    mat->nnz = 0;
}

// ============================================================================
// Helper Functions
// ============================================================================

float normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float pose_distance(
    const Pose *pose1,
    const Pose *pose2)
{
    float dx = pose2->x - pose1->x;
    float dy = pose2->y - pose1->y;
    return sqrtf(dx*dx + dy*dy);
}

void compose_poses(
    const Pose  *p1,
    const Pose  *p2,
    Pose        *result)
{
    float c = cosf(p1->theta);
    float s = sinf(p1->theta);
    
    result->x = p1->x + c * p2->x - s * p2->y;
    result->y = p1->y + s * p2->x + c * p2->y;
    result->theta = normalize_angle(p1->theta + p2->theta);
}

void relative_pose(
    const Pose  *p1,
    const Pose  *p2,
    Pose        *relative)
{
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    float c = cosf(-p1->theta);
    float s = sinf(-p1->theta);
    
    relative->x = c * dx - s * dy;
    relative->y = s * dx + c * dy;
    relative->theta = normalize_angle(p2->theta - p1->theta);
}

void transform_point_cloud(
    const PointCloud    *scan,
    const Pose          *pose,
    PointCloud          *out_scan)
{
    int i;
    float c = cosf(pose->theta);
    float s = sinf(pose->theta);
    
    out_scan->num_points = scan->num_points;
    
    for (i = 0; i < scan->num_points; i++) {
        float x = scan->points[i].x;
        float y = scan->points[i].y;
        
        out_scan->points[i].x = pose->x + c * x - s * y;
        out_scan->points[i].y = pose->y + s * x + c * y;
    }
}

// ============================================================================
// Error and Jacobian Functions
// ============================================================================

void evaluate_error_pose_pose(
    const Pose  *x_i,
    const Pose  *x_j,
    const float z_ij[3],
    float       error[3])
{
    // Compute the relative pose from i to j according to current estimate
    float dx = x_j->x - x_i->x;
    float dy = x_j->y - x_i->y;
    float dtheta = x_j->theta - x_i->theta;
    
    float ci = cosf(-x_i->theta);
    float si = sinf(-x_i->theta);
    
    // Transform difference into frame of x_i
    float rel_x = ci * dx - si * dy;
    float rel_y = si * dx + ci * dy;
    float rel_theta = dtheta;
    
    // Error is difference between predicted and observed
    error[0] = rel_x - z_ij[0];
    error[1] = rel_y - z_ij[1];
    error[2] = rel_theta - z_ij[2];
    
    // Normalize angle error
    error[2] = normalize_angle(error[2]);
}

void compute_jacobian_pose_pose(
    const Pose  *x_i,
    const Pose  *x_j,
    float       A[3][3],
    float       B[3][3])
{
    float si = sinf(x_i->theta);
    float ci = cosf(x_i->theta);
    
    float dx = x_j->x - x_i->x;
    float dy = x_j->y - x_i->y;
    
    // Jacobian w.r.t. x_i (A)
    A[0][0] = -ci;
    A[0][1] = -si;
    A[0][2] = si * dx - ci * dy;
    
    A[1][0] = si;
    A[1][1] = -ci;
    A[1][2] = -ci * dx - si * dy;
    
    A[2][0] = 0.0f;
    A[2][1] = 0.0f;
    A[2][2] = -1.0f;
    
    // Jacobian w.r.t. x_j (B)
    B[0][0] = ci;
    B[0][1] = si;
    B[0][2] = 0.0f;
    
    B[1][0] = -si;
    B[1][1] = ci;
    B[1][2] = 0.0f;
    
    B[2][0] = 0.0f;
    B[2][1] = 0.0f;
    B[2][2] = 1.0f;
}

// ============================================================================
// ICP Integration
// ============================================================================

void slam_perform_icp(
    const PointCloud    *scan1,
    const PointCloud    *scan2,
    const Pose          *initial_guess,
    ICPResult           *result)
{
    // Transform scan1 by initial guess
    PointCloud transformed_scan1;
    transform_point_cloud(scan1, initial_guess, &transformed_scan1);
    
    // Run ICP
    float R[4];  // 2x2 rotation matrix (row-major)
    float t[2];  // translation vector
    
    icp_2d(
        transformed_scan1.points, transformed_scan1.num_points,
        (Point2D*)scan2->points, scan2->num_points,
        20,     // max iterations
        1e-4f,  // tolerance
        R, t
    );
    
    // Extract transformation
    result->dx = t[0];
    result->dy = t[1];
    result->dtheta = atan2f(R[2], R[0]);  // Extract angle from rotation matrix
    result->valid = true;
}

float slam_compute_icp_confidence(
    const PointCloud    *scan1,
    const PointCloud    *scan2,
    const ICPResult     *result)
{
    int i, j;
    float min_dist, dx, dy, dist;
    float total_error, mean_error, match_ratio;
    float error_confidence, confidence;
    int matches;
    Pose transform;
    PointCloud transformed;
    
    // Transform scan1 by ICP result
    transform.x = result->dx;
    transform.y = result->dy;
    transform.theta = result->dtheta;
    
    transform_point_cloud(scan1, &transform, &transformed);
    
    // Compute mean correspondence distance
    total_error = 0.0f;
    matches = 0;
    
    for (i = 0; i < transformed.num_points; i++) {
        min_dist = FLT_MAX;
        
        for (j = 0; j < scan2->num_points; j++) {
            dx = transformed.points[i].x - scan2->points[j].x;
            dy = transformed.points[i].y - scan2->points[j].y;
            dist = sqrtf(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        
        if (min_dist < 0.5f) {  // Threshold for valid match
            total_error += min_dist;
            matches++;
        }
    }
    
    if (matches == 0) return 0.0f;
    
    mean_error = total_error / matches;
    match_ratio = (float)matches / transformed.num_points;
    
    // Confidence based on error and match ratio
    error_confidence = expf(-mean_error * 2.0f);
    confidence = error_confidence * match_ratio;
    
    return confidence;
}

// ============================================================================
// Core SLAM Functions
// ============================================================================

void slam_initialize(SLAMOptimizer *optimizer)
{
    optimizer->current_pose_count = 0;
    optimizer->matrices_initialized = false;
    optimizer->optimization_requested = false;
    optimizer->last_optimization_time = 0;
    
    // Initialize sparse matrix
    int matrix_size = STATE_SIZE;
    int estimated_nnz = MAX_POSES * EST_NNZ_PER_POSE;
    
    sparse_matrix_init(&optimizer->H, matrix_size, matrix_size, estimated_nnz);
    
    // Initialize vectors
    memset(optimizer->b, 0, STATE_SIZE * sizeof(float));
    memset(optimizer->state, 0, STATE_SIZE * sizeof(float));
    
    optimizer->matrices_initialized = true;
}

void slam_cleanup(SLAMOptimizer *optimizer)
{
    sparse_matrix_free(&optimizer->H);
    optimizer->matrices_initialized = false;
}

bool slam_add_pose(
    SLAMOptimizer   *optimizer,
    const Pose      *pose,
    const PointCloud *scan)
{
    if (optimizer->current_pose_count >= MAX_POSES) {
        return false;
    }
    
    int idx = optimizer->current_pose_count;
    
    // Store pose and scan
    optimizer->pose_pool[idx] = *pose;
    optimizer->scan_pool[idx] = *scan;
    
    // Update state vector
    int state_idx = idx * 3;
    optimizer->state[state_idx + 0] = pose->x;
    optimizer->state[state_idx + 1] = pose->y;
    optimizer->state[state_idx + 2] = pose->theta;
    
    optimizer->current_pose_count++;
    
    return true;
}

void slam_add_odometry_constraint(
    SLAMOptimizer   *optimizer,
    int             pose1_id,
    int             pose2_id,
    float           dx,
    float           dy,
    float           dtheta,
    float           confidence)
{
    int i, j, k, i1, i2;
    float info_xy, info_theta;
    float z_ij[3];
    float error[3];
    float A[3][3], B[3][3];
    float omega[3];
    float sum;
    Pose *p1, *p2;
    
    if (!optimizer->matrices_initialized) return;
    if (pose1_id >= optimizer->current_pose_count) return;
    if (pose2_id >= optimizer->current_pose_count) return;
    
    i1 = pose1_id * 3;
    i2 = pose2_id * 3;
    
    // Information matrix weights
    info_xy = confidence;
    info_theta = confidence * 0.5f;
    
    // Compute error
    p1 = &optimizer->pose_pool[pose1_id];
    p2 = &optimizer->pose_pool[pose2_id];
    
    z_ij[0] = dx;
    z_ij[1] = dy;
    z_ij[2] = dtheta;
    evaluate_error_pose_pose(p1, p2, z_ij, error);
    
    // Compute Jacobians
    compute_jacobian_pose_pose(p1, p2, A, B);
    
    // Add to information matrix H and vector b
    // H += A^T * Omega * A + B^T * Omega * B + A^T * Omega * B + B^T * Omega * A
    // b += -A^T * Omega * error - B^T * Omega * error
    
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
            sparse_matrix_add(&optimizer->H, i1 + i, i1 + j, sum);
        }
    }
    
    // B^T * Omega * B
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += B[k][i] * omega[k] * B[k][j];
            }
            sparse_matrix_add(&optimizer->H, i2 + i, i2 + j, sum);
        }
    }
    
    // A^T * Omega * B (off-diagonal blocks)
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += A[k][i] * omega[k] * B[k][j];
            }
            sparse_matrix_add(&optimizer->H, i1 + i, i2 + j, sum);
        }
    }
    
    // B^T * Omega * A (off-diagonal blocks)
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += B[k][i] * omega[k] * A[k][j];
            }
            sparse_matrix_add(&optimizer->H, i2 + i, i1 + j, sum);
        }
    }
}

void slam_add_icp_constraint(
    SLAMOptimizer   *optimizer,
    int             pose1_id,
    int             pose2_id,
    const ICPResult *icp_result,
    float           base_confidence)
{
    if (!icp_result->valid) return;
    
    float adjusted_confidence = base_confidence * icp_result->confidence;
    
    slam_add_odometry_constraint(
        optimizer,
        pose1_id,
        pose2_id,
        icp_result->dx,
        icp_result->dy,
        icp_result->dtheta,
        adjusted_confidence
    );
}

bool slam_detect_loop_closure(
    SLAMOptimizer   *optimizer,
    int             current_pose_id)
{
    int candidate;
    float dist, confidence;
    Pose *current_pose, *candidate_pose;
    PointCloud *current_scan, *candidate_scan;
    Pose initial_guess;
    ICPResult icp_result;
    
    if (current_pose_id < MIN_TEMPORAL_GAP) return false;
    
    current_pose = &optimizer->pose_pool[current_pose_id];
    current_scan = &optimizer->scan_pool[current_pose_id];
    
    // Check spatially close poses
    for (candidate = 0; candidate < current_pose_id - MIN_TEMPORAL_GAP; candidate++) {
        candidate_pose = &optimizer->pose_pool[candidate];
        
        dist = pose_distance(current_pose, candidate_pose);
        
        if (dist < LOOP_DISTANCE_THRESHOLD) {
            // Attempt ICP alignment
            candidate_scan = &optimizer->scan_pool[candidate];
            
            relative_pose(candidate_pose, current_pose, &initial_guess);
            
            slam_perform_icp(current_scan, candidate_scan, &initial_guess, &icp_result);
            
            confidence = slam_compute_icp_confidence(
                current_scan, candidate_scan, &icp_result
            );
            icp_result.confidence = confidence;
            
            if (confidence > ICP_CONFIDENCE_THRESHOLD) {
                // Add loop closure constraint with high confidence
                slam_add_icp_constraint(
                    optimizer,
                    candidate,
                    current_pose_id,
                    &icp_result,
                    200.0f  // High confidence for loop closures
                );
                
                return true;
            }
        }
    }
    
    return false;
}

void slam_optimize_gauss_newton(
    SLAMOptimizer   *optimizer,
    int             max_iterations)
{
    int i, iter, p, idx;
    int state_size;
    float *H_dense, *L, *y, *dx, *neg_b;
    int8_t result;
    float dx_norm;
    
    if (!optimizer->matrices_initialized) return;
    if (optimizer->current_pose_count < 2) return;
    
    state_size = optimizer->current_pose_count * 3;
    
    // Convert sparse matrix to dense for Cholesky (for embedded system)
    H_dense = (float*)malloc(state_size * state_size * sizeof(float));
    if (!H_dense) return;
    
    // Initialize dense matrix to zero
    memset(H_dense, 0, state_size * state_size * sizeof(float));
    
    // Fill dense matrix from sparse
    for (i = 0; i < optimizer->H.nnz; i++) {
        int row = optimizer->H.row_indices[i];
        int col = optimizer->H.col_indices[i];
        if (row < state_size && col < state_size) {
            H_dense[row * state_size + col] = optimizer->H.values[i];
        }
    }
    
    // Gauss-Newton iterations
    for (iter = 0; iter < max_iterations; iter++) {
        // Solve H * dx = -b using Cholesky decomposition
        L = (float*)malloc(state_size * state_size * sizeof(float));
        y = (float*)malloc(state_size * sizeof(float));
        dx = (float*)malloc(state_size * sizeof(float));
        
        if (!L || !y || !dx) {
            if (L) free(L);
            if (y) free(y);
            if (dx) free(dx);
            free(H_dense);
            return;
        }
        
        // Cholesky decomposition
        result = cholesky_decompose(H_dense, L, state_size);
        
        if (result != 0) {
            // Matrix not positive definite
            free(L);
            free(y);
            free(dx);
            break;
        }
        
        // Create -b
        neg_b = (float*)malloc(state_size * sizeof(float));
        if (!neg_b) {
            free(L);
            free(y);
            free(dx);
            break;
        }
        
        for (i = 0; i < state_size; i++) {
            neg_b[i] = -optimizer->b[i];
        }
        
        // Forward substitution: L * y = -b
        forward_substitution(L, neg_b, y, state_size);
        
        // Backward substitution: L^T * dx = y
        backward_substitution(L, y, dx, state_size);
        
        free(neg_b);
        free(L);
        free(y);
        
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
        
        // Write state back to pose pool
        for (p = 0; p < optimizer->current_pose_count; p++) {
            idx = p * 3;
            optimizer->pose_pool[p].x = optimizer->state[idx + 0];
            optimizer->pose_pool[p].y = optimizer->state[idx + 1];
            optimizer->pose_pool[p].theta = optimizer->state[idx + 2];
        }
        
        free(dx);
        
        if (dx_norm < CONVERGENCE_TOLERANCE) {
            break;
        }
    }
    
    free(H_dense);
}

void slam_get_current_pose(
    const SLAMOptimizer *optimizer,
    Pose                *out_pose)
{
    if (optimizer->current_pose_count > 0) {
        *out_pose = optimizer->pose_pool[optimizer->current_pose_count - 1];
    }
}

bool slam_get_pose(
    const SLAMOptimizer *optimizer,
    int                  pose_id,
    Pose                *out_pose)
{
    if (pose_id < 0 || pose_id >= optimizer->current_pose_count) {
        return false;
    }
    
    *out_pose = optimizer->pose_pool[pose_id];
    return true;
}

bool slam_get_scan(
    const SLAMOptimizer *optimizer,
    int                 pose_id,
    PointCloud          *out_scan)
{
    if (pose_id < 0 || pose_id >= optimizer->current_pose_count) {
        return false;
    }
    
    *out_scan = optimizer->scan_pool[pose_id];
    return true;
}
