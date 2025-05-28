
#include "graphslam.h"

#include "graph_nodes.h"
#include "matrices.h"
#include "coordinate_transform.h"
#include "cholesky_decomposition.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>


// Define these as needed for your problem size
#define MAX_POSES      30
#define MAX_LANDMARKS  50
#define STATE_SIZE     (3*MAX_POSES + 2*MAX_LANDMARKS)


/**
 * @brief Make a transformation matrix from a PoseState structure
 * 
 * @param pose  Input pose (x, y, theta)
 * @param matrix_3x3 Output transformation matrix (3x3)
 */
void transform_to_vector(
        float matrix_3x3[3][3],
        
        float vector[3])
{
    // Extract translation
    vector[0] = matrix_3x3[0][2]; // x
    vector[1] = matrix_3x3[1][2]; // y

    // Extract rotation (theta)
    vector[2] = atan2f(matrix_3x3[1][0], matrix_3x3[0][0]);

}


/**
 * @brief Make a transformation matrix from a PoseState structure
 * 
 * @param pose  Input pose (x, y, theta)
 * @param matrix_3x3 Output transformation matrix (3x3)
 */
void transform_to_pose(
        float matrix_3x3[3][3],
        
        PoseState *pose)
{
    // Extract translation
    pose->x = matrix_3x3[0][2]; // x
    pose->y = matrix_3x3[1][2]; // y

    // Extract rotation (theta)
    pose->theta = atan2f(matrix_3x3[1][0], matrix_3x3[0][0]);

}

/**
 * @brief this function evaluates the error between two poses based on an observed relative pose
 * 
 * @param x_i_pose 
 * @param x_j_pose 
 * @param z_ij 
 * @param error 
 */
void evaluate_error_pose_pose(
    PoseState *x_i_pose,
    PoseState *x_j_pose,
    float z_ij[3],      // observed relative pose: [dx, dy, dtheta]
    float error[3])
{
    // Compute the relative pose from i to j according to the current estimate
    float dx        = x_j_pose->x - x_i_pose->x;
    float dy        = x_j_pose->y - x_i_pose->y;
    float dtheta    = x_j_pose->theta - x_i_pose->theta;

    float ci    = cosf(-x_i_pose->theta);
    float si    = sinf(-x_i_pose->theta);

    // Transform the difference into the frame of x_i
    float rel_x     = ci*dx - si*dy;
    float rel_y     = si*dx + ci*dy;
    float rel_theta = dtheta;

    // Error is the difference between predicted and observed relative pose
    error[0] = rel_x     - z_ij[0];
    error[1] = rel_y     - z_ij[1];
    error[2] = rel_theta - z_ij[2];

    // Normalize angle to [-pi, pi]
    while (error[2] >  M_PI) error[2] -= 2*M_PI;
    while (error[2] < -M_PI) error[2] += 2*M_PI;
}

/**
 * @brief Compute the Jacobian of the pose-pose error with respect to both poses
 * 
 * @param x_i_pose  Pose of the first robot (x_i)
 * @param x_j_pose  Pose of the second robot (x_j)
 * @param A         Output Jacobian w.r.t. x_i (3x3)
 * @param B         Output Jacobian w.r.t. x_j (3x3)
 */
void compute_jacobian_pose_pose(
    PoseState *x_i_pose,
    PoseState *x_j_pose,
    
    float   A[3][3],   // d(error)/d(xi)
    float   B[3][3])   // d(error)/d(xj)
{
    
    float si = sinf(x_i_pose->theta);
    float ci = cosf(x_i_pose->theta);

    // Relative pose in world frame
    float dx = x_j_pose->x - x_i_pose->x;
    float dy = x_j_pose->y - x_i_pose->y;

    // Transform difference into xi's frame
    float dx_r      =  ci*dx + si*dy;
    float dy_r      = -si*dx + ci*dy;
    float dtheta    = x_j_pose->theta - x_i_pose->theta;


    // Jacobian w.r.t. x_i (A)
    A[0][0] = -ci;  // d(error_x)/d(xi_x)
    A[0][1] = -si;  // d(error_x)/d(xi_y)
    A[0][2] =  si*dx - ci*dy; // d(error_x)/d(xi_theta)

    A[1][0] =  si;  // d(error_y)/d(xi_x)
    A[1][1] = -ci;  // d(error_y)/d(xi_y)
    A[1][2] = -ci*dx - si*dy; // d(error_y)/d(xi_theta)

    A[2][0] = 0.0f;
    A[2][1] = 0.0f;
    A[2][2] = -1.0f; // d(error_theta)/d(xi_theta)


    // Jacobian w.r.t. x_j (B)
    B[0][0] =  ci;   // d(error_x)/d(xj_x)
    B[0][1] =  si;   // d(error_x)/d(xj_y)
    B[0][2] = 0.0f;  // d(error_x)/d(xj_theta)

    B[1][0] = -si;   // d(error_y)/d(xj_x)
    B[1][1] =  ci;   // d(error_y)/d(xj_y)
    B[1][2] = 0.0f;  // d(error_y)/d(xj_theta)

    B[2][0] = 0.0f;
    B[2][1] = 0.0f;
    B[2][2] = 1.0f;  // d(error_theta)/d(xj_theta)
}

/**
 * @brief this function evaluates the error between a robot pose and a landmark observationa
 * 
 * @param x_i_pose 
 * @param landmark 
 * @param z_il 
 * @param error 
 */
void evaluate_error_pose_landmark(
        PoseState *x_i_pose, 
        float   landmark[2],
        float   z_il[2],    
        float   error[2])
{
    // Compute the difference in world frame
    float dx = landmark[0] - x_i_pose->x;
    float dy = landmark[1] - x_i_pose->y;

    // Transform difference into robot frame using direct trigonometry
    float ci = cosf(-x_i_pose->theta);
    float si = sinf(-x_i_pose->theta);

    float lx = ci*dx - si*dy;
    float ly = si*dx + ci*dy;

    float expected_range    = sqrtf(lx*lx + ly*ly);
    float expected_bearing  = atan2f(ly, lx);

    // Error is observed - expected
    error[0] = z_il[0] - expected_range;
    error[1] = z_il[1] - expected_bearing;

    // Normalize bearing error to [-pi, pi]
    while (error[1] >  M_PI)  error[1] -= 2*M_PI;
    while (error[1] < -M_PI)  error[1] += 2*M_PI;
}


/**
 * @brief Compute the Jacobian of the error with respect to the pose
 * 
 * @param x_i_pose  Robot pose
 * @param landmark  Landmark position [x, y]
 * @param z_il      Observation [range, bearing]
 * @param J         Output Jacobian matrix (2x3)
 */
void compute_jacobian_pose_landmark(
        PoseState *x_i_pose, 
        float   landmark[2],
        float   z_il[2],    
        float   J[2][3])
{

    // Compute the difference in world frame
    float dx = landmark[0] - x_i_pose->x;
    float dy = landmark[1] - x_i_pose->y;

    // Transform difference into robot frame using direct trigonometry
    float ci = cosf(-x_i_pose->theta);
    float si = sinf(-x_i_pose->theta);

    float lx = ci * dx - si * dy;
    float ly = si * dx + ci * dy;

    float range_sq = lx * lx + ly * ly;
    float range = sqrtf(range_sq);

    /**
     * @details the Jacobian is computed as follows:
     * J =  [ d(landmark)/d(x_i)  d(landmark)/d(y_i)   d(landmark)/d(theta_i) ]
     * 
     * J =  [    d(range)/d(x_i)     d(range)/d(y_i)      d(range)/d(theta_i) ]
     *      [  d(bearing)/d(x_i)   d(bearing)/d(y_i)    d(bearing)/d(theta_i) ]
     * 
     * where the partial derivatives of lx and ly w.r.t. x_i, y_i, theta_i:
     *      d(lx)/d(x_i) = -ci;  d(lx)/d(y_i) =  si;  d(lx)/d(theta_i) = -si*dx - ci*dy
     *      d(ly)/d(x_i) = -si;  d(ly)/d(y_i) = -ci;  d(ly)/d(theta_i) =  ci*dx - si*dy
     */


    // Range
    J[0][0] = -lx / range; // d(range)/d(x_i)
    J[0][1] = -ly / range; // d(range)/d(y_i)
    J[0][2] = (lx * (si * dx + ci * dy) - ly * (ci * dx - si * dy)) / range; // d(range)/d(theta_i)

    // Bearing
    J[1][0] =  ly / range_sq; // d(bearing)/d(x_i)
    J[1][1] = -lx / range_sq; // d(bearing)/d(y_i)
    J[1][2] = -1.0f + (lx * (ci * dx - si * dy) + ly * (si * dx + ci * dy)) / range_sq; // d(bearing)/d(theta_i)
}

/**
 * @brief Compute the Jacobian of the error with respect to the landmark position.
 * 
 * @param x_i_pose  Robot pose
 * @param landmark  Landmark position [x, y]
 * @param z_il      Observation [range, bearing]
 * @param J         Output Jacobian matrix (2x2)
 */
void compute_jacobian_landmark(
        PoseState *x_i_pose, 
        float   landmark[2],
        float   z_il[2],    
        float   J[2][2])
{
    // Compute the difference in world frame
    float dx = landmark[0] - x_i_pose->x;
    float dy = landmark[1] - x_i_pose->y;

    // Transform difference into robot frame using direct trigonometry
    float ci = cosf(-x_i_pose->theta);
    float si = sinf(-x_i_pose->theta);

    float lx = ci*dx - si*dy;
    float ly = si*dx + ci*dy;

    float range_sq = lx*lx + ly*ly;
    float range = sqrtf(range_sq);

    // Partial derivatives of lx and ly w.r.t. landmark x, y
    // d(lx)/d(lx) = 1, d(lx)/d(ly) = 0
    // d(ly)/d(lx) = 0, d(ly)/d(ly) = 1

    // But lx = ci*dx - si*dy = ci*(lx - x_i) - si*(ly - y_i)
    // So d(lx)/d(lx) = ci, d(lx)/d(ly) = -si
    //    d(ly)/d(lx) = si, d(ly)/d(ly) =  ci

    // Range
    J[0][0] =  lx / range * ci    + ly / range * si;    // d(range)/d(lx)
    J[0][1] =  lx / range * (-si) + ly / range * ci;    // d(range)/d(ly)

    // Bearing
    J[1][0] = -ly / range_sq * ci    + lx / range_sq * si;  // d(bearing)/d(lx)
    J[1][1] = -ly / range_sq * (-si) + lx / range_sq * ci;  // d(bearing)/d(ly)

    // Simplified:
    // Range
    J[0][0] =  lx / range; // d(range)/d(lx)
    J[0][1] =  ly / range; // d(range)/d(ly)
    // Bearing
    J[1][0] = -ly / range_sq; // d(bearing)/d(lx)
    J[1][1] =  lx / range_sq; // d(bearing)/d(ly)

    // But since lx, ly are already in robot frame, and derivatives are w.r.t. world frame,
    // we need to rotate the derivatives back to world frame:
    // [d/dx, d/dy]^T = R(theta) * [d/dlx, d/dly]^T

    float d_range_dlx   =  lx / range;
    float d_range_dly   =  ly / range;
    float d_bearing_dlx = -ly / range_sq;
    float d_bearing_dly =  lx / range_sq;

    // Rotate back to world frame
    J[0][0] =  ci * d_range_dlx    +  si * d_range_dly;
    J[0][1] = -si * d_range_dlx    +  ci * d_range_dly;
    J[1][0] =  ci * d_bearing_dlx  +  si * d_bearing_dly;
    J[1][1] = -si * d_bearing_dlx  +  ci * d_bearing_dly;
}


// --- High-level Gauss-Newton SLAM functions ---


void build_state_vector(
        GraphNode *head,
        LandmarkMeasurement **landmarks,
        float *state_vector,
        int num_poses,
        int num_landmarks)
{
    // Poses
    GraphNode *current = head;
    int idx = 0;
    while (current != NULL && idx < num_poses) {
        state_vector[3*idx + 0] = current->pose.x;
        state_vector[3*idx + 1] = current->pose.y;
        state_vector[3*idx + 2] = current->pose.theta;
        current = current->next;
        idx++;
    }
    // Landmarks
    for (int l = 0; l < num_landmarks; ++l) {
        state_vector[3*num_poses + 2*l + 0] = landmarks[l]->x;
        state_vector[3*num_poses + 2*l + 1] = landmarks[l]->y;
    }
}


void linearize_constraints(
        GraphNode *head,
        LandmarkMeasurement **landmarks,
        float  *state_vector,
        float  *b,
        float   H[STATE_SIZE][STATE_SIZE],
        int     num_poses,
        int     num_landmarks)
{
    GraphNode *current = head;
    int pose_index = 0;

    // --- Pose-Landmark Constraints ---
    while (current != NULL) {
        PoseState pose = {
            state_vector[3*pose_index + 0],
            state_vector[3*pose_index + 1],
            state_vector[3*pose_index + 2]
        };

        // Iterate through all observations of the current pose
        for (int i = 0; i < current->num_observations; ++i) {
            Observation *obs = &current->observations[i];
            int lidx = obs->landmark_index;

            // Safety check for valid landmark index
            if (lidx < 0 || lidx >= num_landmarks) continue;

            float *lm = &state_vector[3*num_poses + 2*lidx];

            float z_il[2] = {obs->range, obs->bearing};
            float error[2];
            float J_pose[2][3], J_lm[2][2];

            // Evaluate error and Jacobians
            evaluate_error_pose_landmark(&pose, lm, z_il, error);
            compute_jacobian_pose_landmark(&pose, lm, z_il, J_pose);
            compute_jacobian_landmark(&pose, lm, z_il, J_lm);

            // Information matrix (identity)
            float info[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};

            int pose_idx = 3*pose_index;
            int lm_idx  = 3*num_poses + 2*lidx;

            // Fill H and b blocks
            // H_ii (pose-pose)
            for (int r = 0; r < 3; ++r)
                for (int s = 0; s < 3; ++s)
                    for (int m = 0; m < 2; ++m)
                        for (int n = 0; n < 2; ++n)
                            H[pose_idx + r][pose_idx + s] += J_pose[m][r] * info[m][n] * J_pose[n][s];

            // H_ll (landmark-landmark)
            for (int r = 0; r < 2; ++r)
                for (int s = 0; s < 2; ++s)
                    for (int m = 0; m < 2; ++m)
                        for (int n = 0; n < 2; ++n)
                            H[lm_idx + r][lm_idx + s] += J_lm[m][r] * info[m][n] * J_lm[n][s];

            // H_il (pose-landmark)
            for (int r = 0; r < 3; ++r)
                for (int s = 0; s < 2; ++s)
                    for (int m = 0; m < 2; ++m)
                        for (int n = 0; n < 2; ++n)
                            H[pose_idx + r][lm_idx + s] += J_pose[m][r] * info[m][n] * J_lm[n][s];

            // H_li (landmark-pose)
            for (int r = 0; r < 2; ++r)
                for (int s = 0; s < 3; ++s)
                    for (int m = 0; m < 2; ++m)
                        for (int n = 0; n < 2; ++n)
                            H[lm_idx + r][pose_idx + s] += J_lm[m][r] * info[m][n] * J_pose[n][s];

            // b_i (pose)
            for (int r = 0; r < 3; ++r)
                for (int m = 0; m < 2; ++m)
                    b[pose_idx + r] += J_pose[m][r] * info[m][m] * error[m];

            // b_l (landmark)
            for (int r = 0; r < 2; ++r)
                for (int m = 0; m < 2; ++m)
                    b[lm_idx + r] += J_lm[m][r] * info[m][m] * error[m];
        }

        // --- Pose-Pose Constraints ---
        if (current->next != NULL) {
            PoseState *pose_i = &current->pose;
            PoseState *pose_j = &current->next->pose;

            float z_ij[3] = {current->odometry.dx, current->odometry.dy, current->odometry.dtheta};
            float error[3];
            evaluate_error_pose_pose(pose_i, pose_j, z_ij, error);

            float A[3][3], B[3][3];
            compute_jacobian_pose_pose(pose_i, pose_j, A, B);

            float info[3][3] = {{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}};

            int idx_i = 3*pose_index;
            int idx_j = 3*(pose_index+1);

            // Update H and b for pose-pose constraint
            for (int r = 0; r < 3; ++r) {
                for (int s = 0; s < 3; ++s) {
                    // H_ii
                    H[idx_i + r][idx_i + s] += A[r][s];
                    // H_ij
                    H[idx_i + r][idx_j + s] += -A[r][s];
                    // H_ji
                    H[idx_j + r][idx_i + s] += -B[r][s];
                    // H_jj
                    H[idx_j + r][idx_j + s] += B[r][s];
                }
                // b_i
                b[idx_i + r] += A[r][0]*error[0] + A[r][1]*error[1] + A[r][2]*error[2];
                // b_j
                b[idx_j + r] += B[r][0]*error[0] + B[r][1]*error[1] + B[r][2]*error[2];
            }
        }

        // Move to the next pose in the linked list
        current = current->next;
        pose_index++;
    }
}


/**
 * @brief   this function solves H x = -b using Cholesky decomposition.
 * @details H must be symmetric positive definite.
 *          H is STATE_SIZE x STATE_SIZE, b is STATE_SIZE, x is STATE_SIZE.
 * 
 * @param H 
 * @param b 
 * @param x 
 * @param state_size 
 */
void solve_linear_system(
        float   H[STATE_SIZE][STATE_SIZE],
        float  *b,
        float  *x,
        int     state_size)
{
    // Flatten H for Cholesky routines
    float   *A = (float*)malloc(state_size * state_size * sizeof(float));
    float   *L = (float*)malloc(state_size * state_size * sizeof(float));
    float   *y = (float*)malloc(state_size * sizeof(float));
    float *rhs = (float*)malloc(state_size * sizeof(float));

    for (int i = 0; i < state_size; ++i) {
        rhs[i] = -b[i];
        for (int j = 0; j < state_size; ++j)
            A[i*state_size + j] = H[i][j];
    }

    if ( cholesky_decompose(A, L, state_size) != 0 ) {
        printf("Cholesky decomposition failed: matrix not positive definite.\n");
        free(A); free(L); free(y); free(rhs);
        return;
    }

    forward_substitution(L, rhs, y, state_size);
    backward_substitution(L, y, x, state_size);

    free(A); free(L); free(y); free(rhs);
}


void update_state_vector(
        float  *state_vector,
        float  *dx,
        int     state_size)
{
    for (int i = 0; i < state_size; ++i) {
        state_vector[i] += dx[i];
    }
}


void write_state_to_graph(
        GraphNode *head,
        LandmarkMeasurement **landmarks,
        float *state_vector,
        int num_poses,
        int num_landmarks)
{
    // Poses
    GraphNode *current = head;
    int idx = 0;
    while (current != NULL && idx < num_poses) {
        current->pose.x     = state_vector[3*idx + 0];
        current->pose.y     = state_vector[3*idx + 1];
        current->pose.theta = state_vector[3*idx + 2];
        current = current->next;
        idx++;
    }
    // Landmarks
    for (int l = 0; l < num_landmarks; ++l) {
        landmarks[l]->x = state_vector[3*num_poses + 2*l + 0];
        landmarks[l]->y = state_vector[3*num_poses + 2*l + 1];
    }
}


void gauss_newton_slam(
        GraphNode  *head,
        LandmarkMeasurement **landmarks, // array of pointers to all landmarks
        int         num_poses,
        int         num_landmarks,
        int         max_iterations,
        float       tol)
{

    int     state_size = 3*num_poses + 2*num_landmarks;
    float   state_vector[state_size];
    float              b[state_size];
    float              H[state_size][state_size];
    float             dx[state_size];
    float   norm;

    build_state_vector(head, landmarks, state_vector, num_poses, num_landmarks);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Zero out b and H
        for (int i = 0; i < state_size; ++i) {
            b[i] = 0.0f;
            for (int j = 0; j < state_size; ++j)
                H[i][j] = 0.0f;
        }

        // Linearize constraints (fills H and b for both poses and landmarks)
        linearize_constraints(head, landmarks, state_vector, b, H, num_poses, num_landmarks);

        // --- Anchor the first pose (x, y, theta) ---
        float anchor_weight = 1e6f;
        for (int i = 0; i < 3; ++i) {
            H[i][i] += anchor_weight;
            b[i] += anchor_weight * (0.0f - state_vector[i]); // anchor to (0,0,0)
        }

        // --- Anchor the first landmark (x, y) ---
        int lm0_idx = 3*num_poses;
        for (int i = 0; i < 2; ++i) {
            H[lm0_idx + i][lm0_idx + i] += anchor_weight;
            b[lm0_idx + i] += anchor_weight * (0.0f - state_vector[lm0_idx + i]); // anchor to (0,0)
        }

        // Solve H dx = -b
        solve_linear_system(H, b, dx, state_size);

        // Update state vector
        update_state_vector(state_vector, dx, state_size);

        // Write back to graph and landmark structs
        write_state_to_graph(head, landmarks, state_vector, num_poses, num_landmarks);

        // Check for convergence
        norm = 0.0f;
        for (int i = 0; i < state_size; ++i)
            norm += dx[i] * dx[i];
        norm = sqrtf(norm);
        if (norm < tol)
            break;
    }
}



void example_graph_nodes(void) {

    LandmarkMeasurement *landmark1 = malloc(sizeof(LandmarkMeasurement));
    landmark1->id   = 1;
    landmark1->x    = 10.0f;
    landmark1->y    = 15.0f;

    GraphNode *node1 = malloc(sizeof(GraphNode));
    node1->pose.x       = 5.0f;
    node1->pose.y       = 5.0f;
    node1->pose.theta   = 0.0f;

    
    
    // Add an observation of the landmark
    node1->num_observations = 1;
    node1->observations     = malloc(sizeof(Observation) * node1->num_observations);
    node1->observations[0].landmark = landmark1;
    node1->observations[0].range    = 7.0f;
    node1->observations[0].bearing  = 0.5f;

    GraphNode *node2    = malloc(sizeof(GraphNode));
    node2->pose.x       = 6.0f;
    node2->pose.y       = 6.0f;
    node2->pose.theta   = 0.1f;
    
    // Add an observation of the same landmark
    node2->num_observations = 1;
    node2->observations     = malloc(sizeof(Observation) * node2->num_observations);
    node2->observations[0].landmark = landmark1;  // Same landmark as node1
    node2->observations[0].range    = 6.5f;
    node2->observations[0].bearing  = 0.4f;
    
    // Link the nodes
    node1->next = node2;
    node2->next = NULL;
}