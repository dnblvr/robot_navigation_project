
#ifndef __GRAPHSLAM_H__
#define __GRAPHSLAM_H__

#include "graph_nodes.h"
#include "matrices.h"
#include "coordinate_transform.h"
#include "cholesky_decomposition.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>


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
        
        float vector[3]);


/**
 * @brief Make a transformation matrix from a PoseState structure
 * 
 * @param pose  Input pose (x, y, theta)
 * @param matrix_3x3 Output transformation matrix (3x3)
 */
void transform_to_pose(
        float matrix_3x3[3][3],
        
        PoseState *pose);

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
    float error[3]);


/**
 * @brief Compute the Jacobian of the pose-pose error with respect to both poses
 * 
 * @param x_i_pose  Pose of the first robot (xi)
 * @param x_j_pose  Pose of the second robot (xj)
 * @param A         Output Jacobian w.r.t. xi (3x3)
 * @param B         Output Jacobian w.r.t. xj (3x3)
 */
void compute_jacobian_pose_pose(
    PoseState *x_i_pose,
    PoseState *x_j_pose,
    float   A[3][3],   // d(error)/d(xi)
    float   B[3][3]);  // d(error)/d(xj)


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
        float   error[2]);


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
        float   J[2][3]);

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
        float   J[2][2]);


// --- High-level Gauss-Newton SLAM functions ---


void build_state_vector(
        GraphNode *head,
        LandmarkMeasurement **landmarks,
        float *state_vector,
        int num_poses,
        int num_landmarks);


void linearize_constraints(
        GraphNode *head,
        LandmarkMeasurement **landmarks,
        float  *state_vector,
        float  *b,
        float   H[STATE_SIZE][STATE_SIZE],
        int     num_poses,
        int     num_landmarks);


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
        int     state_size);


void update_state_vector(
        float  *state_vector,
        float  *dx,
        int     state_size);


void write_state_to_graph(
        GraphNode *head,
        LandmarkMeasurement **landmarks,
        float *state_vector,
        int num_poses,
        int num_landmarks);


void gauss_newton_slam(
        GraphNode  *head,
        LandmarkMeasurement **landmarks, // array of pointers to all landmarks
        int         num_poses,
        int         num_landmarks,
        int         max_iterations,
        float       tol);


void example_graph_nodes(void);

#endif // __GRAPHSLAM_H__