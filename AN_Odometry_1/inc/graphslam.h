
#ifndef __GRAPHSLAM_H__
#define __GRAPHSLAM_H__

//#include "matrices.h"
#include "coordinate_transform.h"
#include "cholesky_decomposition.h"
#include "data_structures.h"
#include "icp_2d.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


// Memory configuration for MSP432
#define MAX_POSES               100
#define MAX_POINTS_PER_SCAN     90      // RPLiDAR C1 reduced scan
#define STATE_SIZE              (3 * MAX_POSES)
#define EST_NNZ_PER_POSE        12      // Estimated non-zeros per pose

// Loop closure detection parameters
#define LOOP_DISTANCE_THRESHOLD 2.0f    // meters
#define ICP_CONFIDENCE_THRESHOLD 0.8f
#define MIN_TEMPORAL_GAP        20      // poses
#define OPTIMIZE_INTERVAL       10      // poses

// Optimization parameters
#define MAX_GAUSS_NEWTON_ITERS  5
#define CONVERGENCE_TOLERANCE   1e-4f

// Math constant (if not defined by system)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// ============================================================================
// Data Structures
// ============================================================================



/**
 * @brief Sparse matrix representation for memory efficiency
 */
typedef struct {
    int     *row_indices;
    int     *col_indices;
    float   *values;
    int     nnz;            // Number of non-zero elements
    int     capacity;       // Allocated capacity
    int     rows;
    int     cols;
} SparseMatrix;


/**
 * @brief Main SLAM optimizer state
 */
typedef struct {
    // Pose and scan storage
    Pose        pose_pool[MAX_POSES];
    PointCloud  scan_pool[MAX_POSES];
    int         current_pose_count;
    
    // Sparse matrices for optimization
    SparseMatrix    H;          // Information matrix (Hessian)
    float           b[STATE_SIZE];      // RHS vector
    float           state[STATE_SIZE];  // State vector [x0,y0,θ0, x1,y1,θ1, ...]
    
    // Optimization control
    bool        matrices_initialized;
    bool        optimization_requested;
    uint32_t    last_optimization_time;
    
} SLAMOptimizer;


// ============================================================================
// Core SLAM Functions
// ============================================================================

/**
 * @brief Initialize the SLAM optimizer
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 */
void slam_initialize(SLAMOptimizer *optimizer);


/**
 * @brief Clean up and free allocated memory
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 */
void slam_cleanup(SLAMOptimizer *optimizer);


/**
 * @brief Add a new pose and associated scan to the SLAM system
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param pose New pose to add
 * @param scan Point cloud data at this pose
 * @return true if successful, false if storage is full
 */
bool slam_add_pose(
    SLAMOptimizer   *optimizer,
    const Pose      *pose,
    const PointCloud *scan);


/**
 * @brief Add an odometry constraint between consecutive poses
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param pose1_id Index of first pose
 * @param pose2_id Index of second pose
 * @param dx Relative x displacement
 * @param dy Relative y displacement
 * @param dtheta Relative angle change
 * @param confidence Measurement confidence (higher = more certain)
 */
void slam_add_odometry_constraint(
    SLAMOptimizer   *optimizer,
    int             pose1_id,
    int             pose2_id,
    float           dx,
    float           dy,
    float           dtheta,
    float           confidence);


/**
 * @brief Add an ICP-based constraint between poses
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param pose1_id Index of first pose
 * @param pose2_id Index of second pose
 * @param icp_result ICP alignment result
 * @param base_confidence Base confidence value
 */
void slam_add_icp_constraint(
    SLAMOptimizer   *optimizer,
    int             pose1_id,
    int             pose2_id,
    const ICPResult *icp_result,
    float           base_confidence);


/**
 * @brief Detect and add loop closure constraints
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param current_pose_id Current pose index to check for loop closures
 * @return true if loop closure was detected and added
 */
bool slam_detect_loop_closure(
    SLAMOptimizer   *optimizer,
    int             current_pose_id);


/**
 * @brief Perform Gauss-Newton optimization on the pose graph
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param max_iterations Maximum optimization iterations
 */
void slam_optimize_gauss_newton(
    SLAMOptimizer   *optimizer,
    int             max_iterations);


/**
 * @brief Get the current (most recent) pose
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param out_pose Output pose structure
 */
void slam_get_current_pose(
    const SLAMOptimizer *optimizer,
    Pose                *out_pose);


/**
 * @brief Get a specific pose by index
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param pose_id Index of pose to retrieve
 * @param out_pose Output pose structure
 * @return true if pose_id is valid
 */
bool slam_get_pose(
    const SLAMOptimizer *optimizer,
    int                 pose_id,
    Pose                *out_pose);


/**
 * @brief Get the point cloud for a specific pose
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param pose_id Index of pose
 * @param out_scan Output point cloud structure
 * @return true if pose_id is valid
 */
bool slam_get_scan(
    const SLAMOptimizer *optimizer,
    int                 pose_id,
    PointCloud          *out_scan);


// ============================================================================
// Sparse Matrix Utilities
// ============================================================================

/**
 * @brief Initialize a sparse matrix
 * 
 * @param mat Pointer to SparseMatrix structure
 * @param rows Number of rows
 * @param cols Number of columns
 * @param estimated_nnz Estimated number of non-zeros
 */
void sparse_matrix_init(
    SparseMatrix    *mat,
    int             rows,
    int             cols,
    int             estimated_nnz);


/**
 * @brief Free sparse matrix memory
 * 
 * @param mat Pointer to SparseMatrix structure
 */
void sparse_matrix_free(SparseMatrix *mat);


/**
 * @brief Add or update a value in the sparse matrix
 * 
 * @param mat Pointer to SparseMatrix structure
 * @param row Row index
 * @param col Column index
 * @param value Value to add (will be added to existing value)
 */
void sparse_matrix_add(
    SparseMatrix    *mat,
    int             row,
    int             col,
    float           value);


/**
 * @brief Get a value from the sparse matrix
 * 
 * @param mat Pointer to SparseMatrix structure
 * @param row Row index
 * @param col Column index
 * @return Value at (row, col), or 0.0 if not found
 */
float sparse_matrix_get(
    const SparseMatrix  *mat,
    int                 row,
    int                 col);


/**
 * @brief Clear all values in the sparse matrix
 * 
 * @param mat Pointer to SparseMatrix structure
 */
void sparse_matrix_clear(SparseMatrix *mat);


// ============================================================================
// ICP Integration Functions
// ============================================================================

/**
 * @brief Perform ICP alignment between two scans
 * 
 * @param scan1 First point cloud
 * @param scan2 Second point cloud
 * @param initial_guess Initial transformation guess
 * @param result Output ICP result
 */
void slam_perform_icp(
    const PointCloud    *scan1,
    const PointCloud    *scan2,
    const Pose          *initial_guess,
    ICPResult           *result);


/**
 * @brief Compute confidence metric for ICP result
 * 
 * @param scan1 First point cloud
 * @param scan2 Second point cloud
 * @param result ICP transformation result
 * @return Confidence value [0-1]
 */
float slam_compute_icp_confidence(
    const PointCloud    *scan1,
    const PointCloud    *scan2,
    const ICPResult     *result);


// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Normalize angle to [-π, π]
 * 
 * @param angle Input angle in radians
 * @return Normalized angle
 */
float normalize_angle(float angle);


/**
 * @brief Compute Euclidean distance between two poses
 * 
 * @param pose1 First pose
 * @param pose2 Second pose
 * @return Distance in meters
 */
float pose_distance(
    const Pose *pose1,
    const Pose *pose2);


/**
 * @brief Transform a point cloud by a pose
 * 
 * @param scan Input point cloud
 * @param pose Transformation pose
 * @param out_scan Output transformed point cloud
 */
void transform_point_cloud(
    const PointCloud    *scan,
    const Pose          *pose,
    PointCloud          *out_scan);


/**
 * @brief Compose two poses (p1 ⊕ p2)
 * 
 * @param p1 First pose
 * @param p2 Second pose (relative to p1)
 * @param result Output composed pose
 */
void compose_poses(
    const Pose  *p1,
    const Pose  *p2,
    Pose        *result);


/**
 * @brief Compute relative pose from p1 to p2
 * 
 * @param p1 First pose
 * @param p2 Second pose
 * @param relative Output relative pose
 */
void relative_pose(
    const Pose  *p1,
    const Pose  *p2,
    Pose        *relative);


// ============================================================================
// Error and Jacobian Functions
// ============================================================================

/**
 * @brief Evaluate pose-pose error
 * 
 * @param x_i First pose
 * @param x_j Second pose
 * @param z_ij Observed relative pose [dx, dy, dtheta]
 * @param error Output error vector [3]
 */
void evaluate_error_pose_pose(
    const Pose  *x_i,
    const Pose  *x_j,
    const float z_ij[3],
    float       error[3]);


/**
 * @brief Compute Jacobian of pose-pose error
 * 
 * @param x_i First pose
 * @param x_j Second pose
 * @param A Output Jacobian w.r.t. x_i [3x3]
 * @param B Output Jacobian w.r.t. x_j [3x3]
 */
void compute_jacobian_pose_pose(
    const Pose  *x_i,
    const Pose  *x_j,
    float       A[3][3],
    float       B[3][3]);


#endif // __GRAPHSLAM_H__
