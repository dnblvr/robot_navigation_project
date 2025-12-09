/**
 * @file graphslam.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * 
 */

#ifndef __INC_GRAPHSLAM_H__
#define __INC_GRAPHSLAM_H__

#include <inc/ICP_2D.h>
#include "Project_Config.h"


//#include "matrices.h"
#include "coordinate_transform.h"
#include "data_structures.h"
#include "cholesky_decomposition.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>


/**
 * @brief Memory configuration for MSP432
 */
#define MAX_POSES               14              // Sliding window size
#define MAX_POINTS_PER_SCAN     OUTPUT_BUFFER   // Full RPLiDAR C1 scan
#define STATE_SIZE              (3 * MAX_POSES)
#define MAX_CONSTRAINTS         (MAX_POSES * 2) // Odometry + loop closures

/**
 * @brief Loop closure detection parameters
 */
#define LOOP_DISTANCE_THRESHOLD     850.0f  // mm
#define ICP_CONFIDENCE_THRESHOLD    0.6f    // must be tuned based on sensor noise
#define MIN_TEMPORAL_GAP            5       // poses
#define OPTIMIZE_INTERVAL           1       // poses

// Optimization parameters
#define MAX_GAUSS_NEWTON_ITERS  3
#define CONVERGENCE_TOLERANCE   1e-4f

/**
 * @brief Math constant for pi (if not defined by the system) 
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// ----------------------------------------------------------------------------
//
//  Data Structures
//
// ----------------------------------------------------------------------------


/**
 * @brief Stored constraint for rebuilding H/b
 */
typedef struct {
    int     pose1_id;       // First pose index
    int     pose2_id;       // Second pose index
    float   dx;             // Relative x
    float   dy;             // Relative y
    float   dtheta;         // Relative theta
    float   confidence;     // Constraint weight
} Constraint;

/**
 * @brief Main SLAM optimizer state
 */
typedef struct {

    /* Pose and scan storage (simple array with shift-on-full) */

    Pose        pose_pool[MAX_POSES];
    PointCloud  scan_pool[MAX_POSES];


    // Total poses added (can exceed MAX_POSES)
    int         current_pose_count;
    
    // Current number of poses in buffer (0 to MAX_POSES)
    int         buffer_size;



    /* Constraint storage for rebuilding H/b */
    Constraint  constraints[MAX_CONSTRAINTS];
    int         num_constraints;


    /* Dense matrices for optimization (STATE_SIZE x STATE_SIZE) */

    // Information matrix (Hessian)
    float       H[STATE_SIZE][STATE_SIZE];

    // RHS vector
    float       b[STATE_SIZE];
    
    // State vector [x0,y0,θ0, x1,y1,θ1, ...]
    float       state[STATE_SIZE];



    // Optimization control
    uint8_t        matrices_initialized;
    uint8_t        optimization_requested;
    uint32_t    last_optimization_time;
    
} SLAMOptimizer;



// ----------------------------------------------------------------------------
//
//  Helper Functions
//
// ----------------------------------------------------------------------------

/**
 * @brief Normalize angle to [-pi, pi]
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
 * @param[in] p1 First pose
 * @param[in] p2 Second pose (relative to p1)
 * @param[out] result Output composed pose
 */
void compose_poses(
    const Pose  *p1,
    const Pose  *p2,
    Pose        *result);


/**
 * @brief Compute relative pose from p1 to p2
 * 
 * @param[in] p1 First pose
 * @param[in] p2 Second pose
 * @param[out] relative Output relative pose
 */
void relative_pose(
    const Pose  *p1,
    const Pose  *p2,
    Pose        *relative);


// -----------------------------------------------------------------------------
//
//  Error and Jacobian Functions
//
// -----------------------------------------------------------------------------

/**
 * @brief Evaluate pose-pose error
 * 
 * @param[in] x_i First pose
 * @param[in] x_j Second pose
 * @param[out] z_ij Observed relative pose [dx, dy, dtheta]
 * @param[out] error Output error vector [3]
 */
void evaluate_error_pose_pose(
    const Pose  *x_i,
    const Pose  *x_j,
    const float z_ij[3],
    float       error[3]);


/**
 * @brief Compute Jacobian of pose-pose error
 * 
 * @param[in] x_i First pose
 * @param[in] x_j Second pose
 * @param[out] A Output Jacobian w.r.t. x_i [3x3]
 * @param[out] B Output Jacobian w.r.t. x_j [3x3]
 */
void compute_jacobian_pose_pose(
    const Pose  *x_i,
    const Pose  *x_j,
    float       A[3][3],
    float       B[3][3]);


// ----------------------------------------------------------------------------
//
//  ICP Integration Functions
//
// ----------------------------------------------------------------------------

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
 * @param initial_guess Initial transformation applied before ICP
 * @param result ICP transformation result
 * @return Confidence value [0-1]
 */
float slam_compute_icp_confidence(
    const PointCloud    *scan1,
    const PointCloud    *scan2,
    const Pose          *initial_guess,
    const ICPResult     *result);

// ----------------------------------------------------------------------------
//
//  Core SLAM Functions
//
// ----------------------------------------------------------------------------

/**
 * @brief Initialize the SLAM optimizer
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 */
void slam_initialize(SLAMOptimizer *optimizer);




/**
 * @brief Add a new pose and associated scan to the SLAM system
 * 
 * Uses a circular buffer to maintain the last MAX_POSES (15) poses.
 * When the buffer is full, the oldest pose is automatically overwritten.
 * This creates a sliding window of recent poses for memory-efficient SLAM.
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param pose New pose to add
 * @param scan Point cloud data at this pose
 * @return 1 if successful
 */
uint8_t slam_add_pose(
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
 * @return 1 if loop closure was detected and added
 */
uint8_t slam_detect_loop_closure(
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
 * Pose IDs are logical indices in the circular buffer (0 = oldest, buffer_size-1 = newest).
 * Valid range is [0, buffer_size).
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param pose_id Logical index of pose to retrieve (0 = oldest in buffer)
 * @param out_pose Output pose structure
 * @return 1 if pose_id is valid
 */
uint8_t slam_get_pose(
    const SLAMOptimizer *optimizer,
    int                 pose_id,
    Pose                *out_pose);


/**
 * @brief Get the point cloud for a specific pose
 * 
 * Pose IDs are logical indices in the circular buffer (0 = oldest, buffer_size-1 = newest).
 * Valid range is [0, buffer_size).
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @param pose_id Logical index of pose (0 = oldest in buffer)
 * @param out_scan Output point cloud structure
 * @return 1 if pose_id is valid
 */
uint8_t slam_get_scan(
    const SLAMOptimizer *optimizer,
    int                 pose_id,
    PointCloud          *out_scan);


/**
 * @brief Get the number of poses currently in the buffer
 * 
 * @param optimizer Pointer to SLAMOptimizer structure
 * @return Number of poses (0 to MAX_POSES)
 */
static inline int slam_get_buffer_size(const SLAMOptimizer *optimizer)
{
    return optimizer->buffer_size;
}




#endif // __INC_GRAPHSLAM_H__
