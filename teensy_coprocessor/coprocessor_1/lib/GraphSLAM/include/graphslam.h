/**
 * @file graphslam.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 */

#ifndef __GRAPHSLAM_H__
#define __GRAPHSLAM_H__

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <ICP_2D.h>
#include <cholesky_decomposition.h>

#ifdef __cplusplus
extern "C" {
#endif


// ----------------------------------------------------------------------------
// 
//  CONSTANTS
// 
// ----------------------------------------------------------------------------

/**
 * @brief Math constant for pi (if not defined by the system)
 */
#ifndef M_PI_F
  #define M_PI_F    3.14159265358979323846f
#endif


/**
 * @brief Memory configuration for MSP432
 */
#define MAX_POSES               30              // Sliding window size
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
 * @brief weight amount to add to diagonal of H for the first pose to fix it as
 *  an anchor and prevent drift. 
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// ----------------------------------------------------------------------------
//
//  DATA STRUCTURES
//
// ----------------------------------------------------------------------------


/**
 * @brief Stored constraint for rebuilding H/b
 * 
 * @param pose1_id First pose index
 * @param pose2_id Second pose index
 * @param dx Relative x
 * @param dy Relative y
 * @param dtheta Relative theta
 * @param confidence Constraint weight
 */
typedef struct {
    int     pose1_id;
    int     pose2_id;
    float   dx;
    float   dy;
    float   dtheta;
    float   confidence;
} Constraint;

/**
 * @brief Main SLAM optimizer state
 * 
 * @param pose_pool Buffer for storing poses (sliding window)
 * @param scan_pool Buffer for storing scans (sliding window)
 * @param current_pose_count    Total poses added (can exceed `MAX_POSES`)
 * @param buffer_size   Current number of poses in buffer (0 to `MAX_POSES`)
 * 
 * @param constraints   Constraint storage for rebuilding H/b 
 * @param num_constraints Number of constraints stored
 * 
 * @param H     Information matrix (Hessian)
 * @param b     RHS vector
 * @param state State vector [x0,y0,θ0, x1,y1,θ1, ...]
 * 
 * @param matrices_initialized Flag to indicate if H/b/state buffers have been
 *  initialized
 * @param optimization_requested Flag to indicate if optimization is requested
 * @param last_optimization_time Timestamp of the last optimization
 */
typedef struct {

    Pose        pose_pool[MAX_POSES];
    PointCloud  scan_pool[MAX_POSES];

    int         current_pose_count;
    int         buffer_size;

    Constraint  constraints[MAX_CONSTRAINTS];
    int         num_constraints;

    float       H[STATE_SIZE * STATE_SIZE];
    float       b[STATE_SIZE];
    float       state[STATE_SIZE];

    uint8_t     matrices_initialized;
    uint8_t     optimization_requested;
    uint32_t    last_optimization_time;
    
} SLAMOptimizer;



// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Normalize angle to [-pi, pi]
 * 
 * @param[in] angle Input angle in radians
 * 
 * @return `float`
 * @retval Normalized angle
 */
float normalize_angle(float angle);


/**
 * @brief Compute Euclidean distance between two poses
 * 
 * @param[in] pose1 First pose
 * @param[in] pose2 Second pose
 * 
 * @return `float`
 * @return Distance in meters
 */
float pose_distance(
        const Pose *pose1,
        const Pose *pose2);


/**
 * @brief Transform a point cloud by a pose
 * 
 * @param[in] scan Input point cloud
 * @param[in] pose Transformation pose
 * @param[out] out_scan Output transformed point cloud
 */
void transform_point_cloud(
        const PointCloud   *scan,
        const Pose         *pose,
              PointCloud   *out_scan);


/**
 * @brief Compose two poses (`p1` + `p2`)
 * 
 * @param[in] p1 First pose
 * @param[in] p2 Second pose (relative to `p1`)
 * @param[out] result Output composed pose
 */
void compose_poses(
        const Pose  *p1,
        const Pose  *p2,
              Pose  *result);


/**
 * @brief Compute relative pose from `p1` to `p2`
 * 
 * @param[in] p1 First pose
 * @param[in] p2 Second pose
 * @param[out] relative Output relative pose
 */
void relative_pose(
        const Pose  *p1,
        const Pose  *p2,
              Pose  *relative);


// ----------------------------------------------------------------------------
//
//  ICP INTEGRATION FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Perform ICP alignment between two scans
 * 
 * @param[in] scan1 First point cloud
 * @param[in] scan2 Second point cloud
 * @param[in] initial_guess Initial transformation guess which is used to
 *  improve ICP convergence and avoid local minima. Typically gathered from a
 *  state estimator (e.g. inEKF) or odometry
 * 
 * @param[out] result Output ICP result.
 * 
 * @see `ICPResult` structure for details on the output fields. Important to
 *  note that the `ICPResult` fields `dx`, `dy`, `dtheta` represent the
 *  measurement `z_ij` for the pose-pose constraint in the SLAM graph. 
 */
void slam_perform_icp(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const Pose*         initial_guess,
        ICPResult*          result);


/**
 * @brief Iterative ICP wrapper; uses ICP_2D_i() for incremental convergence
 * 
 * @param[in] scan1 First point cloud
 * @param[in] scan2 Second point cloud
 * @param[in] initial_guess Expected relative pose to improve ICP convergence and avoid local minima
 * @param[out] result Output ICP result
 * 
 * @see `ICPResult` structure for details on the output fields
 */
void slam_perform_icp_i(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const Pose*         initial_guess,
        ICPResult*          result);


/**
 * @brief Compute confidence metric for ICP result
 * 
 * @param[in] scan1 First point cloud
 * @param[in] scan2 Second point cloud
 * @param[in] result ICP transformation result
 * 
 * @return `float`
 * 
 * @retval Confidence value [from 0-1]
 */
float slam_compute_icp_confidence(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const ICPResult*    result);


/**
 * @brief Compute confidence metric for ICP result;
 * 
 * @details Improves upon the previous version by retrieving cached
 *  correspondence distances performed from `ICP_2D_i()` to avoid redundant
 *  nearest neighbor search.
 * 
 * @return `float`
 * @retval Confidence value [from 0-1]
 */
float slam_compute_icp_confidence_i();


// -----------------------------------------------------------------------------
//
//  ERROR & JACOBIAN FUNCTIONS
//
// -----------------------------------------------------------------------------

/**
 * @brief Evaluate pose-pose error
 * 
 * @param[in] x_i First pose
 * @param[in] x_j Second pose
 * 
 * @param[out] z_ij Observed relative pose [`dx`, `dy`, `dtheta`]
 * @param[out] error Output error vector [3]
 */
void evaluate_error_pose_pose(
        const Pose* x_i,
        const Pose* x_j,
        const float z_ij[3],
              float error[3]);


/**
 * @brief Compute Jacobian of pose-pose error
 * 
 * @param[in] x_i First pose
 * @param[in] x_j Second pose
 * 
 * @param[out] A Output Jacobian w.r.t. `x_i` [3x3]
 * @param[out] B Output Jacobian w.r.t. `x_j` [3x3]
 */
void compute_jacobian_pose_pose(
        const Pose* x_i,
        const Pose* x_j,
        float       A[3][3],
        float       B[3][3]);

// ----------------------------------------------------------------------------
//
//  CORE SLAM FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief Initialize the SLAM optimizer
 * 
 * @param[in] optimizer Pointer to `SLAMOptimizer` structure
 * 
 * @return `uint8_t`
 * @retval `1` if initialization was successful
 */
uint8_t slam_initialize(SLAMOptimizer* optimizer);


/**
 * @brief Add a new pose and associated scan to the SLAM system
 * 
 *  Uses a circular buffer to maintain the last `MAX_POSES` poses.
 *  When the buffer is full, the oldest pose is automatically overwritten.
 *  This creates a sliding window of recent poses for memory-efficient SLAM.
 * 
 * @param[inout] optimizer Pointer to `SLAMOptimizer` structure
 * @param[in] pose New pose to add
 * @param[in] scan Point cloud data at this pose
 * @return `uint8_t`
 * @retval `1` if successful
 */
uint8_t slam_add_pose(
          SLAMOptimizer*    optimizer,
    const Pose*             pose,
    const PointCloud*       scan);


/**
 * @brief Add an odometry constraint between consecutive poses
 * 
 * Note: H/b are rebuilt in `slam_optimize_gauss_newton()` from all
 *  constraints 
 * 
 * @param[inout] optimizer Pointer to `SLAMOptimizer` structure
 * @param[in] pose1_id Index of first pose
 * @param[in] pose2_id Index of second pose
 * @param[in] dx Relative x displacement
 * @param[in] dy Relative y displacement
 * @param[in] dtheta Relative angle change
 * @param[in] confidence Measurement confidence (higher = more certain)
 */
void slam_add_odometry_constraint(
        SLAMOptimizer*  optimizer,
        int             pose1_id,
        int             pose2_id,
        float           dx,
        float           dy,
        float           dtheta,
        float           confidence);


/**
 * @brief Add an ICP-based constraint between poses
 * 
 * @param[inout] optimizer Pointer to `SLAMOptimizer` structure
 * @param[in] pose1_id Index of first pose
 * @param[in] pose2_id Index of second pose
 * @param[in] icp_result ICP alignment result
 * @param[in] base_confidence Base confidence value
 */
void slam_add_icp_constraint(
              SLAMOptimizer*    optimizer,
              int               pose1_id,
              int               pose2_id,
        const ICPResult*        icp_result,
              float             base_confidence);


/**
 * @brief Detect and add loop closure constraints
 * 
 * @param[inout] optimizer Pointer to `SLAMOptimizer` structure
 * @param[in] current_pose_id Current pose index to check for loop closures\
 * 
 * @return `uint8_t`
 * @retval `1` if loop closure was detected and added
 * @retval `0` if no loop closure was detected
 */
uint8_t slam_detect_loop_closure(
        SLAMOptimizer*  optimizer,
        int             current_pose_id);


/**
 * @brief Perform Gauss-Newton optimization on the pose graph
 * 
 * @param[inout] optimizer Pointer to `SLAMOptimizer` structure
 * @param[in] max_iterations Maximum optimization iterations
 */
void slam_optimize_gauss_newton(
        SLAMOptimizer*  optimizer,
        int             max_iterations);


/**
 * @brief Get the current (most recent) pose
 * 
 * @param[in] optimizer Pointer to `SLAMOptimizer` structure
 * @param[out] out_pose Output pose structure
 */
void slam_get_current_pose(
        const SLAMOptimizer* optimizer,
              Pose*          out_pose);


/**
 * @brief Get a specific pose by index
 * 
 * Pose IDs are logical indices in the circular buffer (0 = oldest,
 *  `buffer_size-1` = newest). Valid range is [0, `buffer_size`).
 * 
 * @param[in] optimizer Pointer to `SLAMOptimizer` structure
 * @param[in] pose_id Logical index of pose to retrieve (0 = oldest in buffer)
 * 
 * @param[out] out_pose Output pose structure
 * @return `uint8_t`
 * @retval `1` if `pose_id` is valid
 * @retval `0` if `pose_id` is invalid
 */
uint8_t slam_get_pose(
        const SLAMOptimizer*    optimizer,
              int               pose_id,
              Pose*             out_pose);


/**
 * @brief Get the point cloud for a specific pose
 * 
 * Pose IDs are logical indices in the circular buffer (0 = oldest,
 *  `buffer_size-1` = newest). Valid range is [0, `buffer_size`).
 * 
 * @param[in] optimizer Pointer to `SLAMOptimizer` structure
 * @param[in] pose_id Logical index of pose (0 = oldest in buffer)
 * 
 * @param[out] out_scan Output point cloud structure
 * @return `uint8_t`
 * @retval `1` if `pose_id` is valid
 * @retval `0` if `pose_id` is invalid
 */
uint8_t slam_get_scan(
        const SLAMOptimizer*    optimizer,
              int               pose_id,
              PointCloud*       out_scan);


/**
 * @brief Get the number of poses currently in the buffer
 * 
 * @param[in] optimizer Pointer to `SLAMOptimizer` structure
 * 
 * @return `int`
 * @retval Number of poses (0 to `MAX_POSES`)
 */
static inline int slam_get_buffer_size(
        const SLAMOptimizer*    optimizer)
{
    return optimizer->buffer_size;
}


















void slam_perform_icp_play(
        const PointCloud*   scan1,
        const PointCloud*   scan2,
        const Pose*         initial_guess,
              ICPResult*    result);


#ifdef __cplusplus
}
#endif


#endif // __GRAPHSLAM_H__
