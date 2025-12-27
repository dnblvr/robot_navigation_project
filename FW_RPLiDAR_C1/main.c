/**
 * @file    main.c
 * @brief   Main testing ground for the LiDAR mapping code.
 *
 * @details This file expands on the main entry point and function definitions
 *  from the Tachometer lab to include UART definitions for the upcoming
 *  RPLiDAR C1 LiDAR scanner.
 *
 *
 * Timer_A is used:
 *  - Timer A0: (was) used to generate PWM signals to drive the DC motors
 *  - Timer A1: Used as a task scheduler to:
 *      - generate periodic interrupts at a specified rate (10 Hz)
 *      - triggers under eight tasks to be run in the main-loop
 *          - with WaitForInterrupt() to wait until
 *          - and Set_All_Interrupts_1() to reaffirm all EUSCI UART
 *                  interrupts
 *  - Timer A3: used to record the amount of steps taken by the wheels
 *
 *
 * UART A0-3:
 *  - UART A0 used for receiving data to
 *  - UART A2 used to receive the consecutive 5-byte messages from the RPLiDAR
 *      C1
 *  - UART A3 (was) used for BLE communication
 * 
 * @note some files are not included in main.c. as of this moment, the
 *      necessary functions are in-place.
 *
 * @note this is a fork of the RPLiDAR C1 to improve its development
 *
 * @author  Gian Fajardo
 * @authors massive credit to Prof. Aaron Nanas for providing the files!
 *
 */


#include "inc/Project_Config.h"


#include <stdint.h>
#include <string.h>


#include "inc/Timer_A1_Tasks.h"


#include "msp.h"
//#include "inc/"
#include "inc/RPLiDAR_C1.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/GPIO.h"
//#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/Tachometer.h"
#include "inc/GPIO_Utilities.h"


#ifdef DEBUG_OUTPUT
#include "inc/Profiler.h"
#endif

//#include "arm_math.h"
#include "inc/matrices.h"
#include "inc/coordinate_transform.h"
#include "inc/ICP_2D.h"
#include "inc/graphslam.h"



// ----------------------------------------------------------------------------
//
//  GRAPHSLAM ALGORITHM VARIABLES
//
// ----------------------------------------------------------------------------


// Testing flags
// #define TEST_ICP_ONLY               1
#define TEST_ODOMETRY_CONSTRAINT    1
#define TEST_LOOP_CLOSURE           1
#define TEST_OPTIMIZATION           1



/**
 * @brief main SLAM structure
 */
SLAMOptimizer slam_optimizer;

/**
 * @brief boolean that indicates whether SLAM has been initialized
 */
uint8_t slam_initialized   = false;

/**
 * @brief Incremental pose change since last scan (from odometry)
 */
Pose incremental_pose = {0.0f, 0.0f, 0.0f, 0};

/**
 * @brief Global accumulated pose estimate (dead reckoning)
 */
Pose global_pose = {0.0f, 0.0f, 0.0f, 0};


/**
 * @brief Test SLAM components incrementally after processing RPLiDAR data
 * 
 * @param[in] local_cloud Pointer to the local scan in sensor frame (for SLAM)
 * @param[in] transformed_cloud Pointer to the transformed point cloud in
 *      global coordinates (for visualization)
 */
void Perform_SLAM(
    PointCloud* local_cloud,
    PointCloud* transformed_cloud)
{
    // counter variables
    // int i;
    static uint32_t scan_counter = 0;

    // debug character
    // char            debug_buffer[128];


#ifdef DEBUG_OUTPUT
    printf(
        "Scan %lu: %d points\n", scan_counter, local_cloud->num_pts);
#endif
    
    // Initialize SLAM on first run
    if (!slam_initialized) {


        slam_initialize(&slam_optimizer);
        slam_initialized = true;
        // BLE_UART_OutString("SLAM initialized\n");

    }
    
    // printf("SLAN_INIT: %d\n", slam_initialized);
    // printf("SLAN_BUF: %2d\n", slam_optimizer.buffer_size);


    
    // ------------------------------------------------------------------------
    // STEP 1: Run ICP between GLOBAL-FRAME point clouds (Stachniss method)
    // ------------------------------------------------------------------------
    
    // Key: Transform scans to global frame BEFORE running ICP
    // This way ICP finds the correction in a consistent reference frame
    Pose icp_correction = {0.0f, 0.0f, 0.0f, 0};
    float icp_confidence = 0.5f;
    bool have_icp_correction = false;
    
    if (slam_optimizer.buffer_size > 0) {
        PointCloud previous_scan_local, previous_scan_global;
        PointCloud current_scan_global;
        Pose previous_pose;
        int prev_id = slam_optimizer.buffer_size - 1;
        
        // Get previous scan and its pose
        if (    slam_get_scan(&slam_optimizer, prev_id, &previous_scan_local)
             && slam_get_pose(&slam_optimizer, prev_id, &previous_pose))
        {
            
            // Transform previous scan to global frame using its pose
            transform_point_cloud(&previous_scan_local, &previous_pose, &previous_scan_global);
            
            // Transform current scan to global frame using predicted pose
            // Predicted pose = previous_pose ⊕ odometry
            Pose predicted_current_pose;
            compose_poses(&incremental_pose, &previous_pose, &predicted_current_pose);
            transform_point_cloud(local_cloud, &predicted_current_pose, &current_scan_global);
            
            // Now run ICP between these two GLOBAL-frame clouds
            ICPResult icp_result;
            Pose zero_guess = {0.0f, 0.0f, 0.0f, 0};  // No guess needed - clouds already aligned
            
            slam_perform_icp(&previous_scan_global,
                           &current_scan_global,
                           &zero_guess,
                           &icp_result);
            
            // Compute confidence
            icp_confidence = slam_compute_icp_confidence(&previous_scan_global,
                                                        &current_scan_global,
                                                        &zero_guess,
                                                        &icp_result);
            
            // ICP result is the CORRECTION to apply to the predicted pose
            icp_correction.x = icp_result.dx;
            icp_correction.y = icp_result.dy;
            icp_correction.theta = icp_result.dtheta;
            have_icp_correction = true;
            
#ifdef DEBUG_OUTPUT
            float correction_mag = sqrtf(icp_correction.x*icp_correction.x + 
                                        icp_correction.y*icp_correction.y);
            if (correction_mag > 5.0f || fabsf(icp_correction.theta) > 0.02f) {
                printf("  ICP correction: dx=%.1f dy=%.1f dθ=%.3f (%.1fmm) conf=%.2f\n",
                       icp_correction.x, icp_correction.y, icp_correction.theta,
                       correction_mag, icp_confidence);
            }
#endif
        }
    }
    
    // ------------------------------------------------------------------------
    // STEP 2: Update global_pose using ODOMETRY (state initialization)
    // This gives us an initial estimate that may differ from ICP observation
    // The optimizer will adjust it to balance odometry and ICP constraints
    // ------------------------------------------------------------------------
    
    compose_poses(&incremental_pose, &global_pose, &global_pose);
    global_pose.timestamp = incremental_pose.timestamp;
    
    // ------------------------------------------------------------------------
    // STEP 2: Add current pose (with ICP-refined global position) to SLAM
    // ------------------------------------------------------------------------
    
    slam_add_pose(&slam_optimizer,
                  &global_pose,
                  local_cloud);

    #ifdef DEBUG_OUTPUT
//    printf("Added pose to buffer. New buffer_size: %d\n",
//           slam_optimizer.buffer_size);
    #endif

    // ------------------------------------------------------------------------
    //
    //  TEST 1: ICP ONLY (standalone ICP test - results not used elsewhere)
    //
    // ------------------------------------------------------------------------

#ifdef TEST_ICP_ONLY
    
    // Only run ICP if we have at least 2 scans in buffer
    if (    slam_optimizer.buffer_size >= 2
         && local_cloud->num_pts > 10)
    {
        
        ICPResult icp_result;
        Pose initial_guess = {0.0f, 0.0f, 0.0f, 0};  // Identity transform
        float confidence;
        PointCloud previous_scan;
        

        // Get the second-to-last scan from the circular buffer
        if (slam_get_scan(&slam_optimizer,
                          slam_optimizer.buffer_size - 2,
                          &previous_scan))
        {
            
            // Run ICP between previous and current LOCAL scan
            slam_perform_icp(
                    &previous_scan,
                    local_cloud,
                    &initial_guess,
                    &icp_result);
            
            // Compute confidence
            confidence = slam_compute_icp_confidence(
                                &previous_scan,
                                local_cloud,
                                &initial_guess,
                                &icp_result);
            icp_result.confidence = confidence;
        

        // Send results via Bluetooth/Serial

    #ifdef DEBUG_OUTPUT
        // sprintf(
        //         debug_buffer,
        printf(
                "ICP: dx=%.3f dy=%.3f dtheta=%.3f conf=%.2f\n",
                icp_result.dx, icp_result.dy, icp_result.dtheta, confidence);
        // BLE_UART_OutString(debug_buffer);

    #endif
        
            // Visual feedback
            if (confidence > 0.5f) {
                LED2_Output(RGB_LED_GREEN);  // Good match
            } else {
                LED2_Output(RGB_LED_RED);    // Poor match
            }
            Clock_Delay1ms(40);
        }
    }
    
#endif // #ifdef TEST_ICP_ONLY


    // ------------------------------------------------------------------------
    //
    //  TEST 3: ODOMETRY CONSTRAINTS
    //
    // ------------------------------------------------------------------------

#ifdef TEST_ODOMETRY_CONSTRAINT

    if (slam_optimizer.buffer_size >= 2) {
        
        int prev_id = slam_optimizer.buffer_size - 2;
        int curr_id = slam_optimizer.buffer_size - 1;
        
        // Measurement = Odometry + ICP correction (Stachniss method)
        // ICP was run on global-frame clouds, so correction is in global frame
        Pose measurement;
        
        if (have_icp_correction && icp_confidence > 0.2f) {
            // Apply ICP correction to odometry measurement
            compose_poses(&icp_correction, &incremental_pose, &measurement);
            
#ifdef DEBUG_OUTPUT
            printf("  Measurement: odom=(%.1f,%.1f,%.3f) + ICP_corr=(%.1f,%.1f,%.3f) = final=(%.1f,%.1f,%.3f)\n",
                   incremental_pose.x, incremental_pose.y, incremental_pose.theta,
                   icp_correction.x, icp_correction.y, icp_correction.theta,
                   measurement.x, measurement.y, measurement.theta);
#endif
        } else {
            // Low confidence or no ICP - use odometry only
            measurement = incremental_pose;
            
#ifdef DEBUG_OUTPUT
            if (have_icp_correction) {
                printf("  Measurement: odom only (ICP conf too low: %.2f)\n", icp_confidence);
            }
#endif
        }
        
        float base_confidence = icp_confidence;
        
        // Compute magnitude of the measurement
        float measurement_magnitude = sqrtf(
                    measurement.x * measurement.x
                 +  measurement.y * measurement.y);
        
        // For small motions (likely noise), use very low confidence
        #define MIN_CONSTRAINT_MOTION_MM 15.0f
        #define MIN_CONFIDENCE_FOR_SMALL_MOTION 0.01f
        
        float effective_confidence = base_confidence;
        if (measurement_magnitude < MIN_CONSTRAINT_MOTION_MM && 
            fabsf(measurement.theta) < 0.05f) {
            effective_confidence = MIN_CONFIDENCE_FOR_SMALL_MOTION;
        }
        
        // Add constraint using ICP-corrected odometry
        slam_add_odometry_constraint(
                &slam_optimizer,
                prev_id,
                curr_id,
                measurement.x,
                measurement.y,
                measurement.theta,
                effective_confidence);
        
#ifdef DEBUG_OUTPUT
        if (effective_confidence < base_confidence) {
            printf("Constraint %d->%d: meas=(%.2f,%.2f,%.3f) mag=%.1f conf=%.2f->%.2f [LOW CONF]\n",
                   prev_id, curr_id,
                   measurement.x, measurement.y, measurement.theta,
                   measurement_magnitude, base_confidence, effective_confidence);
        } else {
            printf("Constraint %d->%d: meas=(%.2f,%.2f,%.3f) mag=%.1f conf=%.2f [ADDED]\n",
                   prev_id, curr_id,
                   measurement.x, measurement.y, measurement.theta,
                   measurement_magnitude, effective_confidence);
        }
#endif
    }
    
#endif // #ifdef TEST_ODOMETRY_CONSTRAINT


    // ------------------------------------------------------------------------
    //
    //  TEST 4: LOOP CLOSURE DETECTION
    //
    // ------------------------------------------------------------------------

#ifdef TEST_LOOP_CLOSURE

    if (slam_optimizer.buffer_size > MIN_TEMPORAL_GAP + 1) {
        

    #ifdef PROCESSING4_OUTPUT
//        printf("CLEAR\n");
    #endif


        uint8_t loop_detected = slam_detect_loop_closure(
                &slam_optimizer,
                slam_optimizer.buffer_size - 1);


    #ifdef DEBUG_OUTPUT
        printf("\n*** LOOP CLOSURE DETECTION ***");
    #endif
        
        if (loop_detected) {

    #ifdef DEBUG_OUTPUT
            printf(" --> *** LOOP CLOSURE DETECTED! ***\n\n");
    #endif

            LED2_Output(RGB_LED_BLUE);
            Clock_Delay1ms(40);

        } else {

            printf("\n\n");

        }

    } // if (slam_optimizer.buffer_size > MIN_TEMPORAL_GAP + 1) {
    
#endif // #ifdef TEST_LOOP_CLOSURE


    // ------------------------------------------------------------------------
    //
    //  TEST 5: OPTIMIZATION
    //
    // ------------------------------------------------------------------------

#ifdef TEST_OPTIMIZATION

    if (     slam_optimizer.buffer_size > 0
         && (slam_optimizer.buffer_size % OPTIMIZE_INTERVAL == 0))
    {
        
#ifdef DEBUG_OUTPUT
        printf("\nOptimizing graph...\n");
#endif

        LED2_Output(RGB_LED_PINK);
        

        slam_optimize_gauss_newton(&slam_optimizer, MAX_GAUSS_NEWTON_ITERS);
        

#ifdef PROCESSING4_OUTPUT
        // Clear visualization before sending complete optimized map
        printf("CLEAR\n");
#endif

        // Retrieve and transform all optimized poses and their point clouds
        int pose_idx;
        Pose optimized_pose;
        PointCloud original_scan, transformed_scan;
        
        // Store last two transformed clouds for alignment testing
        static PointCloud last_two_clouds[2];
        static int clouds_stored = 0;


        // corrects all poses in the buffer and their scans
        for (   pose_idx = 0;
                pose_idx < slam_optimizer.buffer_size;
                pose_idx++)
        {
            
            // Get optimized pose
            if ( !slam_get_pose(&slam_optimizer,
                                pose_idx,
                                &optimized_pose) ) {
                continue;  // Skip if pose retrieval fails
            }
            
            // Get original point cloud for this pose; skip if scan retrieval fails
            if ( !slam_get_scan(&slam_optimizer,
                                pose_idx,
                                &original_scan) ) {
                continue; 
            }
            
            // Transform point cloud to new optimized global frame
            transform_point_cloud(&original_scan,
                                  &optimized_pose,
                                  &transformed_scan);
            
            // Store last two transformed clouds for alignment testing
            if (pose_idx == slam_optimizer.buffer_size - 2 && slam_optimizer.buffer_size >= 2) {
                last_two_clouds[0] = transformed_scan;
                clouds_stored = 1;
            } else if (pose_idx == slam_optimizer.buffer_size - 1 && slam_optimizer.buffer_size >= 2) {
                last_two_clouds[1] = transformed_scan;
                clouds_stored = 2;
            }
            
#ifdef DEBUG_OUTPUT
            // printf("Pose[%2d] optimized: x=%.2f y=%.2f theta=%.3f | %d points transformed\n",
            //        pose_idx,
            //        optimized_pose.x, optimized_pose.y, optimized_pose.theta,
            //        transformed_scan.num_pts);
#endif


#ifdef PROCESSING4_OUTPUT

            int k;

            // Print all optimized poses with correct pose data
            printf("POSE,%5.2f,%5.2f,%5.2f\n",
                    optimized_pose.x,
                    optimized_pose.y,
                    optimized_pose.theta);

            printf("SCAN_START\n");

            for (k = 0; k < transformed_scan.num_pts; k++)
            {
                printf("P,%5.2f,%5.2f\n",
                        transformed_scan.points[k].x,
                        transformed_scan.points[k].y);
            }

            printf("SCAN_END\n");
#endif
        }
        
        // ------------------------------------------------------------------------
        // TEST: Run ICP on last two transformed point clouds to verify alignment
        // ------------------------------------------------------------------------
#ifdef DEBUG_OUTPUT
        if (clouds_stored == 2 && slam_optimizer.buffer_size >= 2) {
            
            float R[4];  // 2x2 rotation matrix
            float t[2];  // translation vector
            Pose zero_guess = {0.0f, 0.0f, 0.0f, 0};
            
            printf("\n=== ALIGNMENT TEST: ICP on last two transformed clouds ===\n");
            printf("Cloud[%d]: %d points\n", slam_optimizer.buffer_size - 2, last_two_clouds[0].num_pts);
            printf("Cloud[%d]: %d points\n", slam_optimizer.buffer_size - 1, last_two_clouds[1].num_pts);
            
            // Run ICP between the two already-transformed clouds
            // If poses are correct, these should already be aligned (small correction needed)
            ICP_2d(last_two_clouds[0].points, last_two_clouds[0].num_pts,
                   last_two_clouds[1].points, last_two_clouds[1].num_pts,
                   25,      // max iterations
                   0.01f,   // tolerance
                   R, t);
            
            // Convert rotation matrix to angle
            float theta_correction = atan2f(R[2], R[0]);  // atan2(R[1][0], R[0][0])
            float translation_mag = sqrtf(t[0]*t[0] + t[1]*t[1]);
            
            printf("ICP Correction needed: dx=%.2f mm, dy=%.2f mm, dtheta=%.3f rad (%.1f deg)\n",
                   t[0], t[1], theta_correction, theta_correction * 57.2958f);
            printf("Translation magnitude: %.2f mm\n", translation_mag);
            
            // Compute mean correspondence error after ICP
            int i, j;
            float total_error = 0.0f;
            int match_count = 0;
            PointCloud transformed_cloud0;
            
            // Apply ICP correction to first cloud
            Pose icp_correction;
            icp_correction.x = t[0];
            icp_correction.y = t[1];
            icp_correction.theta = theta_correction;
            transform_point_cloud(&last_two_clouds[0], &icp_correction, &transformed_cloud0);
            
            // Find mean nearest-neighbor distance
            for (i = 0; i < transformed_cloud0.num_pts; i++) {
                float min_dist = FLT_MAX;
                for (j = 0; j < last_two_clouds[1].num_pts; j++) {
                    float dx = transformed_cloud0.points[i].x - last_two_clouds[1].points[j].x;
                    float dy = transformed_cloud0.points[i].y - last_two_clouds[1].points[j].y;
                    float dist = sqrtf(dx*dx + dy*dy);
                    if (dist < min_dist) min_dist = dist;
                }
                if (min_dist < 200.0f) {  // Only count reasonable matches
                    total_error += min_dist;
                    match_count++;
                }
            }
            
            float mean_error = (match_count > 0) ? (total_error / match_count) : -1.0f;
            printf("Mean correspondence error after ICP: %.2f mm (%d/%d matches)\n",
                   mean_error, match_count, transformed_cloud0.num_pts);
            
            // Alignment quality assessment
            if (translation_mag < 10.0f && fabsf(theta_correction) < 0.05f && mean_error < 20.0f) {
                printf("✓ GOOD ALIGNMENT: Clouds are well-aligned!\n");
            } else if (translation_mag < 50.0f && fabsf(theta_correction) < 0.2f && mean_error < 50.0f) {
                printf("⚠ MODERATE ALIGNMENT: Some drift present\n");
            } else {
                printf("✗ POOR ALIGNMENT: Significant misalignment detected!\n");
            }
            printf("=======================================================\n\n");
        }
        
#endif // #ifdef DEBUG_OUTPUT
        
#ifdef DEBUG_OUTPUT

        // Get current pose for debugging
        slam_get_current_pose(&slam_optimizer, &optimized_pose);

        printf("Current (most recent) pose: x=%.2f y=%.2f theta=%.3f\n",
               optimized_pose.x, optimized_pose.y, optimized_pose.theta);
#endif
        
        LED2_Output(RGB_LED_GREEN);
    }
    
#endif // #ifdef TEST_OPTIMIZATION

    
    // Increment scan counter
    scan_counter++;

}



// ----------------------------------------------------------------------------
//
//  HIGHER-LEVEL RPLiDAR VARIABLES
//
// ----------------------------------------------------------------------------

/**
 * @brief declaration of an `C1_States` struct instance
 * @note this is originally declared in the `RPLiDAR_C1.c` source file
 */
extern C1_States cfg;


/**
 * @brief Angle_Filter function that verifies +/- 60 degrees to the front
 */
uint8_t Front_Scan(uint32_t data) {

#define FS_LIM_1    ( 60 << SHIFT_FACTOR)
#define FS_LIM_2    (300 << SHIFT_FACTOR)

    return (data < FS_LIM_1) || (data > FS_LIM_2);
}



// ----------------------------------------------------------------------------
//
//  TACHOMETER FUNCTIONS
//
// ----------------------------------------------------------------------------


// Initialize length of the tachometer buffers
#define TACHOMETER_BUFFER_LEN         10

// Number of left wheel steps measured by the tachometer
int32_t Left_Steps                  = 0;

// Number of right wheel steps measured by the tachometer
int32_t Right_Steps                 = 0;


// Number of left and right wheel distances in millimeters
float left_steps_mm, right_steps_mm;

// Store tachometer period of the left wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Left[TACHOMETER_BUFFER_LEN];

// Store tachometer period of the right wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Right[TACHOMETER_BUFFER_LEN];

// Direction of the left wheel's rotation
enum Tachometer_Direction Left_Direction;

// Direction of the right wheel's rotation
enum Tachometer_Direction Right_Direction;

// Buffer index
int buffer_idx = 0;

#define POSE_ARRAY_SIZE 100

typedef struct {
    Pose        all_poses[POSE_ARRAY_SIZE];
    uint32_t    num_elements;
} Accumulated_Poses;


Accumulated_Poses   poses;
Accumulated_Poses*  pose_ptr = &poses;


/**
 * @brief resets the Accumulated_Poses structure
 * 
 * @param pose_ptr 
 */
void Reset_Pose_Accumulator(Accumulated_Poses* pose_ptr) {

    // reset the accumulator
    memset(pose_ptr->all_poses, 0, sizeof(Pose) * POSE_ARRAY_SIZE);

    // reset the internal counter
    pose_ptr->num_elements  = 0;
}

/**
 * @brief Accumulate all intermediate poses into single composite transformation
 * 
 * @param pose_ptr  Pointer to pose accumulator buffer
 * @param output    Resulting composite pose change
 * 
 * @details Uses sequential 3x3 transformation matrix multiplication (instead
 *      of the funny way of adding poses together) to properly compose all
 *      incremental pose changes, accounting for rotation effects on subsequent
 *      translations.
 */
void Accumulate_Poses(
        Accumulated_Poses*  pose_ptr,
        Pose*               output)
{

    // if the accumulator is empty, return with zero pose
    if (pose_ptr->num_elements == 0) {
        output->x           = 0.0f;
        output->y           = 0.0f;
        output->theta       = 0.0f;
        output->timestamp   = 0;
        return;
    }

    int i, row, col;

    // Start with identity transformation
    float T_composite[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    float T_temp[3][3];
    float T_pose[3][3];

    // Multiply all intermediate pose transformations sequentially
    for (i = 0; i < pose_ptr->num_elements; i++) {

        Pose *p = &pose_ptr->all_poses[i];

        // Convert pose to 3x3 transformation matrix
        Make_Transformation_Matrix_Pose(p, T_pose);

        // T_composite = T_composite × T_pose
        matrix_multiply_3x3(T_composite, T_pose, T_temp);

        // Copy result back to T_composite
        for (row = 0; row < 3; row++) {
            for (col = 0; col < 3; col++) {
                T_composite[row][col] = T_temp[row][col];
            }
        }
    }

    // Extract final pose from composite transformation matrix
    output->x       = T_composite[0][2];      // Translation x
    output->y       = T_composite[1][2];      // Translation y

    // Extract rotation angle from rotation matrix
    output->theta   = atan2(T_composite[1][0], T_composite[0][0]);

    // Use timestamp from last pose in accumulator with better error handling
    if (pose_ptr->num_elements > 0) {
        output->timestamp \
            = pose_ptr->all_poses[pose_ptr->num_elements - 1].timestamp;

#ifdef DEBUG_OUTPUT
        // Calculate time span for debugging/analysis
//        uint32_t dt = output->timestamp - pose_ptr->all_poses[0].timestamp;

        // Optional: Log time span and sample count for performance analysis
        if (pose_ptr->num_elements > 1) {
//            printf("Pose composite: %lu samples over %lu ticks\n",
//                   pose_ptr->num_elements, dt);
        }
#endif
    } else {
        output->timestamp = 0;  // Fallback for empty accumulator
    }
}

/**
 * @brief Calculate robot pose from wheel encoder data and add to accumulator
 *
 * @param local_counter   Loop iteration counter
 * @param Left_Steps      Left wheel encoder count
 * @param Right_Steps     Right wheel encoder count
 * @param pose_accumulator Pointer to pose accumulator
 * @param timestamp       Current system tick counter for timestamping
 *
 * @details I am following the yaw orientation convention:
 *              if Left_Steps > Right_Steps, (+) yaw
 *              if Left_Steps < Right_Steps, (-) yaw
 *          450 mm required from both wheels to maintain simple rotation
 *          
 *          This function now stores incremental pose changes in the accumulator
 *          rather than maintaining a single running pose estimate.
 */
void Get_Pose(
        uint32_t    local_counter,
        int32_t     Left_Steps,
        int32_t     Right_Steps,
        Accumulated_Poses* pose_accumulator,
        uint32_t    timestamp)
{

    #define COUNTS_TO_DIST 0.6111111111

    // Distance between wheels in mm
    #define WHEEL_BASE_MM 141.0f  


    // variables for the previous values for later
    static float    prev_left_mm   = 0.0f,
                    prev_right_mm  = 0.0f;


    float   left_steps_mm, right_steps_mm,
            delta_left, delta_right,

            delta_s,        // travel distance along a curved direction
            delta_theta,

            x_local, y_local;

    // NOTE: Uses global incremental_pose declared at top of file
    // Do NOT declare a local one here - it shadows the global!


    // Convert encoder counts to distance in mm
    left_steps_mm   = COUNTS_TO_DIST * Left_Steps;
    right_steps_mm  = COUNTS_TO_DIST * Right_Steps;

    // Calculate change in wheel positions since last update
    delta_left  = left_steps_mm  - prev_left_mm;
    delta_right = right_steps_mm - prev_right_mm;

    // Save current values for next iteration
    prev_left_mm    = left_steps_mm;
    prev_right_mm   = right_steps_mm;


    /** ---------------------------------------------------------------------
     * Calculate linear and angular displacement
     */

    // Average linear displacement `s` between the two wheels
    delta_s     = (delta_left + delta_right) / 2.0f;

    // Angular change (in radians)
    delta_theta = (delta_right - delta_left) / WHEEL_BASE_MM;  


    /**
     * Calculate displacement in local (robot) frame; based on arc motion 
     *  formula `s = r*theta`. 
     */

    // Option 1: Moving straight; avoid division by zero
    if (fabs(delta_theta) < 1e-6) {

        x_local = delta_s;
        y_local = 0.0f;


    // Option 2: arc motion; uses the formula
    } else {
        float radius;

        radius  = delta_s / delta_theta;
        x_local = radius * sin(delta_theta);
        y_local = radius * (1.0f - cos(delta_theta));


    }


    // Store incremental pose change (in local robot frame)
    incremental_pose.x      = x_local;
    incremental_pose.y      = y_local;
    incremental_pose.theta  = delta_theta;
    incremental_pose.timestamp = timestamp;

    // Normalize angle to [-pi, pi]
    incremental_pose.theta  = normalize_angle(incremental_pose.theta);

    // Add to accumulator
    if (pose_accumulator->num_elements < POSE_ARRAY_SIZE) {
        pose_accumulator->all_poses[pose_accumulator->num_elements] \
                = incremental_pose;
        pose_accumulator->num_elements++;
    }

    // Debug output
#ifdef DEBUG_OUTPUT
//    printf("trv %3u: L=%7.2f R=%7.2f | x=%7.2f y=%7.2f θ=%6.3f\n",
//            local_counter,
//            left_steps_mm, right_steps_mm,
//            pose_at_time->x, pose_at_time->y, pose_at_time->theta);
#endif

}


// ----------------------------------------------------------------------------
//
//  CONFIGURATION and MAIN LOOP
//
// ----------------------------------------------------------------------------

void main(void) {

    // disable watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;


    // Initialize unused ports to reduce power consumption
    Init_Unused_Ports();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();


    // Initialize the built-in red LED and the RGB LEDs
    LED2_Init();


    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();


    // use Timer_A1_Interrupt to get the odometry measurements
    // resulting output will be measured in increments of 10 ticks per second
    Timer_A1_Interrupt_Init(&Task_Selector, TIMER_A1_CCR0_VALUE);


    // Initialize the tachometers as a function of Timer A3
    Tachometer_Init();
    Reset_Pose_Accumulator(pose_ptr);


    // Initialize the DC motors
//    Motor_Init();


    Set_All_Interrupts_1();


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();

    
    /** ----------------------------------------------------------------------
     * Initialize RPLiDAR C1 configuration struct instance 
     */
    Initialize_RPLiDAR_C1(&cfg);

#ifdef DEBUG_OUTPUT
    // printf("C1 initialized\n\n");
#endif

    Set_All_Interrupts_1();


#ifdef DEBUG_OUTPUT
    printf("test scan -------\n\n");
#endif


    /** ----------------------------------------------------------------------
     * Initialize BlueTooth module instance
     */

    // BLE_UART_Init(BLE_UART_Data_Buffer);
    // Clock_Delay1ms(1000);
    // BLE_UART_Reset();
    // BLE_UART_OutString("BLE UART Active\r\n");
    // Clock_Delay1ms(1000);


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();


    EUSCI_A0->IE   |=  0x0001;  // printf()
    EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1
    EUSCI_A3->IE   |=  0x0001;  // BLE UART Friend

//    Set_All_Interrupts_1();

#ifdef DEBUG_OUTPUT
//    printf("pressed\n");
#endif

    TIMER_A1->CCTL[0]  |=  0x0010;


    /** ----------------------------------------------------------------------
     * main loop
     */

    uint32_t sequence_counter   = 0;
    uint32_t local_counter      = 0;

    while (1)
    {
        /**
         * @note   blocks execution until *any* interrupt occurs
         */
        WaitForInterrupt();

        /**
         * @note receiving information over UART disables RXIE for whatever reason.
         *       Enabling RXIE after W.F.I. ensures that RXIE is *always* ON.
         */
        Set_All_Interrupts_1();

        // printf("%04X\n", EUSCI_A3->IE);


#ifdef TASK_0_FLAG
        /**
         * @note TASK 0: receive BLE motor commands at 10 times a second
         */
        if (task_flag & TASK_0_FLAG) {
            task_flag  &= ~TASK_0_FLAG; // clear the flag

            Timer_A1_Ignore();
            Process_BLE_UART_Data(BLE_UART_Data_Buffer);
            Timer_A1_Acknowledge();
        }
#endif


#ifdef TASK_1_FLAG
        /**
         * @note TASK 1: increment local_counter
         */
        if (task_flag & TASK_1_FLAG) {
            task_flag  &= ~TASK_1_FLAG;
        }
#endif


#ifdef TASK_2_FLAG
        /**
         * @note TASK 2: Get the measurements made by the tachometers
         *       Accumulate incremental pose changes between LiDAR scans
         */
        if (task_flag & TASK_2_FLAG) {
            task_flag  &= ~TASK_2_FLAG;

            // Get current tachometer readings
            Tachometer_Get(&Tachometer_Buffer_Left[buffer_idx],
                           &Left_Direction,
                           &Left_Steps,
                    
                           &Tachometer_Buffer_Right[buffer_idx],
                           &Right_Direction,
                           &Right_Steps);

            // Add incremental pose to accumulator
            Get_Pose(local_counter,
                     Left_Steps, Right_Steps,
                     pose_ptr,
                     tick_counter);
        }
#endif


#ifdef TASK_3_FLAG
        /**
         * @note TASK 3: set flag to collect RPLiDAR C1 data
         */
        if (task_flag & TASK_3_FLAG) {
            task_flag  &= ~TASK_3_FLAG;

            // data collection profiling! --------------------------------
            // Start_Timer_1ms();

            Start_Record(NULL);
//            Start_Record(Front_Scan);

        }
#endif


#ifdef TASK_4_FLAG
        /**
         * @note TASK 4: processes recorded data and runs ICP
         */
        if (task_flag & TASK_4_FLAG)
        {
            task_flag  &= ~TASK_4_FLAG;

    #ifdef DEBUG_OUTPUT
            // printf(cfg.current_state == IDLING      ? "IDLE\n" : "");
            // printf(cfg.current_state == READY       ? "READY\n" : "");
            // printf(cfg.current_state == RECORDING   ? "RECORDING\n" : "");
            // printf(cfg.current_state == PROCESSING  ? "PROCESSING\n" : "");
    #endif

            // Timer_A1_Ignore();  

            LED2_Output(RGB_LED_BLUE);


            // profiling start! -----------------------------------------------

    #ifdef DEBUG_OUTPUT
//            Start_Timer_1ms();
    #endif
            /**
             * @brief If in PROCESSING state, process the recorded data
             */
            if (cfg.current_state == PROCESSING)
            {

                /**
                 * @brief If data is aligned, process it and run SLAM
                 * 
                 * @note the expected Processing Sketch format for pose + 
                 *  pointcloud info is:
                 *   POSE,x,y,theta
                 *   SCAN_START
                 *   P,x,y
                 *   P,x,y
                 *   ...
                 *   SCAN_END
                 */

                // counter variables
                uint32_t k;

                float T_matrix[3][3] = {0};
                PointCloud local_cloud;  // Local scan in sensor frame
                PointCloud transformed_cloud;


                // visually indicate if aligned
                LED2_Output(RGB_LED_WHITE);


                /**
                 * Collate all intermediate poses into single composite
                 *      transformation, and then reset the accumulator.
                 */
    #ifdef DEBUG_OUTPUT
//                uint32_t num_samples = pose_ptr->num_elements;  // Save before reset
    #endif
                // Get incremental motion since last scan
                Accumulate_Poses(pose_ptr, &incremental_pose);
                incremental_pose.timestamp = tick_counter;

                // NOTE: We do NOT update global_pose here anymore!
                // Perform_SLAM() will refine incremental_pose with ICP and update global_pose
                // This ensures state and constraints are consistent
                    
                Reset_Pose_Accumulator(pose_ptr);
                    
    #ifdef DEBUG_OUTPUT
//                printf("Composite pose from %lu samples: "
//                       "x=%.2f y=%.2f theta=%.3f\n",
//                       num_samples,
//                       incremental_pose.x, incremental_pose.y, incremental_pose.theta);
    #endif
                    
                /**
                 * Process RPLiDAR C1 data into buffer `local_cloud`, and
                 *  transform it with the global pose for visualization.
                 */
                Process_RPLiDAR_Data(&local_cloud);

                // Use global_pose for visualization transform
                Make_Transformation_Matrix_Pose(&global_pose,
                                                T_matrix);


                printf("POSE,%5.2f,%5.2f,%5.2f\n",
                       global_pose.x,
                       global_pose.y,
                       global_pose.theta);

                printf("SCAN_START\n");
                    

                // Transform local point cloud
                transformed_cloud.num_pts    = 0;
                for (k = 0; k < local_cloud.num_pts; k++)
                {
                    float local_x = local_cloud.points[k].x;
                    float local_y = local_cloud.points[k].y;

                    transformed_cloud.points[k].x \
                            =    T_matrix[0][0]*local_x \
                               + T_matrix[0][1]*local_y \
                               + T_matrix[0][2];
                    transformed_cloud.points[k].y \
                            =    T_matrix[1][0]*local_x \
                               + T_matrix[1][1]*local_y \
                               + T_matrix[1][2];

    #ifdef PROCESSING4_OUTPUT
                    printf("P,%5.2f,%5.2f\n",
                           transformed_cloud.points[k].x,
                           transformed_cloud.points[k].y);
    #endif
                }

                transformed_cloud.num_pts = k;

    #ifdef PROCESSING4_OUTPUT
                printf("SCAN_END\n");
                // printf("SAVE\n");
    #endif

                    
    #ifdef DEBUG_OUTPUT

        #define DATA_PT 5

                // printf("Transformed cloud: %d points"
                //        " | Sample %3i: local(%7.2f, %7.2f)"
                //        " -> global(%7.2f, %7.2f)\n",
                //         transformed_cloud.num_pts,
                //         DATA_PT,
                //         output[DATA_PT][0],
                //         output[DATA_PT][1],
                //         transformed_cloud.points[DATA_PT].x,
                //         transformed_cloud.points[DATA_PT].y);
    #endif

//                Perform_SLAM(&local_cloud, &transformed_cloud);

                LED2_Output(RGB_LED_OFF);

                // regardless if non-/aligned, reset the state to IDLE
                cfg.current_state   = IDLING;

            }


            // profiling end! -------------------------------------------------

    #ifdef DEBUG_OUTPUT
//            printf("\tdata processing ");
//            Stop_Timer();
    #endif

            Timer_A1_Acknowledge();

        }
#endif // #ifdef TASK_4_FLAG



#ifdef TASK_5_FLAG
        /**
         * @note TASK 5: blank
         */
        if (task_flag & TASK_5_FLAG) {
            task_flag  &= ~TASK_5_FLAG;
        }
#endif // #ifdef TASK_5_FLAG



#ifdef TASK_6_FLAG
        /**
         * @note: TASK 6: print persistently
         */
        if (task_flag & TASK_6_FLAG) {
            task_flag  &= ~TASK_6_FLAG;

    #ifdef DEBUG_OUTPUT
//            printf("%5d %5d\n", local_counter, sequence_counter);
    #endif

            sequence_counter++;
        }
#endif // #ifdef TASK_6_FLAG



#ifdef TASK_7_FLAG
        /**
         * @note TASK 7: increment counter
         */
        if (task_flag & TASK_7_FLAG) {
            task_flag  &= ~TASK_7_FLAG;

    #ifdef DEBUG_OUTPUT
            printf("%5d\n\n", local_counter);
            printf("\n\n");
    #endif
            local_counter++;

            sequence_counter = 0;
        }
#endif // #ifdef TASK_7_FLAG

    }

} // void main(void) end ------------------------------------------------------

