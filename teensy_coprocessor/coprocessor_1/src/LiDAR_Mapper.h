/**
 * @file LiDAR_Mapper.h
 * @brief Teensy implementation of the LiDAR mapping module.
 *
 * @details This file mirrors the floating_point_impl.h pattern: the entire
 *  implementation lives here and main.cpp switches into it via a single
 *  preprocessor guard.  It exercises the new RPLiDAR_C1 library end-to-end on
 *  the Teensy without touching any other subsystems.
 *
 * @author Gian Fajardo
 */

#pragma once

#include <Arduino.h>
#include <RPLiDAR_C1.h>
#include <LPUART8.h>
#include <coordinate_transform.h>
#include <ICP_2D.h>
#include <graphslam.h>

#include "Timer_Tasks.h"

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


#ifdef DEBUG_OUTPUTS
    printf(
        "Scan %lu: %d points\n", scan_counter, local_cloud->num_pts);
#endif
    
    // Initialize SLAM on first run
    if (!slam_initialized) {


        slam_initialize(&slam_optimizer);
        slam_initialized = true;
        // BLE_UART_OutString("SLAM initialized\n");

    }
    
    
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
            
#ifdef DEBUG_OUTPUTS
            float correction_mag = sqrtf(   icp_correction.x*icp_correction.x
                                         +  icp_correction.y*icp_correction.y);

            if (    (correction_mag > 5.0f)
                 || (fabsf(icp_correction.theta) > 0.02f))
            {
                Serial.printf("  ICP correction: dx=%.1f"
                              " dy=%.1f dθ=%.3f (%.1fmm) conf=%.2f\n",
                              icp_correction.x, icp_correction.y, icp_correction.  theta,
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

    #ifdef DEBUG_OUTPUTS
//    Serial.printf("Added pose to buffer. New buffer_size: %d\n",
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

    #ifdef DEBUG_OUTPUTS
        // sprintf(
        //         debug_buffer,
        Serial.printf(
                "ICP: dx=%.3f dy=%.3f dtheta=%.3f conf=%.2f\n",
                icp_result.dx, icp_result.dy, icp_result.dtheta, confidence);
        // BLE_UART_OutString(debug_buffer);

    #endif
        
            // Visual feedback
            if (confidence > 0.5f) {
                // LED2_Output(RGB_LED_GREEN);  // Good match
            } else {
                // LED2_Output(RGB_LED_RED);    // Poor match
            }
            delay(40);
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
            
#ifdef DEBUG_OUTPUTS
            printf("  Measurement: odom=(%.1f,%.1f,%.3f) + ICP_corr=(%.1f,%.1f,%.3f) = final=(%.1f,%.1f,%.3f)\n",
                   incremental_pose.x, incremental_pose.y, incremental_pose.theta,
                   icp_correction.x, icp_correction.y, icp_correction.theta,
                   measurement.x, measurement.y, measurement.theta);
#endif
        } else {
            // Low confidence or no ICP - use odometry only
            measurement = incremental_pose;
            
#ifdef DEBUG_OUTPUTS
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
        
#ifdef DEBUG_OUTPUTS
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
        

    #ifdef PROCESSING_EXPECTED_OUTPUTS
//        printf("CLEAR\n");
    #endif


        uint8_t loop_detected = slam_detect_loop_closure(
                &slam_optimizer,
                slam_optimizer.buffer_size - 1);


    #ifdef DEBUG_OUTPUTS
        printf("\n*** LOOP CLOSURE DETECTION ***");
    #endif
        
        if (loop_detected) {

    #ifdef DEBUG_OUTPUTS
            printf(" --> *** LOOP CLOSURE DETECTED! ***\n\n");
    #endif

            // LED2_Output(RGB_LED_BLUE);
            delay(40);

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
        
#ifdef DEBUG_OUTPUTS
        printf("\nOptimizing graph...\n");
#endif

        // LED2_Output(RGB_LED_PINK);
        

        slam_optimize_gauss_newton(&slam_optimizer, MAX_GAUSS_NEWTON_ITERS);
        

#ifdef PROCESSING_EXPECTED_OUTPUTS
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
            if (    pose_idx == slam_optimizer.buffer_size - 2
                &&  slam_optimizer.buffer_size >= 2)
            {
                last_two_clouds[0] = transformed_scan;
                clouds_stored = 1;
            } else if (     pose_idx == slam_optimizer.buffer_size - 1
                        &&  slam_optimizer.buffer_size >= 2)
            {
                last_two_clouds[1] = transformed_scan;
                clouds_stored = 2;
            }
            
#ifdef DEBUG_OUTPUTS
            // printf("Pose[%2d] optimized: x=%.2f y=%.2f theta=%.3f | %d points transformed\n",
            //        pose_idx,
            //        optimized_pose.x, optimized_pose.y, optimized_pose.theta,
            //        transformed_scan.num_pts);
#endif


#ifdef PROCESSING_EXPECTED_OUTPUTS

            int k;

            // Print all optimized poses with correct pose data
            Serial.printf("POSE,%5.2f,%5.2f,%5.2f\n",
                          optimized_pose.x,
                          optimized_pose.y,
                          optimized_pose.theta);

            Serial.printf("SCAN_START\n");

            for (k = 0; k < transformed_scan.num_pts; k++)
            {
                Serial.printf("P,%5.2f,%5.2f\n",
                        transformed_scan.points[k].x,
                        transformed_scan.points[k].y);
            }

            printf("SCAN_END\n");
#endif
        }
        
        // ------------------------------------------------------------------------
        // TEST: Run ICP on last two transformed point clouds to verify alignment
        // ------------------------------------------------------------------------
#ifdef DEBUG_OUTPUTS
        if (clouds_stored == 2 && slam_optimizer.buffer_size >= 2) {
            
            float R[4];  // 2x2 rotation matrix
            float t[2];  // translation vector
            Pose zero_guess = {0.0f, 0.0f, 0.0f, 0};
            
            Serial.printf("\n=== ALIGNMENT TEST: ICP on last two transformed clouds ===\n");
            Serial.printf("Cloud[%d]: %d points\n", slam_optimizer.buffer_size - 2, last_two_clouds[0].num_pts);
            Serial.printf("Cloud[%d]: %d points\n", slam_optimizer.buffer_size - 1, last_two_clouds[1].num_pts);
            
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
            
            Serial.printf("ICP Correction needed: dx=%.2f mm, dy=%.2f mm, dtheta=%.3f rad (%.1f deg)\n",
                   t[0], t[1], theta_correction, theta_correction * 57.2958f);
            Serial.printf("Translation magnitude: %.2f mm\n", translation_mag);
            
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
                printf("GOOD ALIGNMENT: Clouds are well-aligned!\n");
            } else if (translation_mag < 50.0f && fabsf(theta_correction) < 0.2f && mean_error < 50.0f) {
                printf("MODERATE ALIGNMENT: Some drift present\n");
            } else {
                printf("POOR ALIGNMENT: Significant misalignment detected!\n");
            }
            printf("=======================================================\n\n");
        }
        
#endif // #ifdef DEBUG_OUTPUTS
        
#ifdef DEBUG_OUTPUTS

        // Get current pose for debugging
        slam_get_current_pose(&slam_optimizer, &optimized_pose);

        printf("Current (most recent) pose: x=%.2f y=%.2f theta=%.3f\n",
               optimized_pose.x, optimized_pose.y, optimized_pose.theta);
#endif
        
        // LED2_Output(RGB_LED_GREEN);
    }
    
#endif // #ifdef TEST_OPTIMIZATION

    
    // Increment scan counter
    scan_counter++;

}


// ----------------------------------------------------------------------------
//
//  Configuration
//
// ----------------------------------------------------------------------------

/**
 * @brief 
 */
#define MSP432_Serial  Serial5

/**
 * @brief 
 */
#define FLAG_MASK(n)       (1 << (n))

/**
 * @brief 
 */
#define ECHO_REQUEST_FLAG  FLAG_MASK(0)

/**
 * @brief 
 */
uint8_t comms_state = 0;

/**
 * @brief 
 * 
 * @param UART_Buffer 
 */
void Communications_Handler(volatile char UART_Buffer[]) {

    // if seen, communication is established
    if (Check_UART_Data(UART_Buffer, "!E")) {

        comms_state    |=  ECHO_REQUEST_FLAG;
    }
}


/**
 * @brief Hardware serial port wired to the RPLiDAR C1.
 *        Teensy 4.x Serial1 = pins 0 (RX) / 1 (TX).
 */
#define RPLIDAR_Serial  Serial1



// ----------------------------------------------------------------------------
//  Module-level state
// ----------------------------------------------------------------------------

/**
 * @brief FSM and buffer state for the RPLiDAR C1.
 *        Passed by pointer to Initialize_RPLiDAR_C1() and consulted by
 *        the application to detect PROCESSING frames.
 */
static C1_States rplidar_cfg;

/**
 * @brief Output point cloud populated by Process_RPLiDAR_Data().
 */
static PointCloud rplidar_cloud;

/**
 * @brief IntervalTimer instance used to trigger periodic tasks in a deferred
 *  interrupt handling pattern. Other high priority tasks will be handled in
 *  the task-selector function.
 */
IntervalTimer loop_timer;

#define MS_TO_US        1000
#define LOOP_INTERVAL_MS 100


/**
 * @brief user-defined macro to wait for interrupt Assembly instruction 
 * 
 * @note This macro wraps the assembly instruction "wfi" to improve code
 *  readability.
 */
#define WaitForInterrupt()  asm("wfi")


// ============================================================================
//
//  SETUP
//
// ============================================================================

void setup()
{


#ifdef DEBUG_OUTPUTS

    // USB CDC — wait up to 3 s for a monitor, then continue regardless so
    // the MSP432 hardware-UART handshake is not blocked by USB CDC.
    // Serial.begin(115200); // cannot be initialized here because if we want the MSP432_Serial communication to be live during setup. This needs to be established until after data collection starts at which point the USB stream will send over the point cloud data for analysis on the PC.
    // while (!Serial);

#endif


    // set up LED for debugging
    // Serial.println("Initializing communication with MSP432...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    


    LPUART8_SetPort(&MSP432_Serial);
    LPUART8_Init(460800);
    LPUART8_AttachISR(&Communications_Handler);


    // confirm communication with MSP432 by waiting for an echo response to our handshake message
    while (!(comms_state & ECHO_REQUEST_FLAG)) {
        // Serial.println("Establishing communication...");
        digitalToggle(LED_BUILTIN);

        LPUART8_OutString("!E\r\n");
        
        WaitForInterrupt();
    }


    // confirmmation of comms establishment visually
    digitalWrite(LED_BUILTIN, LOW);

    return;

#ifdef DEBUG_OUTPUTS

    Serial.println("==============================================");
    Serial.println("  RPLiDAR C1 — Teensy Arduino port test");
    Serial.println("==============================================");

#endif

    // Bind Serial1 to the RPLiDAR driver ---------------------------------
    RPLiDAR_UART_SetPort(&RPLIDAR_Serial);

    
    // Initialize scanner:
    //  - Configure_RPLiDAR_Struct(&rplidar_cfg)
    //  - RPLiDAR_UART_Init()   --> Serial1.begin(460800)
    //  - STOP --> RESET --> GET_HEALTH --> SCAN
    Serial.println("[1/3] Initializing RPLiDAR C1...");
    Initialize_RPLiDAR_C1(&rplidar_cfg);

    
    // Now that all TX commands are sent, flush TX and replace HardwareSerial's
    // LPUART6 vector with our bare-metal RX ISR.  Must happen AFTER init so
    // that HardwareSerial's TX-interrupt path is no longer needed.
    Serial.println("[2/3] Attaching bare-metal LPUART6 RX ISR...");
    RPLiDAR_UART_AttachISR();
    
    
    Serial.println("[3/3] Starting loop timer...");
    loop_timer.begin(Task_Selector, LOOP_INTERVAL_MS * MS_TO_US);


    Serial.println("Setup complete. Streaming scan frames:");
    Serial.println("----------------------------------------------");

}


// ============================================================================
//
//  LOOP
//
// ============================================================================

void loop()
{

    // Sleep until the next interrupt (LPUART6_RX_ISR or IntervalTimer).
    // The Cortex-M7 wfi instruction resumes as soon as any unmasked
    // interrupt fires, so byte processing latency is interrupt latency
    // rather than polling latency.
    WaitForInterrupt();

    
    if (task_flag & TASK_3_FLAG) {
        task_flag &= ~TASK_3_FLAG;
        
        Start_Record(NULL);

    }

    // -------------------------------------------------------------------------
    // TASK_4: process a complete scan frame (gated by the task scheduler)
    //
    // task_flag is set by Task_Selector() (IntervalTimer ISR) only when
    // timer_ignore_flag == 0.  The LiDAR FSM sets timer_ignore_flag = 1
    // via _timer_ignore() at recording start and clears it via
    // _timer_acknowledge() when End_Record() transitions the state to
    // PROCESSING.  So TASK_4_FLAG arrives only after a full frame is ready.
    // -------------------------------------------------------------------------

    if (task_flag & TASK_4_FLAG) {
        task_flag &= ~TASK_4_FLAG;

        if (rplidar_cfg.current_state == PROCESSING) {

            Process_RPLiDAR_Data(&rplidar_cloud);

            
            #ifdef PROCESSING4_OUTPUT
            Serial.printf("POSE,%5.2f,%5.2f,%5.2f\n",
                        //   global_pose.x,
                        //   global_pose.y,
                        //   global_pose.theta);
                          0.f,
                          0.f,
                          0.f);   
            Serial.println("SCAN_START");
            #endif


            for (uint32_t i = 0; i < rplidar_cloud.num_pts; i++) {
                
    #ifdef PROCESSING4_OUTPUT
                Serial.printf("P,%5.2f,%5.2f\n",
                              rplidar_cloud.points[i].x,
                              rplidar_cloud.points[i].y);
    #endif
                
            }

    #ifdef PROCESSING4_OUTPUT
            Serial.println("SCAN_END");
    #endif


            // --- Re-arm for next frame --------------------------------------
            rplidar_cfg.current_state   = IDLING;

        } // if (state == PROCESSING)

    } // if (task_flag & TASK_4_FLAG)
}
