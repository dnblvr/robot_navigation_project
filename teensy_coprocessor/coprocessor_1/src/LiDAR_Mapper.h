/**
 * @file LiDAR_Mapper.h
 * @brief Teensy implementation of the LiDAR mapping module.
 *
 * @author Gian Fajardo
 */

#pragma once

#include <Arduino.h>
#include <RPLiDAR_C1.h>
#include <inEKF_se2.h>
#include <UART8.h>
#include <Timer_Tasks.h>

#include <ICP_2D.h>
#include <graphslam.h>
#include <cholesky_decomposition.h>


// ————————————————————————————————————————————————————————————————————————————
//
//  GRAPHSLAM ALGORITHM VARIABLES
//
// ————————————————————————————————————————————————————————————————————————————


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
 * @brief Runs the GraphSLAM algorithm for each iteration.
 * 
 * @param local_cloud Pointer to the localized point cloud.
 * @param today_pose Current pose of the robot.
 * @param delta_pose Change in pose since the last update.
 * 
 * @return void
 */
void Run_GraphSLAM(
        PointCloud* local_cloud, 
        se2_t today_pose, 
        se2_t delta_pose)
{
    
    // Initialize SLAM on first run
    if (!slam_initialized) {
        slam_initialized    = slam_initialize(&slam_optimizer);
    }


    /**
     * @brief use ICP to get an adjacent match and appropriate transformation
     *  to the global frame, with the inEKF output as a guess.
     * 
     * @note  for diagnosis reasons, calculate the confidence of the ICP result
     */

    bool have_icp_correction = false;
    float icp_confidence    = 0.5f;
    ICPResult icp_result;


    // this gets the previous scan and pose
    if (slam_optimizer.buffer_size > 0) {

        // Get previous scan and pose using `prev_id` & the getter functions
        int prev_id = slam_optimizer.buffer_size - 1;
        PointCloud  previous_scan_local;
        se2_t        previous_pose;
        
        // Get previous scan and its pose
        if (    slam_get_scan(&slam_optimizer, prev_id, &previous_scan_local)
             && slam_get_pose(&slam_optimizer, prev_id, &previous_pose))
        {

            /**
             * @todo use the new se2_t structure for the initial guess 
             */
            se2_t init_guess = {delta_pose.x,
                delta_pose.y,
                delta_pose.theta};

            slam_perform_icp_i(&previous_scan_local,
                               local_cloud,
                               &init_guess,
                               &icp_result);
            
            // Compute confidence
            icp_confidence = slam_compute_icp_confidence_i();
            
            have_icp_correction = true;
            
        #ifdef DEBUG_OUTPUT

            float correction_mag = sqrtf(   icp_result.dx*icp_result.dx
                                          + icp_result.dy*icp_result.dy);

            if (    (correction_mag > 5.0f)
                 || (fabsf(icp_result.dtheta) > 0.02f))
            {
                Serial.printf("  ICP correction: dx=%.1f"
                              " dy=%.1f d\u03b8=%.3f (%.1fmm) conf=%.2f\n",
                              icp_result.dx, icp_result.dy, icp_result.dtheta,
                              correction_mag, icp_confidence);
            }

        #endif

        }

    }


    /**
     * @brief add the new pose and scan to the graph
     */
    
    se2_t global_pose    = {today_pose.x, today_pose.y, today_pose.theta};
    se2_t change_pose    = {delta_pose.x, delta_pose.y, delta_pose.theta};
    
    slam_add_pose(&slam_optimizer,
                  &global_pose,
                  local_cloud);

    #ifdef DEBUG_OUTPUT
    Serial.printf("Added pose to buffer. New buffer_size: %d\n",
                  slam_optimizer.buffer_size);
    #endif


    /**
     * @brief compose the icp-corrected pose and add the odometry constraint to the graph: odometry constraints between consecutive poses
     */

    if (slam_optimizer.buffer_size >= 2) {
        
        int prev_id = slam_optimizer.buffer_size - 2;
        int curr_id = slam_optimizer.buffer_size - 1;
        
        
        se2_t measurement;
        

        // Apply ICP correction to odometry measurement
        if (have_icp_correction && icp_confidence > 0.2f) {

            measurement.x       = icp_result.dx;
            measurement.y       = icp_result.dy;
            measurement.theta   = icp_result.dtheta;
            
#ifdef DEBUG_OUTPUT
            Serial.printf("  Measurement: odom=(%.1f,%.1f,%.3f)"
                    " + ICP_corr=(%.1f,%.1f,%.3f)"
                    " = final=(%.1f,%.1f,%.3f)\n",
                   change_pose.x, change_pose.y, change_pose.theta,
                   icp_result.dx, icp_result.dy, icp_result.dtheta,
                   measurement.x, measurement.y, measurement.theta);
#endif

        // Low confidence or no ICP - use recent change in odometry `change_pose` only
        } else {

            measurement.x       = change_pose.x;
            measurement.y       = change_pose.y;
            measurement.theta   = change_pose.theta;
            
#ifdef DEBUG_OUTPUT
            if (have_icp_correction) {
                Serial.printf("  Measurement: odom only"
                              " (ICP conf too low: %.2f)\n",
                              icp_confidence);
            }
#endif
        } // if (have_icp_correction && icp_confidence > 0.2f)

        
        float base_confidence = icp_confidence;
        
        // Compute magnitude of the measurement
        float measurement_magnitude = sqrtf(
                    measurement.x * measurement.x
                 +  measurement.y * measurement.y);
        
        // For small motions (likely noise), use very low confidence
        #define MIN_CONSTRAINT_MOTION_MM 15.0f
        #define MIN_CONFIDENCE_FOR_SMALL_MOTION 0.01f
        
        float effective_confidence = base_confidence;
        if (    measurement_magnitude < MIN_CONSTRAINT_MOTION_MM
             && fabsf(measurement.theta) < 0.05f)
        {

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

            Serial.printf("Constraint %d->%d: meas=(%.2f,%.2f,%.3f)"
                          " mag=%.1f conf=%.2f->%.2f [LOW CONF]\n",
                          prev_id, curr_id,
                          measurement.x, measurement.y, measurement.theta,
                          measurement_magnitude, base_confidence,   effective_confidence);

        } else {

            Serial.printf("Constraint %d->%d: meas=(%.2f,%.2f,%.3f)"
                          " mag=%.1f conf=%.2f [ADDED]\n",
                          prev_id, curr_id,
                          measurement.x, measurement.y, measurement.theta,
                          measurement_magnitude, effective_confidence);

        }
#endif
    } // if (slam_optimizer.buffer_size >= 2) {
    


    /**
     * @brief loop-closure detection
     */
    if (slam_optimizer.buffer_size > MIN_TEMPORAL_GAP + 1) {

        uint8_t loop_detected = slam_detect_loop_closure(
                &slam_optimizer,
                slam_optimizer.buffer_size - 1);

    #ifdef DEBUG_OUTPUT
        Serial.printf("\n*** LOOP CLOSURE DETECTION *** %s\n",
                      loop_detected ? "LOOP DETECTED!": "no loop\n");
    #endif

    }


    /**
     * @brief run the optimizer for every scan (for now), or every few scans,
     *  depending on how long optimization takes.  
     * 
     * @note it should not take long with the teensy's enormous processing power
     */
#ifdef TEST_OPTIMIZATION

    if (    (slam_optimizer.buffer_size > 0)
         && (slam_optimizer.buffer_size % OPTIMIZE_INTERVAL == 0))
    {
        
#ifdef DEBUG_OUTPUT
        Serial.printf("\nOptimizing graph...\n");
#endif

        slam_optimize_gauss_newton(&slam_optimizer, MAX_GAUSS_NEWTON_ITERS);
        

#ifdef PROCESSING_EXPECTED_OUTPUTS
        // Clear visualization before sending complete optimized map
        Serial.printf("CLEAR\n");
#endif

        // Retrieve and transform all optimized poses and their point clouds
        int pose_idx;
        se2_t optimized_pose;
        PointCloud original_scan, transformed_scan;

        // corrects all poses in the buffer and their scans
        for (   pose_idx = 0;
                pose_idx < slam_optimizer.buffer_size;
                pose_idx++)
        {
            
            // Get optimized pose (global)
            if (    !slam_get_pose(&slam_optimizer,
                                   pose_idx,
                                   &optimized_pose) )
            {
                continue;  // Skip if pose retrieval fails
            }
            
            // Get original point cloud for this pose; skip if scan retrieval
            // fails
            if (    !slam_get_scan(&slam_optimizer,
                                   pose_idx,
                                   &original_scan) ) {
                continue;
            }

            // use the ith pose (now global) to transform point cloud to new optimized global frame
            transform_point_cloud(&original_scan,  
                                  &optimized_pose, 
                                  &transformed_scan);

            
            
#ifdef DEBUG_OUTPUT
            // Serial.printf("se2_t[%2d] optimized: x=%.2f y=%.2f theta=%.3f | %d points transformed\n",
            //        pose_idx,
            //        optimized_pose.x, optimized_pose.y, optimized_pose.theta,
            //        transformed_scan.num_pts);
#endif


#ifdef PROCESSING_EXPECTED_OUTPUTS

            // Send optimized pose and transformed scan to PC for visualization
            processing4_print((se2_t){optimized_pose.x,
                                            optimized_pose.y,
                                            optimized_pose.theta},
                               &transformed_scan);
            
#endif
        }

    }

#endif

}


// ————————————————————————————————————————————————————————————————————————————
//
//  ISR CONFIGURATION AND HANDLERS
//
// ————————————————————————————————————————————————————————————————————————————

/**
 * @brief IntervalTimer instance used to trigger periodic tasks in a deferred
 *  interrupt handling pattern. Other high priority tasks will be handled in
 *  the task-selector function.
 */
IntervalTimer loop_timer;

#define MS_TO_US           1000
#define LOOP_INTERVAL_MS    100

/**
 * @brief user-defined macro to wait for interrupt Assembly instruction 
 * 
 * @note This macro wraps the assembly instruction "wfi" to improve code
 *  readability.
 */
#define WaitForInterrupt()  asm("wfi")

/**
 * @brief 
 */
#define MSP432_Serial       Serial5

/**
 * @brief 
 */
extern volatile uint32_t    comms_state;

/**
 * @brief 
 */
extern void Handle_UART_Communications(volatile char UART_Buffer[]);


// ────────────────────────────────────────────────────────────────────────────
//
//  BUZZER AND BUTTON CONFIGURATION
//
// ────────────────────────────────────────────────────────────────────────────

#define BUZZER_PIN   9
#define BUTTON_PIN  14

/**
 * @brief Provide audible feedback using the buzzer.
 */
void beep_feedback() {

    analogWriteFrequency(BUZZER_PIN, 2000u);
    analogWrite(BUZZER_PIN, 40);

    delay(20);
    analogWrite(BUZZER_PIN, 0);
}



// ————————————————————————————————————————————————————————————————————————————
//
//  MODULE LEVEL STATE
//
// ————————————————————————————————————————————————————————————————————————————

/**
 * @brief FSM and buffer state for the RPLiDAR C1.
 *        Passed by pointer to `Initialize_RPLiDAR_C1()` and consulted by
 *        the application to detect PROCESSING frames.
 */
static C1_States rplidar_cfg;

/**
 * @brief Hardware serial port wired to the RPLiDAR C1.
 *        Teensy 4.x `Serial1` = pins 0 (RX) / 1 (TX).
 */
#define RPLIDAR_Serial  Serial1

/**
 * @brief 
 */
extern volatile uint8_t timer_ignore_flag;


// ————————————————————————————————————————————————————————————————————————————
//
//  SETUP
//
// ————————————————————————————————————————————————————————————————————————————

void setup()
{

    // 
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);


    // establish communications with PC and wait until handshake is complete
    Serial.begin(115200); 
    // while (!Serial);
    
    
    // set up LED for debugging
    // Serial.println("Initializing communication with MSP432...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);


    LPUART8_SetPort(&MSP432_Serial);
    LPUART8_Init(460800);
    LPUART8_AttachISR(&Handle_UART_Communications);


    // —— Bind Serial1 to the RPLiDAR driver ——————————————————————————————————
    RPLiDAR_UART_SetPort(&RPLIDAR_Serial);

    
    // Initialize scanner:
    //  - Configure_RPLiDAR_Struct(&rplidar_cfg)
    //  - RPLiDAR_UART_Init()   ——> Serial1.begin(460800)
    //  - STOP ——> RESET ——> GET_HEALTH ——> SCAN
    // Serial.println("[1/4] Initializing RPLiDAR C1...");
    Initialize_RPLiDAR_C1(&rplidar_cfg);

    digitalToggle(LED_BUILTIN);

    
    // Now that all TX commands are sent, flush TX and replace HardwareSerial's
    // LPUART6 vector with our bare-metal RX ISR.  Must happen AFTER init so
    // that HardwareSerial's TX-interrupt path is no longer needed.
    // Serial.println("[2/4] Attaching bare-metal LPUART6 RX ISR...");
    RPLiDAR_UART_AttachISR();
    
    
    
    // confirm communication with MSP432 by waiting for an echo response to our
    // handshake message
    // Serial.println("[3/4] Resetting MSP432 timer...");
    LPUART8_OutString("!E\r\n");
    Block_Wait_Until(ECHO_REQUEST_FLAG);

    // bluetooth module should receive a handshake with  
    // Block_Wait_Until(HALT_REQUEST_FLAG);
    
    
    // confirmation of comms establishment visually
    // Serial.println("Communication established.");
    digitalWrite(LED_BUILTIN, LOW);
    
    
    // Serial.println("[4/4] Starting loop timer...");
    loop_timer.begin(Task_Selector, LOOP_INTERVAL_MS * MS_TO_US);
    
}


// ————————————————————————————————————————————————————————————————————————————
//
//  SUPERLOOP
//
// ————————————————————————————————————————————————————————————————————————————

/**
 * @brief in this cooperative multitasking environment, the main thread sleeps
 *  until an interrupt wakes it up. Then, based on the task flags set by the
 *  Task_Selector (`IntervalTimer` ISR), it executes the corresponding tasks
 *  within the superloop. This design allows for efficient handling of time-
 *  sensitive tasks while keeping the main loop responsive to events.
 */
void loop()
{
    static uint32_t pose_counter   = 0;

    static se2_t previous_pose    = {0.0f, 0.0f, 0.0f};
    static se2_t today_pose       = {0.0f, 0.0f, 0.0f};
    static se2_t delta_pose       = {0.0f, 0.0f, 0.0f};

    // Sleep until the next interrupt (`LPUART6_RX_ISR`, `IntervalTimer`, etc.). The Cortex-M7 `wfi` instruction resumes as soon as any unmasked interrupt fires.
    WaitForInterrupt();

    // Serial.printf(comms_state & HALT_REQUEST_FLAG ? "y": "");
    
    
#ifdef TASK_2_FLAG  // ————————————————————————————————————————————————————————
    /**
     * @note TASK 2: high-priority task that, when triggered, pauses all RPLiDAR data collection indefinitely. Then, the current pose and scan is displayed in the Serial Monitor for debugging and visualization purposes.
     */
    if (    (task_flag   & TASK_2_FLAG)
         && (comms_state & HALT_REQUEST_FLAG))
    {
        task_flag      &= ~TASK_2_FLAG;
        comms_state    &= ~HALT_REQUEST_FLAG;

        // counter
        uint32_t i;
        
        // stop timer to pause all other tasks
        loop_timer.end();

        // here, do not stop the LiDAR sensor as it provides the necessary
        // interrupts to trigger the superloop and to get inside here.


        // wait for button press to resume post-scan processing and/or SLAM optimization, after which there will be visual feedback
        while (digitalRead(BUTTON_PIN) == HIGH);

        // Serial.printf("in Task 2\n");
        digitalToggle(LED_BUILTIN);
        for (i = 0; i < 2*3; i++) {
            digitalToggle(LED_BUILTIN);
            delay(20);
        }

        
        // print all current poses and scans for visualization in Processing
        for (i = 0; i < pose_counter; i++) {

            se2_t pose;
            PointCloud cloud;

            if (    slam_get_pose(&slam_optimizer, i, &pose)
                 && slam_get_scan(&slam_optimizer, i, &cloud))
            {
                se2_t pose_ss = {pose.x, pose.y, pose.theta};

            #ifdef PROCESSING4_EXPECTED_OUTPUTS

                processing4_print(&pose, &cloud);

            #else

                C_format_print(pose, &cloud);

            #endif
                
            }

        } // for (i = 0; i < pose_counter; i++) {


        // optionally, run SLAM on what is left of all frames


    }
    
#endif


#ifdef TASK_3_FLAG  // ————————————————————————————————————————————————————————
    /**
     * @note TASK 3: records the new pose frame if over the distance threshold
     */
    if (task_flag & TASK_3_FLAG) {
        task_flag &= ~TASK_3_FLAG;

        #define MIN_RECORD_LIMIT_MM 100.0f

        // Get current pose
        today_pose      = Get_Current_State();

        /**
         * @brief start a new recording and use it to start SLAM if the pose has
         *  changed significantly.  This is a simple heuristic to trigger new
         *  frames based on motion, rather than just time.
         * @todo just integrate an RTOS into this thing already! lmao
         */
        if (    euclidean_distance_SE2(previous_pose,
                                       today_pose)      > MIN_RECORD_LIMIT_MM
             || (fabsf(today_pose.theta - previous_pose.theta) > 1.5f) )
        {

            Serial.printf("Significant pose change detected: dx=%.1f mm,"
                          " dy=%.1f mm, dθ=%.3f rad\n",
                          today_pose.x - previous_pose.x,
                          today_pose.y - previous_pose.y,
                          today_pose.theta - previous_pose.theta);
                          
            // This function runs to restart the module-level state machine to
            // begin recording a new frame.
            Start_RPLiDAR_C1_Record(NULL);
            
        #ifdef DEBUG_OUTPUT
            // Serial.printf("se2_t request received.\n");
        #endif

            // delta_pose is the incremental change since last frame, which will be used as the odometry measurement for SLAM.  
            difference_SE2(previous_pose, today_pose, &delta_pose);
            
            // only update 
            previous_pose   = today_pose;

            // give sound feedback
            beep_feedback();

            // for this simple test, when a new frame is triggered, this will record
            pose_counter++;

        }

        digitalToggle(LED_BUILTIN);
        delay(20);
        digitalToggle(LED_BUILTIN);
        

    }
#endif


#ifdef TASK_4_FLAG  // ————————————————————————————————————————————————————————
    /**
     * @note TASK_4: process a complete scan frame (gated by the task scheduler)
     *  task_flag is set by `Task_Selector()` (IntervalTimer ISR) only when
     *  `timer_ignore_flag == 0`.  The LiDAR FSM sets `timer_ignore_flag = 1`
     *  via `_timer_ignore()` at recording start and clears it via
     *  `_timer_acknowledge()` when `End_Record()` transitions the state to
     *  `PROCESSING`.  So `TASK_4_FLAG` arrives only after a full frame is
     *  ready.
     */
    if (    (task_flag & TASK_4_FLAG)
         && (rplidar_cfg.current_state == PROCESSING))
    {

        /**
         * @note if defined, then the algorithm will process the scan and pose
         *  data that is preferred by GraphSLAM, untransformed scan.
         * 
         * @note if not defined, then the algorithm will process the scan and
         *  pose data for visual debugging, transformed scan.
         */
        #define GRAPHSLAM_OUTPUT 1

// GRAPHSLAM-compatible OUTPUT (with original untransformed scan)
#ifdef GRAPHSLAM_OUTPUT

        // store local variables for SLAM function
        PointCloud local_cloud;


        // the untransformed scan data is a preferred format for the ICP and slam buffer as it avoids an extra transformation step before optimization and it can be transformed as needed when the optimized poses are retrieved.
        Process_RPLiDAR_Data((se2_t){0.f,0.f,0.f}, &local_cloud);


        // Initialize SLAM on first run
        if (!slam_initialized)
            slam_initialized    = slam_initialize(&slam_optimizer);

        
        se2_t d_p = {delta_pose.x, delta_pose.y, delta_pose.theta};
        
        slam_add_pose(&slam_optimizer,
                      &d_p,
                      &local_cloud);


// NON-GRAPHSLAM OUTPUT (with original untransformed scan)
#else 

        // store local variables
        PointCloud transformed_scan;

        // transform the scan so it can be visualized in the global frame using the current pose estimate `today_pose`. This is not necessary for the SLAM optimization itself, which necessitates the original untransformed scan, but it is useful for visualization and debugging to see the scan in the global frame.  
        Process_RPLiDAR_Data(today_pose, &transformed_scan);


        // Initialize SLAM on first run
        if (!slam_initialized)
            slam_initialized    = slam_initialize(&slam_optimizer);

        
        slam_add_pose(&slam_optimizer,
                      (const se2_t *)&today_pose,
                      &transformed_scan);
        

#endif

        // ——— Re-arm for next frame ——————————————————————————————————————————
        task_flag &= ~TASK_4_FLAG;
        rplidar_cfg.current_state   = IDLING;


        // perform ICP + SLAM optimization
        // Run_GraphSLAM(
        //         &local_cloud, 
        //         today_pose, 
        //         delta_pose);



    } // if (task_flag & TASK_4_FLAG)

#endif 

#ifdef TASK_7_FLAG  // ————————————————————————————————————————————————————————
    if (task_flag & TASK_7_FLAG) {
        task_flag &= ~TASK_7_FLAG;

        static uint32_t counter = 0;
        
        // Serial.printf("cycle %5u\n\n", counter);
        counter++;
    }
#endif

}
