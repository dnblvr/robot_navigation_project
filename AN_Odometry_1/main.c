/**
 * @file main.c
 * @brief Main source code for the Tachometer program.
 *
 * This file contains the main entry point and function definitions for the
 *      Tachometer program.
 *
 *
 * Timer_A is used in this lab:
 *  - Timer A0: Used to generate PWM signals to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (10 Hz)
 *
 * @author Gian Fajardo
 *
 *          massive credit to Aaron Nanas for providing the files
 *
 */

#include <stdint.h>
#include <string.h>

#include "msp.h"
//#include "inc/"
#include "inc/RPLiDAR_A2_UART.h"
#include "inc/RPLiDAR_C1.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
//#include "inc/BLE_A3_UART.h"
#include "inc/GPIO.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Bumper_Switches.h"
#include "inc/Motor.h"
//#include "inc/Timer_A0_Interrupt.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/Tachometer.h"



//#include "arm_math.h"
#include "inc/icp_2d.h"
#include "inc/graphslam.h"
#include "inc/coordinate_transform.h"


// ----------------------------------------------------------------------------
//
//  SLAM ALGORITHM VARIABLES
//
// ----------------------------------------------------------------------------

// SLAM Testing Variables
SLAMOptimizer slam_optimizer;
bool slam_initialized = false;
PointCloud previous_scan;
bool has_previous_scan = false;
Pose current_pose = {0.0f, 0.0f, 0.0f, 0};  // x, y, theta, timestamp
int slam_pose_count = 0;

// Testing flags
#define TEST_ICP_ONLY           1
//#define TEST_ADD_POSE           1
//#define TEST_ODOMETRY_CONSTRAINT 1
//#define TEST_LOOP_CLOSURE       1
//#define TEST_OPTIMIZATION       11


/**
 * @brief Test SLAM components incrementally after processing RPLiDAR data
 * 
 * @param output The processed point cloud in Cartesian coordinates
 */
void test_slam(
    float output[OUTPUT_BUFFER][3])
{

    // counter variable
    int i;
    
    static uint32_t scan_counter = 0;
    PointCloud      current_scan;
    char            debug_buffer[128];

    
    // Convert output array to PointCloud structure
    current_scan.num_points     = 0;
    for (i = 0; i < OUTPUT_BUFFER && i < MAX_POINTS_PER_SCAN; i++) {

        // if is a valid point
        if (output[i][0] != 0.0f || output[i][1] != 0.0f) {  

            current_scan.points[current_scan.num_points].x  = output[i][0];
            current_scan.points[current_scan.num_points].y  = output[i][1];
            current_scan.num_points++;
        }

    }
    
    sprintf(
        debug_buffer,
        "Scan %lu: %d points\n", scan_counter, current_scan.num_points);

//    BLE_UART_OutString(debug_buffer);
    
    // Initialize SLAM on first run
    if (!slam_initialized) {
        slam_initialize(&slam_optimizer);
        slam_initialized = true;
//        BLE_UART_OutString("SLAM initialized\n");
    }
    
    // ========================================================================
    // TEST 1: ICP ONLY
    // ========================================================================
    #ifdef TEST_ICP_ONLY
    if (has_previous_scan && current_scan.num_points > 10) {
        
        ICPResult icp_result;
        Pose initial_guess = {0.0f, 0.0f, 0.0f, 0};  // Identity transform
        float confidence;
        
        // Run ICP between previous and current scan
        slam_perform_icp(
                &previous_scan,
                &current_scan,
                &initial_guess,
                &icp_result);
        
        // Compute confidence
        confidence = slam_compute_icp_confidence(
                            &previous_scan,
                            &current_scan,
                            &icp_result);
        icp_result.confidence = confidence;
        
        // Send results via Bluetooth
        sprintf(
                debug_buffer,
                "ICP: dx=%.3f dy=%.3f dtheta=%.3f conf=%.2f\n",
                icp_result.dx, icp_result.dy, icp_result.dtheta, confidence);
//        BLE_UART_OutString(debug_buffer);
        
        // Visual feedback
        if (confidence > 0.5f) {
            LED2_Output(RGB_LED_GREEN);  // Good match
        } else {
            LED2_Output(RGB_LED_RED);    // Poor match
        }
    }
    #endif
    
    // ========================================================================
    // TEST 2: ADD POSES
    // ========================================================================
    #ifdef TEST_ADD_POSE
    {
        bool success;
        
        // Update pose based on ICP (simple integration)
        if (has_previous_scan) {
            ICPResult icp_result;
            Pose initial_guess = {0.0f, 0.0f, 0.0f, 0};
            
            slam_perform_icp(&previous_scan, &current_scan, &initial_guess, &icp_result);
            
            // Update current pose
            current_pose.x += icp_result.dx;
            current_pose.y += icp_result.dy;
            current_pose.theta += icp_result.dtheta;
            current_pose.theta = normalize_angle(current_pose.theta);
        }
        
        // Add pose to SLAM
        success = slam_add_pose(&slam_optimizer, &current_pose, &current_scan);
        
        if (success) {
            slam_pose_count++;
//            sprintf(debug_buffer, "Pose %d added: x=%.2f y=%.2f theta=%.2f\n",
//                    slam_pose_count, current_pose.x, current_pose.y, current_pose.theta);
//            BLE_UART_OutString(debug_buffer);
        } else {
//            BLE_UART_OutString("ERROR: Failed to add pose\n");
        }
    }
    #endif
    
    // ========================================================================
    // TEST 3: ODOMETRY CONSTRAINTS
    // ========================================================================
    #ifdef TEST_ODOMETRY_CONSTRAINT
    if (slam_pose_count > 1) {
        
        int prev_id = slam_pose_count - 2;
        int curr_id = slam_pose_count - 1;
        
        ICPResult icp_result;
        Pose initial_guess = {0.0f, 0.0f, 0.0f, 0};
        
        slam_perform_icp(&previous_scan, &current_scan, &initial_guess, &icp_result);
        
        // Add odometry constraint between consecutive poses
        slam_add_odometry_constraint(
            &slam_optimizer,
            prev_id,
            curr_id,
            icp_result.dx,
            icp_result.dy,
            icp_result.dtheta,
            100.0f  // confidence
        );
        
        sprintf(debug_buffer, "Constraint added: %d -> %d\n", prev_id, curr_id);
        BLE_UART_OutString(debug_buffer);
    }
    #endif
    
    // ========================================================================
    // TEST 4: LOOP CLOSURE DETECTION
    // ========================================================================
    #ifdef TEST_LOOP_CLOSURE
    if (slam_pose_count > MIN_TEMPORAL_GAP + 1) {
        
        bool loop_detected = slam_detect_loop_closure(
            &slam_optimizer,
            slam_pose_count - 1
        );
        
        if (loop_detected) {
            BLE_UART_OutString("*** LOOP CLOSURE DETECTED! ***\n");
            LED2_Output(RGB_LED_BLUE);
            Clock_Delay1ms(500);
        }
    }
    #endif
    
    // ========================================================================
    // TEST 5: OPTIMIZATION
    // ========================================================================
    #ifdef TEST_OPTIMIZATION
    if (slam_pose_count > 0 && (slam_pose_count % OPTIMIZE_INTERVAL == 0)) {
        
        BLE_UART_OutString("Optimizing graph...\n");
        LED2_Output(RGB_LED_PINK);
        
        slam_optimize_gauss_newton(&slam_optimizer, 5);
        
        // Get optimized current pose
        Pose optimized_pose;
        slam_get_current_pose(&slam_optimizer, &optimized_pose);
        
        sprintf(debug_buffer, "Optimized: x=%.2f y=%.2f theta=%.2f\n",
                optimized_pose.x, optimized_pose.y, optimized_pose.theta);
        BLE_UART_OutString(debug_buffer);
        
        LED2_Output(RGB_LED_GREEN);
    }
    #endif
    
    // ========================================================================
    // SEND TO VISUALIZER (if enabled)
    // ========================================================================
    #if defined(TEST_ADD_POSE) || defined(TEST_OPTIMIZATION)
    {
        Pose viz_pose;
        slam_get_current_pose(&slam_optimizer, &viz_pose);
        
        // Send pose
        sprintf(
                debug_buffer,
                "POSE,%.3f,%.3f,%.3f\n", 
                viz_pose.x, viz_pose.y, viz_pose.theta);
        BLE_UART_OutString(debug_buffer);
        

        // Send scan (decimated)
        BLE_UART_OutString("SCAN_START\n");
        for (i = 0; i < current_scan.num_points; i += 4) {  // Every 4th point

            sprintf(debug_buffer,
                    "POINT,%.3f,%.3f\n",
                    current_scan.points[i].x, current_scan.points[i].y);
            BLE_UART_OutString(debug_buffer);
        }

        BLE_UART_OutString("SCAN_END\n");

    }
    #endif
    
    // Save current scan as previous for next iteration
    previous_scan       = current_scan;
    has_previous_scan   = true;
    scan_counter++;

}


// ----------------------------------------------------------------------------
//
//  HIGHER-LEVEL RPLiDAR VARIABLES
//
// ----------------------------------------------------------------------------

/**
 * @brief declaration of an `RPLiDAR_Config` struct instance
 * @note will be declared in RPLiDAR_C1.c source file
 */
extern RPLiDAR_Config cfg;


/**
 * @brief RPLiDAR C1 RX data buffer.
 */
uint8_t RPLiDAR_RX_Data[RPLiDAR_UART_BUFFER_SIZE] = {0};


/**
 * @brief RPLiDAR C1 output data buffer.
 */
float output[OUTPUT_BUFFER][3] = {0};



// ----------------------------------------------------------------------------
//
//  MOTOR-OPERATION FUNCTIONS
//
// ----------------------------------------------------------------------------


/**
 * @brief Motor speed settings
 */
//uint16_t forward_speed  = 500, // forward duty cycle
//         backward_speed = 300, // reverse duty cycle
//         rotation_speed = 300; // rotation duty cycle

/**
 * @brief MotorCommand function pointer type
 *
 * @param[in] left_speed    The speed of the left motor.
 * @param[in] right_speed   The speed of the right motor.
 */
//typedef void (*MotorCommand)(uint16_t left_speed, uint16_t right_speed);

/**
 * @brief instance of the MotorCommand function pointer
 */
//static MotorCommand command;

/**
 * @brief   Wrapper function to stop the motors. In order to offer compatibility with the
 *      MotorCommand function prototype, the parameters are unused.
 * 
 * @param[in] left_speed    The speed of the left motor.
 * @param[in] right_speed   The speed of the right motor.
 */
//void Motor_Stop_Wrapper(uint16_t left_speed, uint16_t right_speed) {
//
//    Motor_Stop();
//}




// ----------------------------------------------------------------------------
//
//  TACHOMETER FUNCTIONS
//
// ----------------------------------------------------------------------------


// Initialize length of the tachometer buffers
#define TACHOMETER_BUFFER_LEN         10

// Set the maximum RPM for both wheels
//#define MAX_RPM                       120

// Set the minimum RPM for both wheels
//#define MIN_RPM                       30

// Desired RPM for the left wheel
uint16_t Desired_RPM_Left           = 70;

// Desired RPM for the right wheel
uint16_t Desired_RPM_Right          = 70;

// Declare a global variable used to store the measured RPM by the left
// tachometer
//uint16_t Actual_RPM_Left            = 0;

// Declare a global variable used to store the measured RPM by the right
// tachometer
//uint16_t Actual_RPM_Right           = 0;

// Set initial duty cycle of the left wheel to 25%
//uint16_t Duty_Cycle_Left            = 3750;

// Set initial duty cycle of the right wheel to 25%
//uint16_t Duty_Cycle_Right           = 3750;

// Number of left wheel steps measured by the tachometer
int32_t Left_Steps                  = 0;

// Number of right wheel steps measured by the tachometer
int32_t Right_Steps                 = 0;



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
//
////#define MOTOR_CONTROL_SYSTEM 1
//
//void Get_Odometry()
//{
////    char buffer[40];
//
//
//    static uint32_t counter = 0;
//
//    buffer_idx = buffer_idx + 1;
//
//    counter++;
//
//    // the incremental control system is updated when the buffer is filled.
//    // from there, it takes the average
//    if (buffer_idx >= TACHOMETER_BUFFER_LEN)
//    {
//
//        // reset the buffer index
//        buffer_idx = 0;
//
//        // (1/Tachometer Step/Cycles) * (12,000,000 Cycles / Second) * (60 Second / Minute) * (1/360 Rotation/Step)
//
//        Actual_RPM_Left     = 2000000 / ( Average_of_Buffer(Tachometer_Buffer_Left,  TACHOMETER_BUFFER_LEN) );
//        Actual_RPM_Right    = 2000000 / ( Average_of_Buffer(Tachometer_Buffer_Right, TACHOMETER_BUFFER_LEN) );
//
//
//#ifdef MOTOR_CONTROL_SYSTEM
//
//        // If the actual RPM measured on the left wheel is greater than
//        // the desired RPM, then decrease the duty cycle on the left
//        // wheel
//        if (        (Actual_RPM_Left > (Desired_RPM_Left + 3))
//                &&  (Duty_Cycle_Left > 100)) {
//
//            Duty_Cycle_Left = Duty_Cycle_Left - 100;
//
//
//        // Otherwise, if the actual RPM is less than the desired RPM,
//        // then increase the duty cycle on the left wheel
//        } else if (     (Actual_RPM_Left < (Desired_RPM_Left - 3))
//                    &&  (Duty_Cycle_Left < 14898)) {
//
//            Duty_Cycle_Left = Duty_Cycle_Left + 100;
//
//        }
//
//
//        // If the actual RPM measured on the right wheel is greater
//        // than the desired RPM, then decrease the duty cycle on the
//        // right wheel
//        if (        (Actual_RPM_Right > (Desired_RPM_Right + 3))
//                &&  (Duty_Cycle_Right > 100)) {
//
//            Duty_Cycle_Right = Duty_Cycle_Right - 100;
//
//
//        // Otherwise, if the actual RPM is less than the desired RPM,
//        // then increase the duty cycle on the right wheel
//        } else if (     (Actual_RPM_Right < (Desired_RPM_Right - 3))
//                    &&  (Duty_Cycle_Right < 14898)) {
//
//            Duty_Cycle_Right = Duty_Cycle_Right + 100;
//        }
//
//
//#endif // defined(MOTOR_CONTROL_SYSTEM)
//
//    }
//
//
//    // Move the motors using the function pointer "command" with the updated duty cycle
//    if (command != NULL) {
//
//        command(Duty_Cycle_Left, Duty_Cycle_Right);
//
//    } else {
//
//        Motor_Stop();
//    }
//
//
//
//}


// ----------------------------------------------------------------------------
//
//  BLUETOOTH COMMUNICATION FUNCTIONS
//
// ----------------------------------------------------------------------------


/**
 * @brief local instance of message_length globalized from BLE_A3_UART.c
 */
extern volatile int message_length;


int begin_state     = 0;

/**
 * @brief processes UART feed based on if they're similar to
 *
 * @param BLE_UART_Buffer
 */
//void Process_BLE_UART_Data(volatile char BLE_UART_Buffer[])
//{
////    printf("len = %2d\n", message_length);
//
//    // internal counter variable
//    int j;
//
//    if (message_length < 4)
//        return;
//
//    printf("BLE UART Data: ");
//
//    for (j = 0; j < message_length; j++) {
//        printf("%c", BLE_UART_Buffer[j]);
//    }
//
//    printf("\n");
//
//
//    command = NULL;
//
//    if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B11")) {
//
//        command             = &Motor_Stop_Wrapper;
//        Desired_RPM_Left    = 0;
//        Desired_RPM_Right   = 0;
//
//
//    // 2: begin operation
//    } else
//    if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B21")) {
//
//        command             = &Motor_Stop_Wrapper;
//        Desired_RPM_Left    = 0;
//        Desired_RPM_Right   = 0;
//
//        begin_state = 1;
//
//        LED2_Output(RGB_LED_WHITE);
//
//
//    // 5: UP is activated when pressed
//    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B51")) {
//
//        command             = &Motor_Forward;
//        Desired_RPM_Left    = forward_speed;
//        Desired_RPM_Right   = forward_speed;
//
//        LED2_Output(RGB_LED_GREEN);
//
//
//    // 6: DOWN is activated when pressed
//    } else
//    if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B61")) {
//
//        command             = &Motor_Backward;
//        Desired_RPM_Left    = backward_speed;
//        Desired_RPM_Right   = backward_speed;
//
//        LED2_Output(RGB_LED_PINK);
//
//
//    // 7: LEFT is activated when pressed
//    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B71")) {
//
//        // update duty cycles
//        command             = &Motor_Left;
//        Desired_RPM_Left    = rotation_speed;
//        Desired_RPM_Right   = rotation_speed;
//
////        LED2_Output(RGB_LED_YELLOW);
//
//
//    // 8: RIGHT is activated when pressed
//    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B81")) {
//
//        // update duty cycles
//        command             = &Motor_Right;
//        Desired_RPM_Left    = rotation_speed;
//        Desired_RPM_Right   = rotation_speed;
//
////        LED2_Output(RGB_LED_BLUE);
//
//
//    } else {
//
//        command             = &Motor_Stop_Wrapper;
//        Desired_RPM_Left    = 0;
//        Desired_RPM_Right   = 0;
//
//        LED2_Output(RGB_LED_OFF);
//
//    }
//
//    // to apply the command only once, clear the buffer memory
//    for (j = 0; j < message_length; j++) {
//        BLE_UART_Buffer[j] = 0;
//    }
//    message_length = 0;
//
//
//    // Move the motors using the function pointer "command" with the updated duty cycle
//    if (command != NULL) {
//
//        command(Desired_RPM_Left, Desired_RPM_Right);
//
//    } else {
//
//        Motor_Stop();
//    }
//
//}



/**
 * @brief Calculate robot pose from wheel encoder data
 *
 * @param left_steps_mm   Left wheel displacement in mm
 * @param right_steps_mm  Right wheel displacement in mm
 * @param pose            Pointer to current pose (will be updated)
 *
 * @details   I am following the yaw orientation convention
 *          if Left_Steps > Right_Steps:
 *              (+) yaw
 *          vice versa
 *          450 mm required from both wheels to maintain simple rotation
 */
void Get_Pose(
        uint32_t local_counter,
        int32_t Left_Steps,
        int32_t Right_Steps,
        Pose *pose_at_time)
{

    #define CONVERT_COUNTS_TO_DIST 0.6111111111
    #define WHEEL_BASE_MM 141.0f  // Distance between wheels in mm


    // variables for the previous values for later
    static float    prev_left_mm   = 0.0f,
                    prev_right_mm  = 0.0f;


    float   left_steps_mm, right_steps_mm,
            delta_left, delta_right,

            delta_s,        // travel distance along a curved direction
            delta_theta,

            x_local, y_local;

    // saved values for later
    float cos_theta, sin_theta;

    // Convert encoder counts to distance in mm
    left_steps_mm   = CONVERT_COUNTS_TO_DIST * Left_Steps;
    right_steps_mm  = CONVERT_COUNTS_TO_DIST * Right_Steps;


    // Calculate change in wheel positions since last update
    delta_left  = left_steps_mm  - prev_left_mm;
    delta_right = right_steps_mm - prev_right_mm;


    // Save current values for next iteration
    prev_left_mm    = left_steps_mm;
    prev_right_mm   = right_steps_mm;


    // Calculate linear and angular displacement
    delta_s     = (delta_left + delta_right) / 2.0f;  // Average linear displacement
    delta_theta = (delta_right - delta_left) / WHEEL_BASE_MM;  // Angular change (radians)


    // Calculate displacement in local (robot) frame;
    // based on arc motion; based on formula `s = r*theta`
    // Option 1: Moving straight; avoid division by zero
    if (fabs(delta_theta) < 1e-6) {

        x_local = delta_s;
        y_local = 0.0f;

    // Option 2: arc motion; uses the formula
    } else {

        float radius    = delta_s / delta_theta;
        x_local = radius * sin(delta_theta);
        y_local = radius * (1.0f - cos(delta_theta));
    }


    // Transform to global frame using current orientation
    cos_theta   = cos(pose_at_time->theta);
    sin_theta   = sin(pose_at_time->theta);


    pose_at_time->x     += cos_theta*x_local - sin_theta*y_local;
    pose_at_time->y     += sin_theta*x_local + cos_theta*y_local;
    pose_at_time->theta += delta_theta;


    // Normalize angle to [-π, π]
    pose_at_time->theta = normalize_angle(pose_at_time->theta);


    // Debug output
//    printf("trv %3u: L=%7.2f R=%7.2f | x=%7.2f y=%7.2f θ=%6.3f\n",
//            local_counter,
//            left_steps_mm, right_steps_mm,
//            pose_at_time->x, pose_at_time->y, pose_at_time->theta);
}




void print_results(void) {

    static int  l   = 0;


    // print the first n resutls
    if (l < 35) {

    //    for (k = 0; k < cfg.print_counter; k += 5) {
//        for (k = 0; k < 25*MESSAGE_LENGTH; k += 5) {
//            printf("%02X%02X%02X%02X%02X %i; ",
//                   RPLiDAR_RX_Data[k+0],
//                   RPLiDAR_RX_Data[k+1],
//                   RPLiDAR_RX_Data[k+2],
//                   RPLiDAR_RX_Data[k+3],
//                   RPLiDAR_RX_Data[k+4],
//                   pattern(RPLiDAR_RX_Data + k));
//
//        }
//        printf("\n\n");
//    }

    // confirms that the bits are aligned


//        for (k = 0; k < RPLiDAR_UART_BUFFER_SIZE; k += MSG_LENGTH) {
//
//            if ( (k % 4*MSG_LENGTH*MSG_LENGTH) == 0 ) {
//                printf("_");
//            }
//
//
//            printf("%i", pattern(RPLiDAR_RX_Data + k));
//
//        }
//
//        printf("\n\n");

//        printf("output = \n");
//        for (k = 0; k < OUTPUT_BUFFER; k+=5) {
//
//            printf("%03.2f %03.2f\n", output[k][0], output[k][1]);
//
//        }
//        printf("\n\n");


        l++;
    }


}


// ----------------------------------------------------------------------------
//
//  TIMER A1 TASK SELECTOR
//
// ----------------------------------------------------------------------------


#define TASK_0_FLAG     0x01 << 0
#define TASK_0_DIV_FREQ 1
#define TASK_0_OFFSET   0

#define TASK_1_FLAG     0x01 << 1
#define TASK_1_DIV_FREQ 1
#define TASK_1_OFFSET   0

#define TASK_2_FLAG     0x01 << 2
#define TASK_2_DIV_FREQ 10
#define TASK_2_OFFSET   0

#define TASK_3_FLAG     0x01 << 3
#define TASK_3_DIV_FREQ 20
#define TASK_3_OFFSET   0

#define TASK_4_FLAG     0x01 << 4
#define TASK_4_DIV_FREQ 20
#define TASK_4_OFFSET   5

//#define TASK_5_FLAG     0x01 << 5
//#define TASK_5_DIV_FREQ 20
//#define TASK_5_OFFSET   19

#define TASK_6_FLAG     0x01 << 6
#define TASK_6_DIV_FREQ 1
#define TASK_6_OFFSET   0

#define TASK_7_FLAG     0x01 << 7
#define TASK_7_DIV_FREQ 20
#define TASK_7_OFFSET   19


volatile uint8_t    task_flag       = 0;
volatile uint32_t   tick_counter    = 0;

/**
 * @brief Task selector function
 * 
 */
void Task_Selector(void) {


    // task 0
    if ( ((tick_counter + TASK_0_OFFSET) % TASK_0_DIV_FREQ) == 0 ) {
        task_flag  |= TASK_0_FLAG;
//        printf("T0\n");
    }


    // task 1
//    if ( ((tick_counter + TASK_1_OFFSET) % TASK_1_DIV_FREQ) == 0 )
//        task_flag  |= TASK_1_FLAG;

    // task 2
    if ( ((tick_counter + TASK_2_OFFSET) % TASK_2_DIV_FREQ) == 0 ) {


        task_flag  |= TASK_2_FLAG;

    }

    // task 3
    if ( ((tick_counter + TASK_3_OFFSET) % TASK_3_DIV_FREQ) == 0 )
        task_flag  |= TASK_3_FLAG;

    // task 4
    if ( ((tick_counter + TASK_4_OFFSET) % TASK_4_DIV_FREQ) == 0 )
        task_flag  |= TASK_4_FLAG;


    // task 5
//    if ( ((tick_counter + TASK_5_OFFSET) % TASK_5_DIV_FREQ) == 0 )
//        task_flag  |= TASK_5_FLAG;

    // task 6
    if ( ((tick_counter + TASK_6_OFFSET) % TASK_6_DIV_FREQ) == 0 ) {
        task_flag  |= TASK_6_FLAG;
    }


    // task 7 - send results
    if ( ((tick_counter + TASK_7_OFFSET) % TASK_7_DIV_FREQ) == 0 )
        task_flag  |= TASK_7_FLAG;


    // Increment the counter
    tick_counter++;

}



/**
 * @brief set up all receive and send UART interrupts
 */
void Set_All_Interrupts_1(void) {


    EUSCI_A0->IE   |=  0x0001;  // printf()
    EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1
    EUSCI_A3->IE   |=  0x0001;  // BLE UART Friend

    // take out the register that assigns the interrupt EN of TIMER_A1
//    TIMER_A1->CCTL[0]  |= 0x0010;

}

/**
 * @brief set up all receive and send UART interrupts
 * @details difference 1 & 2 is that it holds and releases the whole EUSCI_Ax block
 *      while performing changes. In reality, it is not necessary
 */
void Set_All_Interrupts_2(void) {

    EUSCI_A0->CTLW0    |=  0x01;
    EUSCI_A2->CTLW0    |=  0x01;
    EUSCI_A3->CTLW0    |=  0x01;


    EUSCI_A0->IE   |=  0x0001;  // printf()
    EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1
    EUSCI_A3->IE   |=  0x0001;  // BLE UART Friend
//    TIMER_A1->CCTL[0]  |= 0x0010;


    EUSCI_A0->CTLW0    &= ~0x01;
    EUSCI_A2->CTLW0    &= ~0x01;
    EUSCI_A3->CTLW0    &= ~0x01;

}

// ----------------------------------------------------------------------------
//
//  CONFIGURATION and MAIN LOOP
//
// ----------------------------------------------------------------------------

int main(void) {

    // disable watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // main-loop counter
    int local_counter   = 0;

    // Initialize a buffer that will be used to receive command string from the
    // BLE module
//    volatile char BLE_UART_Data_Buffer[BLE_UART_BUFFER_SIZE] = {0};


    // Initialize the 48 MHz Clock
    Clock_Init48MHz();


    // Initialize the built-in red LED and the RGB LEDs
//    LED1_Init();
    LED2_Init();


    // Initialize the user buttons
//    Buttons_Init();


    // Initialize the front and back LEDs on the chassis board
//    Chassis_Board_LEDs_Init();


    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();



    // Initialize the bumper switches which will be used to generate external
    // I/O-triggered interrupts
//    Bumper_Switches_Init(&Bumper_Switches_Handler);
//    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);


    // use Timer_A1_Interrupt to get the odometry measurements
    Timer_A1_Interrupt_Init(
            &Task_Selector,
            TIMER_A1_CCR0_VALUE); // in clock ticks


    // Initialize the tachometers as a function of Timer A3
    Tachometer_Init();


    // Initialize the DC motors
//    Motor_Init();


    Set_All_Interrupts_1();


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();

    // RPLiDAR C1 initialization ---------------------------------------

    initialize_RPLiDAR_C1(&cfg, RPLiDAR_RX_Data);

    printf("C1 initialized\n\n");

    // RPLiDAR C1 initialization end -----------------------------------


    Set_All_Interrupts_1();


    printf("test scan -------\n\n");

    // BlueTooth module initialization ---------------------------------

//    BLE_UART_Init(BLE_UART_Data_Buffer);
//    Clock_Delay1ms(1000);
//    BLE_UART_Reset();
//    BLE_UART_OutString("BLE UART Active\r\n");
//    Clock_Delay1ms(1000);


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();


    EUSCI_A0->IE   |=  0x0001;  // printf()
    EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1
    EUSCI_A3->IE   |=  0x0001;  // BLE UART Friend


    printf("pressed\n");


    // Bluetooth module initialization end -----------------------------

    TIMER_A1->CCTL[0]  |=  0x0010;

    while (1)
    {

        /**
         * @note   blocks execution until any interrupt occurs
         */
        WaitForInterrupt();

        /**
         * @note receiving information over UART disables RXIE for whatever reason.
         *       Enabling RXIE after __WFI() ensures that RXIE is *always* ON.
         */


        EUSCI_A0->IE   |=  0x0001;  // printf()
        EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1
        EUSCI_A3->IE   |=  0x0001;  // BLE UART Friend
        TIMER_A1->CCTL[0]  |= 0x0010;




//        printf("%04X\n", EUSCI_A3->IE);



        // TASK 0: receive BLE motor commands at 10 times a second
        if (task_flag & TASK_0_FLAG) {
            task_flag  &= ~TASK_0_FLAG; // clear the flag

//            Timer_A1_Ignore();

//            Process_BLE_UART_Data(BLE_UART_Data_Buffer);

//            Timer_A1_Acknowledge();
        }


        // TASK 1: increment local_counter
//        if (task_flag & TASK_1_FLAG) {
//            task_flag  &= ~TASK_1_FLAG;
//
//
//        }



        /**
         * @note TASK 2: Get the measurements made by the tachometers
         */
        if (task_flag & TASK_2_FLAG) {
            task_flag  &= ~TASK_2_FLAG;



            Tachometer_Get(
                    &Tachometer_Buffer_Left[buffer_idx],
                    &Left_Direction,
                    &Left_Steps,

                    &Tachometer_Buffer_Right[buffer_idx],
                    &Right_Direction,
                    &Right_Steps);


            Get_Pose(local_counter, Left_Steps, Right_Steps, &current_pose);
            current_pose.timestamp = tick_counter;  // Record timestamp


        }


        // TASK 3: turn on the RPLiDAR C1
        if (task_flag & TASK_3_FLAG) {
            task_flag  &= ~TASK_3_FLAG;


            if (cfg.current_state == IDLING) {

                // clear the memory before proceeding
                memset(RPLiDAR_RX_Data, 0, RPLiDAR_UART_BUFFER_SIZE);

                cfg.current_state   = READY;
            }

        }



        /**
         * @note prepares the processor to record data
         */
        if (task_flag & TASK_4_FLAG) {
            task_flag  &= ~TASK_4_FLAG;


#define DATA_PT 5

            Timer_A1_Ignore();


            if (cfg.current_state == PROCESSING) {


                int aligned = 1;
                int k;
//
                for (k = 0; k < RPLiDAR_UART_BUFFER_SIZE; k += 2*MSG_LENGTH) {
//
                     printf("%i", pattern(RPLiDAR_RX_Data + k));
//
//                    aligned &=      pattern(RPLiDAR_RX_Data + k + 0) \
//                                &&  pattern(RPLiDAR_RX_Data + k + 1);
//
                }
                aligned =       pattern(RPLiDAR_RX_Data + 0) \
                            &&  pattern(RPLiDAR_RX_Data + 1) \
                            &&  pattern(RPLiDAR_RX_Data + 2) \
                            &&  pattern(RPLiDAR_RX_Data + 3);

                printf("\n%d\n", aligned);

                if (local_counter > 5) {
//                if (aligned) {

//                    printf("recording\n");

                    process_rplidar_data(RPLiDAR_RX_Data, output);

//                    printf("%3i: %7.2f, %7.2f\n",
//                           DATA_PT, output[DATA_PT][0], output[DATA_PT][1]);


                    // Transform point cloud to global frame using current pose
                    float T_matrix[3][3] = {0};
                    PointCloud transformed_cloud;
                    
                    // Create transformation matrix from current pose
                    make_transformation_matrix_pose(&current_pose, T_matrix);
                    

                    // Transform each point to global frame and populate PointCloud
                    transformed_cloud.num_points = 0;

                    int k;
                    for (k = 0; k < OUTPUT_BUFFER; k++) {

                        // Skip invalid points
                        if (output[k][0] == 0.0f && output[k][1] == 0.0f) {
                            continue;
                        }
                        
                        // Each point as homogeneous coordinates [x, y, 1]
                        float point[3] = {output[k][0], output[k][1], 1.0f};
                        
                        // Transformed point = T_matrix * point
                        transformed_cloud.points[transformed_cloud.num_points].x \
                            =    T_matrix[0][0]*point[0] \
                               + T_matrix[0][1]*point[1] \
                               + T_matrix[0][2]*point[2];
                        transformed_cloud.points[transformed_cloud.num_points].y \
                            =    T_matrix[1][0]*point[0] \
                               + T_matrix[1][1]*point[1] \
                               + T_matrix[1][2]*point[2];
                        
                        transformed_cloud.num_points++;
                    }
                    
//                    printf("Transformed cloud: %d points | Sample %3i: local(%7.2f, %7.2f) -> global(%7.2f, %7.2f)\n",
//                           transformed_cloud.num_points, DATA_PT,
//                           output[DATA_PT][0], output[DATA_PT][1],
//                           transformed_cloud.points[DATA_PT].x, transformed_cloud.points[DATA_PT].y);

    //                test_slam(&transformed_cloud);


                }

                // regardless, reset the state to IDLE
                cfg.current_state   = IDLING;





            } else {

                printf("counting\n");

            }

            Timer_A1_Acknowledge();

        }


        if (task_flag & TASK_6_FLAG) {
            task_flag  &= ~TASK_6_FLAG;

//            printf("am here 6");
            printf("%5d\n", local_counter);


        }




        /**
         * @note TASK 6: increment counter if either:
         *      - LiDAR data is correct!
         */
        if (task_flag & TASK_7_FLAG) {
            task_flag  &= ~TASK_7_FLAG;


            printf("\n\n\n");


            local_counter++;
        }

    }

}

