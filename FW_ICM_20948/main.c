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
//#include "inc/RPLiDAR_C1.h"
#include "inc/BLE.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/EUSCI_A2_UART.h"
#include "inc/GPIO.h"
//#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/Tachometer.h"
#include "inc/GPIO_Utilities.h"

#include "inc/ICM_20948.h"



#ifdef DEBUG_OUTPUT
#include "inc/Profiler.h"
#endif

//#include "arm_math.h"
#include "inc/matrices.h"
#include "inc/coordinate_transform.h"
//#include "inc/ICP_2D.h"
//#include "inc/graphslam.h"

void I2C_Task(void) {
    printf("hi\n");

}


volatile uint8_t comms_established = 0;


void Communications_Handler(volatile char UART_Buffer[]) {

    // if seen, communication established and echo back
    if (Check_UART_Data(UART_Buffer, "!E\r\n")) {

        UART_A2_OutString("!E\r\n");

        comms_established = 1;
    }

}


// ----------------------------------------------------------------------------
//
//  GRAPHSLAM ALGORITHM VARIABLES
//
// ----------------------------------------------------------------------------


/**
 * @brief Incremental pose change since last scan (from odometry)
 */
Pose incremental_pose = {0.0f, 0.0f, 0.0f, 0};

/**
 * @brief Global accumulated pose estimate (dead reckoning)
 */
Pose global_pose = {0.0f, 0.0f, 0.0f, 0};



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

    LED2_Output(RGB_LED_BLUE);

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();
    printf("printf enabled\n");


    // use Timer_A1_Interrupt to get the odometry measurements
    // resulting output will be measured in increments of 10 ticks per second
    Timer_A1_Interrupt_Init(&Timer_A1_Task_Selector,
                            TIMER_A1_CCR0_VALUE);


    // Initialize the tachometers as a function of Timer A3
    Tachometer_Init();
    Reset_Pose_Accumulator(pose_ptr);



    UART_A2_Init(&Communications_Handler);


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();

    while (!comms_established) {
        EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1

    }


    LED2_Output(RGB_LED_RED);

    return;


#ifdef OLD_SYSTEM

    // Initialize the ICM20948
    EUSCI_B1_I2C_Init();

    icm20948_config_t config = {.addr_accel_gyro    = ICM20948_ADDR_ACCEL_GYRO,
                                .addr_mag           = ICM20948_ADDR_MAG};

    
    if (icm20948_init(&config) == 0)
        printf("ICM-20948 successfully initialized!\n");
    else
        printf("ICM-20948 unsuccessfully initialized!\n");


    int16_t gyro_raw[3]     = {0},
            mag_raw[3]      = {0};

    float   gyro_dps[3]     = {0},
            mag_ut[3]       = {0};
#else

    // Initialize the ICM20948 with interrupt from pin at P4.6
    EUSCI_B1_I2C_Init(&I2C_Task);

    if (icm20948_init(ICM20948_ADDR_ACCEL_GYRO_0, ICM20948_ADDR_MAG) == 0)
        printf("ICM-20948 successfully initialized!\n");
    else
        printf("ICM-20948 unsuccessfully initialized!\n");

#endif


    // Initialize the DC motors
    Motor_Init();


    Set_All_Interrupts_1();


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();


    Set_All_Interrupts_1();


#ifdef DEBUG_OUTPUT
    printf("test scan -------\n\n");
#endif


    /** ----------------------------------------------------------------------
     * Initialize Bluetooth module instance
     */

     BLE_UART_Init(Process_BLE_UART_Data);
     Clock_Delay1ms(1000);
     BLE_UART_Reset();
     BLE_UART_OutString("BLE UART Active\r\n");
     Clock_Delay1ms(1000);


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();


    EUSCI_A0->IE   |=  0x0001;  // printf()
//    EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1
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

//            printf("hello\n");

//            Timer_A1_Ignore();
//            Process_BLE_UART_Data(BLE_UART_Data_Buffer);
//            Timer_A1_Acknowledge();

        }
#endif


#ifdef TASK_1_FLAG
        /**
         * @note TASK 1: access ICM20948
         */
        if (task_flag & TASK_1_FLAG) {
            task_flag  &= ~TASK_1_FLAG;

#ifdef OLD_SYSTEM

            // counter variable
            uint32_t i;

            icm20948_read_raw_gyro(&config, gyro_raw);
            icm20948_read_raw_mag(&config, mag_raw);

            for (i = 0; i < 3; i++) {
                gyro_dps[i] = (float)gyro_raw[i] / 131.0f;
                mag_ut[i]   = ((float)mag_raw[i] / 20) * 3;
            }

            // accel(g)   = raw_value / (65535 / full_scale)
            // ex) if full_scale == +-4g then accel = raw_value / (65535 / 8) = raw_value / 8192
            // gyro(dps)  = raw_value / (65535 / full_scale)
            // ex) if full_scale == +-250dps then gyro = raw_value / (65535 / 500) = raw_value / 131
            // mag(uT)    = raw_value / (32752 / 4912) = (approx) (raw_value / 20) * 3
            // temp  = ((raw_value - ambient_temp) / speed_of_sound) + 21

//            printf("gyro.  x: %+2.5f, y: %+2.5f, z:%+2.5f\n",
//                    gyro_dps[0],
//                    gyro_dps[1],
//                    gyro_dps[2]);
//
//            printf("mag.   x: %+2.5f, y: %+2.5f, z:%+2.5f\n",
//                    mag_ut[0],
//                    mag_ut[1],
//                    mag_ut[2]);


#else

            float output_data[3] = {0.f};

//            icm_read_gyro_y(output_data);
            icm_read_heading_xy(output_data);


#endif

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
//            printf("\n\n");
    #endif
            local_counter++;

            sequence_counter = 0;
        }
#endif // #ifdef TASK_7_FLAG

    }

} // void main(void) end ------------------------------------------------------

