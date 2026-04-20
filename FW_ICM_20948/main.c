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


// state estimator header
#include "inc/inEKF_se2.h"


void I2C_Task(void) {
    printf("hi\n");

}




// ----------------------------------------------------------------------------
//
//  STATE ESTIMATOR / COMMUNICATIONS
//
// ----------------------------------------------------------------------------

/**
 *
 */
#define FLAG(n)     (0x01 << n)

/**
 * @brief flag variable that requests the state of the communication switch
 */
#define ECHO_REQUEST_FLAG   (0x01 << 0)

/**
 * @brief flag variable that requests the state of the communication switch
 */
#define STATE_REQUEST_FLAG  (0x01 << 1)

/**
 * @brief flag variable that syncs the timer
 */
#define TIMER_RESET_FLAG    (0x01 << 2)

/**
 * @brief flag variable that establishes the state of the switch
 *
 * @note if 0b0001, EUSCIA2 is established
 * @note if 0b0010, EUSCIA2 received a send and
 */
volatile uint8_t comms_state = 0;


/**
 * @brief EUSCIA2 RXIFG decision-handler function
 *
 *
 *
 * @param[in] UART_Buffer buffer holding the byte buffer
 */
void Handle_UART_Communications(volatile char UART_Buffer[]) {

    // if seen, communication is established. then echo back
    if (Check_UART_Data(UART_Buffer, "!E")) {

        // echo
        UART_A2_OutString("!E\r\n");

        comms_state    |=  ECHO_REQUEST_FLAG;


    // state command was received. This accumulates the pose and defers
    // command operation to timer-based interrupts to prevent race conditions
    } else if (Check_UART_Data(UART_Buffer, "!S")) {

//        printf("at uart h\n");

        // let the timer task receive the okay to
        comms_state    |=  STATE_REQUEST_FLAG;


    } else if (Check_UART_Data(UART_Buffer, "!R")) {

        // let the timer task receive the okay to
        comms_state    |=  TIMER_RESET_FLAG;


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


    // Distance between wheels in mm
    #define WHEEL_BASE_MM 141.0f




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

// Encoder count to millimeter conversion
#define COUNTS_TO_DIST  0.6111111111

// Distance between wheels in mm
#define WHEEL_BASE_MM   141.0f


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


    // Initialize the tachometers as a function of Timer A3
    Tachometer_Init();


    // Initialize the DC motors
    Motor_Init();


    /** ----------------------------------------------------------------------
     * Initialize MSP432 --> T4.0 / i.MX RT1062 communications
     */

    UART_A2_Init(&Handle_UART_Communications);

    while (!comms_established) {
        EUSCI_A2->IE   |=  0x0001;  // RPLiDAR C1

    }


    LED2_Output(RGB_LED_RED);

    return;

    /** -----------------------------------------------------------------------
     * @note Initialize the ICM20948
     */

#ifdef OLD_SYSTEM
    
    EUSCI_B1_I2C_Init();

    icm20948_config_t config = {.addr_accel_gyro    = ICM20948_ADDR_ACCEL_GYRO,
                                .addr_mag           = ICM20948_ADDR_MAG};

    
    if (icm20948_init(&config) == 0)
        printf("ICM-20948 successfully initialized!\n");
    else
        printf("ICM-20948 unsuccessfully initialized!\n");


    int16_t gyro_raw[3]	= {0},
            mag_raw[3] 	= {0};

    float   gyro_dps[3]	= {0},
            mag_ut[3]  	= {0};

#else

    // Initialize the ICM20948 with interrupt from pin at P4.6
    EUSCI_B1_I2C_Init(&I2C_Task);

    if (icm20948_init(ICM20948_ADDR_ACCEL_GYRO_0,
                      ICM20948_ADDR_MAG) == 0)
        printf("ICM-20948 successfully initialized!\n");
    else
        printf("ICM-20948 unsuccessfully initialized!\n");

#endif

    /** -----------------------------------------------------------------------
     * @note state estimator instantiation
     *
     *   dt            = 1/20 s  (TASK_1 rate)
     *   process_noise = {10 mm, 10 mm, 0.05 rad) intentionally loose to let
     *                   covariances converge before measurement updates
     *                   tighten them
     *   mag_noise     = 0.05 rad  (~3 deg std dev)
     *   chi2_threshold = 3.84  (95 % chi-squared gate, 1 DOF)
     *   flt.L         = WHEEL_BASE_MM overrides hardcoded 0.3 m in init
     */

    InEKF_SE2_t flt;

    {
        state_se2_t proc_noise = {10.0f, 10.0f, 0.05f};
        inEKF_SE2_init(&flt,
                       1.0f/20.0f,
                       WHEEL_BASE_MM,
                       &proc_noise,
                       0.05f,
                       3.84f);
    }



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



    // EUSCI_A0->IE   |=  0x0003;  // printf()
     EUSCI_A2->IE   |=  0x0001;  // to Teensy UART
    // EUSCI_A3->IE   |=  0x0003;  // BLE UART Friend
    
//    Set_All_Interrupts_1();



#ifdef DEBUG_OUTPUT
    printf("before\n");
#endif


    // receive echo sequence again and proceed and color the LED blue
//    Wait_Until_Condition(TIMER_RESET_FLAG);
    // when echo sequence `!E\r\n` is detected, proceed and color the LED red
    Wait_Until_Condition(ECHO_REQUEST_FLAG);


    printf("after\n");


    // use Timer_A1_Interrupt to get the odometry measurements resulting
    // output will be measured in increments of 10 ticks per second
    Timer_A1_Interrupt_Init(&Timer_A1_Task_Selector,
                            (uint16_t)(TEN_TICKS_PER_SEC/2));
                            
                            
    LED2_Output(RGB_LED_BLUE);



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
         * @note TASK 0: none
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
         * @note TASK 1: access ICM20948 gyro and wheel encoders at 20 times
         * a sec to accumulate incremental pose changes
         */
        if (task_flag & TASK_1_FLAG) {
            task_flag  &= ~TASK_1_FLAG;

            // Get current tachometer readings
            Tachometer_Get_Steps(&Left_Steps, &Right_Steps);

#ifdef OLD_SYSTEM

            // counter variable
            uint32_t i;

            // Read full gyro frame; extract Z-axis yaw rate (rad/s)
            icm20948_read_raw_gyro(&config, gyro_raw);
            for (i = 0; i < 3; i++) {
                gyro_dps[i] = (float)gyro_raw[i] / 131.0f;
            }

            float omega_gyro = gyro_dps[2] * (M_PI_F / 180.0f);

#else

            // Full gyro frame not yet wired in this branch;
            // placeholder until icm_read_gyro_y (Z-axis) is available.
            // alpha=0.5 stays — encoder-only omega path remains active.
            float omega_gyro = 0.0f;

#endif


            // Compute per-step wheel velocities in mm/s from encoder deltas
            {
                static int32_t prev_L_steps = 0, prev_R_steps = 0;

                float delta_L_mm = COUNTS_TO_DIST * \
                                    (float)(Left_Steps  - prev_L_steps);
                float delta_R_mm = COUNTS_TO_DIST * \
                                    (float)(Right_Steps - prev_R_steps);
                prev_L_steps = Left_Steps;
                prev_R_steps = Right_Steps;

                float v_L = delta_L_mm * flt.inv_dt;
                float v_R = delta_R_mm * flt.inv_dt;

                // InEKF predict step
                inEKF_SE2_predict(&flt, v_L, v_R, omega_gyro);

#ifdef DEBUG_OUTPUT
//                printf("\tyaw\tv_l\tv_r\tstate\n");
                printf(
//                       "\t%+3.2f %+3.2f %+3.2f"
                       " %+3.2f, %+3.2f %+3.2f\n",
//                       omega_gyro,
//                       v_L,
//                       v_R,
                       flt.state.x,
                       flt.state.y,
                       flt.state.theta);
#endif
            }

        }

#endif


#ifdef TASK_2_FLAG

        /**
         * @note TASK 2: update the inEKF with a magnetometer heading measurement
         */
        if (task_flag & TASK_2_FLAG) {
            task_flag  &= ~TASK_2_FLAG;

#ifdef OLD_SYSTEM

            // Read full mag frame; derive heading (rad) and norm from x/y/z
            uint32_t i;
            icm20948_read_raw_mag(&config, mag_raw);
            for (i = 0; i < 3; i++) {
                mag_ut[i] = ((float)mag_raw[i] / 20) * 3;
            }

#ifdef DEBUG_OUTPUT
//            printf("mag.   x: %+2.5f, y: %+2.5f, z:%+2.5f\n",
//                    mag_ut[0], mag_ut[1], mag_ut[2]);
#endif

            {
                float psi_mag  = atan2f(mag_ut[1], mag_ut[0]);
                float mag_norm = sqrtf(   mag_ut[0]*mag_ut[0]
                                        + mag_ut[1]*mag_ut[1]
                                        + mag_ut[2]*mag_ut[2]);
                inEKF_SE2_update_mag(&flt, psi_mag, mag_norm);
            }

#else

            // Read heading frame (radians); mag_norm is a mid-gate placeholder
            // until a raw mag read can provide the computed norm.
            float heading_data[3] = {0.f};
            icm_read_heading_xy(heading_data);
            inEKF_SE2_update_mag(&flt, heading_data[0], 4000.0f);

#endif

        }

#endif


#ifdef TASK_3_FLAG

        /**
         * @note TASK 3: deferred !S pose-transmit worker.
         *
         *   Runs at 10 Hz, always after TASK_2 in this loop's priority order,
         *   so the mag update is already applied before we ship the pose.
         *   When Handle_UART_Communications sets comms_state bit 1 (!S received),
         *   this task transmits the current absolute SE(2) state {x, y, theta}
         *   as 12 raw little-endian bytes over UART A2. The Teensy receives
         *   sequential absolute poses and computes per-step deltas itself.
         */
        if (    (task_flag & TASK_3_FLAG)
             && 1)
        {

            printf("am here; %1u\n",
                   (comms_state & STATE_REQUEST_FLAG) ? 1 : 0);


            if (comms_state & STATE_REQUEST_FLAG) {


                task_flag   &= ~TASK_3_FLAG;
                comms_state &= ~STATE_REQUEST_FLAG;


                // transmit absolute SE(2) state {x, y, theta}: 12 raw bytes
                uint32_t    i;
                uint8_t*    p = (uint8_t *)&flt.state;

                UART_A2_OutChar('#');
                UART_A2_OutChar('S');

//                for (i = 0; i < 3u * sizeof(float); i++)
                for (i = 0; i < sizeof(flt.state); i++)
                    UART_A2_OutChar(p[i]);
            }

                
        }

#endif



#ifdef TASK_6_FLAG
        /**
         * @note: TASK 6: print persistently
         */
        if (task_flag & TASK_6_FLAG) {
            task_flag  &= ~TASK_6_FLAG;

        #ifdef DEBUG_OUTPUT
            // printf("%5d %5d\n", local_counter, sequence_counter);
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

