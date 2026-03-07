/**
 * @file rplidar_impl.h
 * @brief Teensy test harness for the RPLiDAR C1 Arduino driver.
 *
 * @details This file mirrors the floating_point_impl.h pattern: the entire
 *  implementation lives here and main.cpp switches into it via a single
 *  preprocessor guard.  It exercises the new RPLiDAR_C1 library end-to-end on
 *  the Teensy without touching any other subsystems.
 *
 *  Hardware wiring assumed:
 *    Teensy pin 0  (RX1)  ←  RPLiDAR C1 TX
 *    Teensy pin 1  (TX1)  →  RPLiDAR C1 RX
 *    3.3 V / GND as usual; motor driver power separate
 *
 *  Build in platformio.ini:
 *    [env:teensy40]  or  [env:teensy41]
 *    build_flags = -DRPLIDAR_IMPL
 *
 * @note The byte feed is implemented as a polling loop in loop() rather than
 *  via serialEvent1() so that the data path is fully explicit during
 *  bring-up.  Switching to a serialEvent is a one-line change once confirmed
 *  working (see comment in loop()).
 *
 * @author Gian Fajardo
 */

#pragma once

#include <Arduino.h>

#include <RPLiDAR_C1.h>

#include "Timer_A1_Tasks.h"

#define PROCESSING4_OUTPUT 1


// ----------------------------------------------------------------------------
//  Configuration
// ----------------------------------------------------------------------------

/**
 * @brief Hardware serial port wired to the RPLiDAR C1.
 *        Teensy 4.x Serial1 = pins 0 (RX) / 1 (TX).
 */
#define RPLIDAR_Serial  Serial1

/**
 * @brief How many points to echo to the USB Serial monitor each frame.
 *        Set to 0 to suppress per-point output entirely.
 */
#define RPLIDAR_PRINT_POINTS    5


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
 * @brief Frame counter for simple rate monitoring.
 */
static uint32_t rplidar_frame_count = 0;


// ----------------------------------------------------------------------------
//  Re-arm helper
// ----------------------------------------------------------------------------

/**
 * @brief Reset the acquisition FSM to READY after a PROCESSING frame.
 *
 * @details Call immediately after Process_RPLiDAR_Data() returns.
 *  End_Record() (called internally when the buffer fills) leaves
 *  limit_status == HOLD and buffer_pointer == RX_POINTER; we just need to
 *  reset the byte counter and transition current_state back to READY.
 */
static inline void RPLiDAR_ReArm(void)
{
    rplidar_cfg.isr_counter   = 0;
    rplidar_cfg.current_state = READY;
    rplidar_cfg.limit_status  = HOLD;
    // buffer_pointer is already == RX_POINTER (set by End_Record)
    // interm_buffer_pointer and counter are reset by Find_Pattern_Action
    //   on the next acquisition cycle
}

IntervalTimer loop_timer;

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
    // USB CDC — wait for monitor so initialisation messages aren't lost
    Serial.begin(115200);
    while (!Serial);

    Serial.println("==============================================");
    Serial.println("  RPLiDAR C1 — Teensy Arduino port test");
    Serial.println("==============================================");

    // Bind Serial1 to the RPLiDAR driver ---------------------------------
    RPLiDAR_UART_SetPort(&RPLIDAR_Serial);

    

    loop_timer.begin(Task_Selector, LOOP_INTERVAL_MS * 1000); // convert ms to us

    // Initialise scanner:
    //  - Configure_RPLiDAR_Struct(&rplidar_cfg)
    //  - RPLiDAR_UART_Init()   → Serial1.begin(460800)
    //  - STOP → RESET → GET_HEALTH → SCAN
    Serial.println("[1/2] Initialising RPLiDAR C1...");
    Initialize_RPLiDAR_C1(&rplidar_cfg);

    // Arm the acquisition FSM (IDLING → READY)
    Serial.println("[2/2] Arming FSM...");
    Start_Record(NULL);     // NULL → Scan_All (accept every angle)

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
    // WaitForInterrupt();  // sleep until the next loop interval timer interrupt


    // -------------------------------------------------------------------------
    // Feed incoming bytes into the acquisition FSM.
    //
    // Polling approach — explicit and easy to debug during bring-up.
    //
    // Alternative once confirmed working:
    //   Remove this while-loop and add instead:
    //
    //   void serialEvent1() {
    //       while (Serial1.available())
    //           RPLiDAR_ProcessByte(Serial1.read());
    //   }
    //
    //   The serialEvent1 callback is automatically invoked by the Arduino
    //   framework after each loop() return when bytes are available.
    // -------------------------------------------------------------------------

    while (RPLIDAR_Serial.available())
        RPLiDAR_ProcessByte((uint8_t)RPLIDAR_Serial.read());


    // -------------------------------------------------------------------------
    // Check if a complete scan frame has been captured
    // -------------------------------------------------------------------------

    if (rplidar_cfg.current_state == PROCESSING) {

        Process_RPLiDAR_Data(&rplidar_cloud);

        rplidar_frame_count++;

        // --- Summary line ------------------------------------------------------
        // Serial.printf("Frame %4u | %3u pts\n",
        //               rplidar_frame_count,
        //               rplidar_cloud.num_pts);

        

        printf("POSE,%5.2f,%5.2f,%5.2f\n",
        //         global_pose.x,
        //         global_pose.y,
        //         global_pose.theta);
                0,
                0,
                0);

        printf("SCAN_START\n");

        // --- Optional per-point sample -----------------------------------------
#if RPLIDAR_PRINT_POINTS > 0
        uint32_t print_n = min((uint32_t)RPLIDAR_PRINT_POINTS,
                               rplidar_cloud.num_pts);

        for (uint32_t i = 0; i < print_n; i++) {
            // Serial.printf("  [%3u]  x = %8.2f mm   y = %8.2f mm\n",
            //               i,
            //               rplidar_cloud.points[i].x,
            //               rplidar_cloud.points[i].y);

    #ifdef PROCESSING4_OUTPUT
            Serial.printf("P,%5.2f,%5.2f\n",
                          rplidar_cloud.points[i].x,
                          rplidar_cloud.points[i].y);
    #endif
        }

    #ifdef PROCESSING4_OUTPUT
        printf("SCAN_END\n");
    #endif

#endif

        // --- Re-arm for next frame ---------------------------------------------
        RPLiDAR_ReArm();
    }
}
