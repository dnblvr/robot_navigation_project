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

    // Initialize scanner:
    //  - Configure_RPLiDAR_Struct(&rplidar_cfg)
    //  - RPLiDAR_UART_Init()   --> Serial1.begin(460800)
    //  - STOP --> RESET --> GET_HEALTH --> SCAN
    Serial.println("Initializing RPLiDAR C1...");
    Initialize_RPLiDAR_C1(&rplidar_cfg);

    // Arm the acquisition FSM (IDLING --> READY)
    Serial.println("Arming FSM...");
    Start_Record(NULL);     // NULL --> Scan_All function by default

    // Now that all TX commands are sent, flush TX and replace HardwareSerial's
    // LPUART6 vector with our bare-metal RX ISR.  Must happen AFTER init so
    // that HardwareSerial's TX-interrupt path is no longer needed.
    Serial.println("Attaching bare-metal LPUART6 RX ISR...");
    RPLiDAR_UART_AttachISR();

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
    // Serial.println("Entering loop()...");

    // Sleep until the next interrupt (LPUART6_RX_ISR or IntervalTimer).
    // The Cortex-M7 wfi instruction resumes as soon as any unmasked
    // interrupt fires, so byte processing latency is interrupt latency
    // rather than polling latency.
    WaitForInterrupt();

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

            rplidar_frame_count++;
            
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

#if RPLIDAR_PRINT_POINTS > 0
            uint32_t print_n = min((uint32_t)RPLIDAR_PRINT_POINTS,
                                   rplidar_cloud.num_pts);

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
#endif

            // ----------------------------------------------------------------
            // DEBUG STATS  (printed every frame under RPLIDAR_DEBUG)
            // ----------------------------------------------------------------
            // Each metric answers one diagnostic question:
            //   empty_fifo  — ISR fired but FIFO was already empty on entry
            //                 (RXEMPT on first read); should always be 0.
            //   find[0..4]  — how many times each byte-offset was used to
            //                 re-align the stream; normally only offset 0 fires.
            //   find_fail   — times no valid packet boundary was found in a
            //                 20-byte window; 0 = data stream is clean.
            //   bad_start   — times Record_Action byte-0 had wrong start bits;
            //                 any value > 0 means the FSM byte counter drifted.
            //   interm      — points stored before End_Record; expected ~100.
            // ----------------------------------------------------------------
#ifdef RPLIDAR_DEBUG
            // Serial.printf("[DBG F=%lu] pts=%u interm=%lu | "
            //               "empty_fifo=%lu | "
            //               "find[0..4]={%lu,%lu,%lu,%lu,%lu} fail=%lu | "
            //               "bad_start=%lu\n",
            //               rplidar_frame_count,
            //               rplidar_cloud.num_pts,
            //               dbg_last_interm,
            //               dbg_isr_empty_entry,
            //               dbg_find_offsets[0], dbg_find_offsets[1],
            //               dbg_find_offsets[2], dbg_find_offsets[3],
            //               dbg_find_offsets[4],
            //               dbg_find_fail,
            //               dbg_bad_start_bit);

            // Highlight frames that contain any anomaly
            if (
                //     invalid_pts         > 0
                // ||  dbg_bad_start_bit   > 0
                // ||  dbg_find_fail       > 0
                // ||  dbg_isr_empty_entry > 0
                dbg_bad_packet_valid
            )
            {
                Serial.printf(  "[ANOMALY F=%lu] bad_start=%lu "
                                "find_fail=%lu empty_fifo=%lu\n",
                                rplidar_frame_count,
                                dbg_bad_start_bit,
                                dbg_find_fail,
                                dbg_isr_empty_entry);
            }
            
            if (dbg_bad_packet_valid) {
                Serial.printf(  "[BAD PKT] %02X %02X %02X %02X %02X"
                                "  (start_bits=%u, expected 1 or 2)\n",
                                dbg_bad_packet[0], dbg_bad_packet[1],
                                dbg_bad_packet[2], dbg_bad_packet[3],
                                dbg_bad_packet[4],
                                dbg_bad_packet[0] & 0x03u);
            }
            
            RPLiDAR_ResetDebugStats();
#endif  // RPLIDAR_DEBUG

            // --- Re-arm for next frame -----------------------------------------
            RPLiDAR_ReArm();

        } // if (state == PROCESSING)

    } // if (task_flag & TASK_4_FLAG)
}
