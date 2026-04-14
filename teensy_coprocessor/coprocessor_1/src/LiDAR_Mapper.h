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

#include "Timer_Tasks.h"

#define MSP432_Serial  Serial5


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
