/**
 * @file LPUART8_Config.h
 * @brief Build-time configuration for the Arduino/Teensy RPLiDAR C1 driver.
 *
 * @details This replaces Project_Config.h from the MSP432 build. All defines
 *  here are safe to override from platformio.ini build_flags.
 *
 * @note To enable debug output add -DRPLIDAR_DEBUG to build_flags.
 *       To enable Processing4 visualiser output add -DPROCESSING4_OUTPUT.
 */

#ifndef __LPUART8_CONFIG_H__
#define __LPUART8_CONFIG_H__


// ----------------------------------------------------------------------------
//
//  BAUD RATE CONFIGURATION
//
// ----------------------------------------------------------------------------

/**
 * @brief LPUART8 fixed  baud rate (460,800 bps).
 */
#ifndef LPUART8_BAUD_RATE
#define LPUART8_BAUD_RATE    460800
#endif


// ----------------------------------------------------------------------------
//
//  SCAN DECIMATION CONSTANTS
//
// ----------------------------------------------------------------------------

/**
 * @brief Onboard decimation factor — every Nth 5-byte message is kept.
 *
 * @note  Must be >= 2.  Value of 4 gives full angle coverage at 100 points/
 *        scan on an RPLiDAR C1 running at its standard ~4 000 samples/s.
 */
// #ifndef DECIMATION_FACTOR
// #define DECIMATION_FACTOR   4
// #endif

// #if DECIMATION_FACTOR < 2
// #error "DECIMATION_FACTOR must be >= 2"
// #endif


/**
 * @brief Output downsampling applied after acquisition (1 = no downsampling).
 */
// #ifndef SKIP_FACTOR
// #define SKIP_FACTOR     1
// #endif


// ----------------------------------------------------------------------------
//
//  BUFFER GEOMETRY
//
// ----------------------------------------------------------------------------

/**
 * @brief   Raw 5-byte RPLiDAR message length
 */
#define MSG_LENGTH              5

/**
 * @brief   Number of bytes consumed during the pattern-search phase  
 */
#define FIND_INDEX              (MSG_LENGTH * 4)

/**
 * @brief   Number of bytes skipped per decimation cycle
 */
// #define SKIP_INDEX              (MSG_LENGTH * (DECIMATION_FACTOR - 1))

/**
 * @brief   Number of bytes consumed during the initial hold phase
 */
// #define WAIT_INDEX              (MSG_LENGTH * 8)

/**
 * @brief   Intermediary (angle-distance) buffer size
 */
// #define PROCESS_BUFFER_SIZE     (SKIP_FACTOR * OUTPUT_BUFFER)




#endif /* __LPUART8_CONFIG_H__ */
