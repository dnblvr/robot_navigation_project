/**
 * @file RPLiDAR_Config.h
 * @brief Build-time configuration for the Arduino/Teensy RPLiDAR C1 driver.
 *
 * @details This replaces Project_Config.h from the MSP432 build. All defines
 *  here are safe to override from platformio.ini build_flags.
 *
 * @note To enable debug output add -DRPLIDAR_DEBUG to build_flags.
 *       To enable Processing4 visualiser output add -DPROCESSING4_OUTPUT.
 */

#ifndef __RPLIDAR_CONFIG_H__
#define __RPLIDAR_CONFIG_H__


// ----------------------------------------------------------------------------
//  Baud rate
// ----------------------------------------------------------------------------

/**
 * @brief RPLiDAR C1 fixed UART baud rate (460 800 bps).
 */
#ifndef RPLIDAR_BAUD
#define RPLIDAR_BAUD    460800
#endif


// ----------------------------------------------------------------------------
//  Scan decimation
// ----------------------------------------------------------------------------

/**
 * @brief Onboard decimation factor — every Nth 5-byte message is kept.
 *
 * @note  Must be >= 2.  Value of 4 gives full angle coverage at 100 points/
 *        scan on an RPLiDAR C1 running at its standard ~4 000 samples/s.
 */
#ifndef DECIMATION_FACTOR
#define DECIMATION_FACTOR   4
#endif

#if DECIMATION_FACTOR < 2
#error "DECIMATION_FACTOR must be >= 2"
#endif


/**
 * @brief Output downsampling applied after acquisition (1 = no downsampling).
 */
#ifndef SKIP_FACTOR
#define SKIP_FACTOR     1
#endif


// ----------------------------------------------------------------------------
//  Buffer geometry (derived — not intended to be overridden)
// ----------------------------------------------------------------------------

/**  Raw 5-byte RPLiDAR message length  */
#define MSG_LENGTH              5

/**  Number of bytes consumed during the pattern-search phase  */
#define FIND_INDEX              (MSG_LENGTH * 4)

/**  Number of bytes skipped per decimation cycle  */
#define SKIP_INDEX              (MSG_LENGTH * (DECIMATION_FACTOR - 1))

/**  Number of bytes consumed during the initial hold phase  */
#define WAIT_INDEX              (MSG_LENGTH * 8)

/**  Intermediary (angle-distance) buffer size  */
#define PROCESS_BUFFER_SIZE     (SKIP_FACTOR * OUTPUT_BUFFER)


// ----------------------------------------------------------------------------
//  Debug / visualiser toggles
// ----------------------------------------------------------------------------

// #define RPLIDAR_DEBUG       1    ///< verbose protocol tracing
// #define PROCESSING4_OUTPUT  1    ///< Processing 4 serial output format
// #define DEBUG_OUTPUT        1    ///< general debug printf output

#if defined(PROCESSING4_OUTPUT) && defined(DEBUG_OUTPUT)
#error "PROCESSING4_OUTPUT and DEBUG_OUTPUT cannot be active at the same time."
#endif


#endif /* __RPLIDAR_CONFIG_H__ */
