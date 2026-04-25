/**
 * @file RPLiDAR_Config.h
 * @brief Build-time configuration for the Arduino/Teensy RPLiDAR C1 driver.
 *
 * @details This replaces Project_Config.h from the MSP432 build. All defines
 *  here are safe to override from platformio.ini build_flags.
 *
 * @note To enable debug output add -D RPLIDAR_DEBUG to build_flags.
 *       To enable Processing4 visualiser output add -D PROCESSING4_OUTPUT.
 */

#ifndef __RPLIDAR_CONFIG_H__
#define __RPLIDAR_CONFIG_H__


// ----------------------------------------------------------------------------
//
//  BAUD RATE CONFIGURATION
//
// ----------------------------------------------------------------------------

/**
 * @brief RPLiDAR C1 fixed UART baud rate (460,800 bps).
 */
#define RPLIDAR_BAUD    460800


// ----------------------------------------------------------------------------
//
//  SCAN DECIMATION CHARACTERISTICS
//
// ----------------------------------------------------------------------------

/**
 * @brief Onboard decimation factor — every Nth 5-byte message is kept.
 *
 * @note  Must be >= 2.  Value of 4 gives full angle coverage at 100 points/
 *        scan on an RPLiDAR C1 running at its standard ~4 000 samples/s.
 */
#define DECIMATION_FACTOR   4

#if DECIMATION_FACTOR < 2
#error "DECIMATION_FACTOR must be >= 2"
#endif


/**
 * @brief Output downsampling applied after acquisition (1 = no downsampling).
 */
#define SKIP_FACTOR     1


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
#define SKIP_INDEX              (MSG_LENGTH * (DECIMATION_FACTOR - 1))

/**
 * @brief   Number of bytes consumed during the initial hold phase
 */
#define WAIT_INDEX              (MSG_LENGTH * 8)

/**
 * @brief   Intermediary (angle-distance) buffer size
 */
#define PROCESS_BUFFER_SIZE     (SKIP_FACTOR * OUTPUT_BUFFER)


// ----------------------------------------------------------------------------
//
//  DEBUG
//
// ----------------------------------------------------------------------------

#if defined(PROCESSING4_OUTPUT) && defined(DEBUG_OUTPUT)
#error "PROCESSING4_OUTPUT and DEBUG_OUTPUT cannot be active at the same time."
#endif


#endif /* __RPLIDAR_CONFIG_H__ */
