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



/**
 * @brief   Number of bytes consumed during the pattern-search phase  
 */
#define UART8_BUFFER_SIZE   64


#endif /* __LPUART8_CONFIG_H__ */
