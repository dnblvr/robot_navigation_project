/*
 * RPLiDAR_C1.h
 *
 */

#ifndef INC_RPLIDAR_C1_H_
#define INC_RPLIDAR_C1_H_


#include <stdint.h>
#include "RPLiDAR_A2_UART.h"
#include "coordinate_transform.h"
#include "Clock.h"



/**
 * @brief Command definitions for the RPLiDAR C1
 * @details These commands are used to control the RPLiDAR C1 and retrieve
 *          information.
 */

// Single-Request, No-Response
extern const uint16_t      STOP[3],
                          RESET[3];

// Single-Request, Single-Response
extern const uint8_t   GET_INFO[2],
                     GET_HEALTH[2],
                 GET_SAMPLERATE[2],
                 GET_LIDAR_CONF[2],

// Single-Request, Multiple-Response
                           SCAN[2],
                   EXPRESS_SCAN[2];


/**
 * @brief Initialize the RPLiDAR C1.
 *
 * This function initializes the RPLiDAR C1 by sending the appropriate commands
 *      to the device and configuring the UART A2 interface.
 *
 * @return None
 */
void initialize_RPLiDAR_C1(
        RPLiDAR_Config *config,
        uint8_t        *RPLiDAR_RX_Data);


/**
 * @brief
 */
void process_rplidar_data(
        uint8_t RX_DATA[OUTPUT_BUFFER],
        float   out[OUTPUT_BUFFER][3]);


#endif /* INC_RPLIDAR_C1_H_ */
