/*
 * RPLiDAR.h
 *
 *  Created on: Apr 28, 2025
 *      Author: thatg
 */

#ifndef INC_RPLIDAR_H_
#define INC_RPLIDAR_H_

#include "./inc/EUSCI_A3_UART.h"


void Single_Request_Single_Response(uint8_t *output, uint8_t *command);


#endif /* INC_RPLIDAR_H_ */
