/*
 * I2C.h
 *
 *  Created on: Mar 31, 2025
 *      Author: thatg
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "msp.h"

void config_EUSCI_B1_I2C(void);

uint8_t I2C_send_byte(
        uint8_t  slave,
        uint8_t *data);

uint8_t I2C_send_bytes(
        uint8_t  slave,
        uint8_t *data,
        uint8_t  num_bytes);

uint8_t I2C_receive_byte(
        uint8_t  slave,
        uint8_t *data);

uint8_t I2C_receive_bytes(
        uint8_t slave,
        uint8_t *data,
        uint8_t num_bytes);


#endif /* INC_I2C_H_ */
