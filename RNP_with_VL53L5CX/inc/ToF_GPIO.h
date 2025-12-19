/*
 * ToF_GPIO.h
 *
 *  Created on: Apr 2, 2025
 *      Author: thatg
 */

#ifndef INC_TOF_GPIO_H_
#define INC_TOF_GPIO_H_


//#include <stdint.h>
#include "msp.h"


/**
 * powers on the sensor by configuring LPn and I2C_RST
 * to initialize:
 *      LPn     -> P4.6
 *      I2C_RST -> P1.5
 */
void config_VL53LCX_pins(void);

void VL53L5CX_wake(void);

void VL53L5CX_sleep(void);

void VL53L5CX_reset(void);

void VL53L5CX_no_reset(void);


#endif /* INC_TOF_GPIO_H_ */
