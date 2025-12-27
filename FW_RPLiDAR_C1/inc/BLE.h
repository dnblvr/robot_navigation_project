/*
 * BLE.h
 *
 */

#ifndef __INC_BLE_H__
#define __INC_BLE_H__


#include <stdio.h>
#include <string.h>
#include "GPIO.h"
#include "Motor.h"
#include "BLE_A3_UART.h"


// Initialize length of the tachometer buffers
#define TACHOMETER_BUFFER_LEN         10

// Set the maximum RPM for both wheels
//#define MAX_RPM                       120

// Set the minimum RPM for both wheels
//#define MIN_RPM                       30

// Desired RPM for the left wheel
uint16_t Desired_RPM_Left;

// Desired RPM for the right wheel
uint16_t Desired_RPM_Right;

// Declare a global variable used to store the measured RPM by the left
// tachometer
//uint16_t Actual_RPM_Left            = 0;

// Declare a global variable used to store the measured RPM by the right
// tachometer
//uint16_t Actual_RPM_Right           = 0;

// Set initial duty cycle of the left wheel to 25%
//uint16_t Duty_Cycle_Left            = 3750;

// Set initial duty cycle of the right wheel to 25%
//uint16_t Duty_Cycle_Right           = 3750;


//int begin_state;


/**
 * @brief Motor speed settings
 */
uint16_t forward_speed  = 500, // forward duty cycle
         backward_speed = 300, // reverse duty cycle
         rotation_speed = 300; // rotation duty cycle

/**
 * @brief MotorCommand function pointer type
 *
 * @param[in] left_speed    The speed of the left motor.
 * @param[in] right_speed   The speed of the right motor.
 */
typedef void (*MotorCommand)(uint16_t left_speed, uint16_t right_speed);


/**
 * @brief   Wrapper function to stop the motors. In order to offer compatibility with the
 *      MotorCommand function prototype, the parameters are unused.
 *
 * @param[in] left_speed    The speed of the left motor.
 * @param[in] right_speed   The speed of the right motor.
 */
void Motor_Stop_Wrapper(uint16_t left_speed, uint16_t right_speed);


/**
 * @brief processes UART feed based on if they're similar to
 *
 * @param BLE_UART_Buffer
 */
void Process_BLE_UART_Data(volatile char BLE_UART_Buffer[]);


#endif /* __INC_BLE_H__ */
