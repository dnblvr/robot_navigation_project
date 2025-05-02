/**
 * @file Motor.h
 * @brief Header file for the Motor driver.
 *
 * This file contains the function definitions for controlling the DC motors using Pulse Width Modulation (PWM).
 * It provides functions for initializing the motor driver, controlling motor movement in various directions,
 * adjusting motor speed with PWM, and stopping the motors.
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/Timer_A0_PWM.h"

/**
 * @brief
 *
 * @param None
 *
 * @return None
 */
void Motor_Init();

/**
 * @brief Moves the motors forward with specified duty cycles.
 *
 * This function configures the motors to move in a forward direction by setting the appropriate GPIO pins.
 * It also updates the duty cycle for both left and right motors using Timer A0 PWM control to adjust motor speed.
 *
 * @param left_duty_cycle The duty cycle for the left motor (0-99%).
 *
 * @param right_duty_cycle The duty cycle for the right motor (0-99%).
 *
 * @return None
 */
void Motor_Forward(uint16_t left_duty_cycle, uint16_t right_duty_cycle);

/**
 * @brief Move the motors backward with specified duty cycles.
 *
 * This function configures both motors to move backward. It updates the duty cycle for both left
 * and right motors using Timer A0 PWM control to adjust motor speed.
 *
 * @param left_duty_cycle The duty cycle for the left motor (0-99%).
 *
 * @param right_duty_cycle The duty cycle for the right motor (0-99%).
 *
 * @return None
 */
void Motor_Backward(uint16_t left_duty_cycle, uint16_t right_duty_cycle);

/**
 * @brief
 *
 * @param left_duty_cycle
 *
 * @param right_duty_cycle
 *
 * @return None
 */
void Motor_Left(uint16_t left_duty_cycle, uint16_t right_duty_cycle);

/**
 * @brief
 *
 * @param left_duty_cycle
 *
 * @param right_duty_cycle
 *
 * @return None
 */
void Motor_Right(uint16_t left_duty_cycle, uint16_t right_duty_cycle);

/**
 * @brief Stop the motors and set the duty cycle to 0%.
 *
 * This function disables both motors, effectively stopping them, and sets the duty cycle for both motors to 0%.
 *
 * @return None
 */
void Motor_Stop();

#endif /* INC_MOTOR_H_ */
