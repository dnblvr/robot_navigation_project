
#ifndef __MOTOR_COMMANDS_H__
#define __MOTOR_COMMANDS_H__

#include "rrt_star.h"
#include "Motor.h"

#include "./inc/motor_commands.h"


#include <stdint.h>


/*
void Motor_Init();
void Motor_Forward(uint16_t left_duty_cycle, uint16_t right_duty_cycle);
void Motor_Backward(uint16_t left_duty_cycle, uint16_t right_duty_cycle);
void Motor_Left(uint16_t left_duty_cycle, uint16_t right_duty_cycle);
void Motor_Right(uint16_t left_duty_cycle, uint16_t right_duty_cycle);
void Motor_Stop();
*/

/**
 * @brief this parameter is used to set the maximum number of operations in the motor_commands array.
 */
#define MAXIMUM_OPERATIONS 40

/**
 * @brief this chooses the appropriate motor function based on the duty cycle.
 * 
 * @param duty_cycle if negative, turn left; if positive, turn right
 */
void Motor_Turn(int16_t duty_cycle);

/**
 * @brief this function moves the motors forward or backward based on the duty cycle.
 * 
 * @param duty_cycle 
 */
void Motor_Move(int16_t duty_cycle);

/**
 * @brief time required to turn 180 degrees.
 */
const uint32_t TIME_180_DEGREES = 1000; // time in ms to turn 180 degrees

/**
 * @brief this structure represents the motor commands for a robot tailored to the motor functions in Motor.c.
 */
typedef struct {
	void   	   *Motor_Function;
	uint16_t   	PWM_Motor_Speed;
	uint32_t	time_distance;
} MotorCommands;

/**
 * @brief this array contains the motor commands for the robot.
 * 
 * @note the first element is a pointer to the function, the second element is the PWM speed, and the third element is the time distance.
 */
MotorCommands motor_commands[2*MAXIMUM_OPERATIONS] = {0};



// void 


#endif // __MOTOR_COMMANDS_H__
