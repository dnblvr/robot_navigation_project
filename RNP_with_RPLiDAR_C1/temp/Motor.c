/**
 * @file Motor.c
 * @brief Source code for the Motor driver.
 *
 * This file contains the function definitions for controlling the DC motors using Pulse Width Modulation (PWM).
 * It provides functions for initializing the motor driver, controlling motor movement in various directions,
 * adjusting motor speed with PWM, and stopping the motors.
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/Motor.h"

void Motor_Init()
{
    // Configure the following pins as output GPIO pins: P5.4 and P5.5
    // by clearing Bits 4 and 5 of the SEL0 and SEL1 registers for P5
    // and setting Bits 4 and 5 of the DIR register for P5
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;

    // Initialize the output of the following pins to 0: P5.4 and P5.5
    // by clearing Bits 4 and 5 of the OUT register for P5
    P5->OUT &= ~0x30;

    // Configure the following pins as output GPIO pins: P3.6 and P3.7
    // by clearing Bits 6 and 7 of the SEL0 and SEL1 registers for P3
    // and setting Bits 6 and 7 of the DIR register for P3
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;

    // Initialize the output of the following pins to 0: P3.6 and P3.7
    // by clearing Bits 6 and 7 of the OUT register for P3
    P3->OUT &= ~0xC0;

    // Initialize Timer A0 with a period of 20 ms using a period constant of 15000
    Timer_A0_PWM_Init(TIMER_A0_PERIOD_CONSTANT, 0, 0);
}

void Motor_Forward(uint16_t left_duty_cycle, uint16_t right_duty_cycle)
{
    // Configure the motors to move in a forward direction
    // by clearing Bits 4 and 5 of the OUT register for P5
    P5->OUT &= ~0x30;

    // Update the duty cycle for both motors
    Timer_A0_Update_Duty_Cycle_1(right_duty_cycle);
    Timer_A0_Update_Duty_Cycle_2(left_duty_cycle);

    // Enable the motors by setting Bits 6 and 7 of the OUT register for P3
    P3->OUT |= 0xC0;
}

void Motor_Backward(uint16_t left_duty_cycle, uint16_t right_duty_cycle)
{
    // Configure the motors to move in a forward direction
    // by setting Bits 4 and 5 of the OUT register for P5
    P5->OUT |= 0x30;

    // Update the duty cycle for both motors
    Timer_A0_Update_Duty_Cycle_1(right_duty_cycle);
    Timer_A0_Update_Duty_Cycle_2(left_duty_cycle);

    // Enable the motors by setting Bits 6 and 7 of the OUT register for P3
    P3->OUT |= 0xC0;
}

void Motor_Left(uint16_t left_duty_cycle, uint16_t right_duty_cycle)
{
    // Configure the left motor to move in a backward direction
    // by setting Bit 4 of the OUT register for P5
    P5->OUT |= 0x10;

    // Configure the right motor to move in a forward direction
    // by clearing Bit 5 of the OUT register for P5
    P5->OUT &= ~0x20;

    // Update the duty cycle for both motors
    Timer_A0_Update_Duty_Cycle_1(right_duty_cycle);
    Timer_A0_Update_Duty_Cycle_2(left_duty_cycle);

    // Enable the motors by setting Bits 6 and 7 of the OUT register for P3
    P3->OUT |= 0xC0;
}

void Motor_Right(uint16_t left_duty_cycle, uint16_t right_duty_cycle)
{
    // Configure the left motor to move in a forward direction
    // by clearing Bit 4 of the OUT register for P5
    P5->OUT &= ~0x10;

    // Configure the right motor to move in a backward direction
    // by setting Bit 5 of the OUT register for P5
    P5->OUT |= 0x20;

    // Update the duty cycle for both motors
    Timer_A0_Update_Duty_Cycle_1(right_duty_cycle);
    Timer_A0_Update_Duty_Cycle_2(left_duty_cycle);

    // Enable the motors by setting Bits 6 and 7 of the OUT register for P3
    P3->OUT |= 0xC0;
}

void Motor_Stop()
{
    // Disable the motors by clearing Bits 6 and 7 of the OUT register for P3
    // and clearing Bits 4 and 5 of the OUT register for P5
    P3->OUT &= ~0xC0;
    P5->OUT &= ~0x30;

    // Update the duty cycle for both motors to 0%
    Timer_A0_Update_Duty_Cycle_1(0);
    Timer_A0_Update_Duty_Cycle_2(0);
}
