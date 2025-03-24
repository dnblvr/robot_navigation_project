/**
 * @file Tachometer.c
 * @brief Source code for the Tachometer driver.
 *
 * This file contains the function definitions for interfacing with the tachometers.
 * The Tachometer driver initializes pins P5.0 and P5.2 as GPIO inputs which are used to determine
 * the direction of the motor rotation. In addition, it also initializes the Timer A3 module
 * to measure the speed of the motor rotation.
 *
 * The following pins are configured as input:
 *  - P10.4 (Right Encoder A)
 *  - P5.0 (Right Encoder B)
 *  - P10.5 (Left Encoder A)
 *  - P5.2 (Left Encoder B)
 *
 * @author Aaron Nanas
 */

#include "../inc/Tachometer.h"

// Global variable used to store the time of the previous rising edge for the right wheel
static uint16_t Tachometer_Previous_Right_Time;

// Global variable used to store the time of the current rising edge for the right wheel
static uint16_t Tachometer_Current_Right_Time;

// Global variable used to store the time of the previous rising edge for the left wheel
static uint16_t Tachometer_Previous_Left_Time;

// Global variable used to store the time of the current rising edge for the left wheel
static uint16_t Tachometer_Current_Left_Time;

// Global variable used to keep track of the number of steps.
// Increments with every step forward. Decrements with every step backward
static int Tachometer_Right_Steps = 0;

// Global variable used to keep track of the number of steps.
// Increments with every step forward. Decrements with every step backward
static int Tachometer_Left_Steps = 0;

enum Tachometer_Direction Tachometer_Right_Dir = STOPPED;
enum Tachometer_Direction Tachometer_Left_Dir = STOPPED;

void Tachometer_Right_Int(uint16_t current_time)
{
    // Store the time of the previous rising edge for the right wheel
    Tachometer_Previous_Right_Time = Tachometer_Current_Right_Time;

    // Store the time of the current rising edge for the right wheel
    Tachometer_Current_Right_Time = current_time;

    // Check if P5.0 (Right Encoder B) is low
    if ((P5->IN & 0x01) == 0)
    {
        // If the Right Encoder B is low, then the motor is moving backward
        Tachometer_Right_Steps = Tachometer_Right_Steps - 1;
        Tachometer_Right_Dir = REVERSE;
    }

    else
    {
        // If the Right Encoder B is high, then the motor is moving forward
        Tachometer_Right_Steps = Tachometer_Right_Steps + 1;
        Tachometer_Right_Dir = FORWARD;
    }
}

void Tachometer_Left_Int(uint16_t current_time)
{
    // Store the time of the previous rising edge for the left wheel
    Tachometer_Previous_Left_Time = Tachometer_Current_Left_Time;

    // Store the time of the current rising edge for the left wheel
    Tachometer_Current_Left_Time = current_time;

    // Check if P5.2 (Left Encoder B) is low
    if ((P5->IN & 0x04) == 0)
    {
        // If the Left Encoder B is low, then the motor is moving backward
        Tachometer_Left_Steps = Tachometer_Left_Steps - 1;
        Tachometer_Left_Dir = REVERSE;
    }

    else
    {
        // If the Left Encoder B is high, then the motor is moving forward
        Tachometer_Left_Steps = Tachometer_Left_Steps + 1;
        Tachometer_Left_Dir = FORWARD;
    }
}

void Tachometer_Init()
{
    // Configure the following pins as input GPIO pins: P5.0 (Right Encoder B) and P5.2 (Left Encoder B)
    // by clearing Bits 0 and 2 of the SEL0 and SEL1 registers for P5
    // and setting Bits 0 and 2 of the DIR register for P5
    P5->SEL0 &= ~0x05;
    P5->SEL1 &= ~0x05;
    P5->DIR &= ~0x05;

    // Initialize Timer A3 Capture
    Timer_A3_Capture_Init(&Tachometer_Right_Int, &Tachometer_Left_Int);
}

void Tachometer_Get(uint16_t *left_tach,
                    enum Tachometer_Direction *left_dir,
                    int32_t *left_steps,
                    uint16_t *right_tach,
                    enum Tachometer_Direction *right_dir,
                    int32_t *right_steps)
{
    *left_tach = (Tachometer_Current_Left_Time - Tachometer_Previous_Left_Time);
    *left_dir = Tachometer_Left_Dir;
    *left_steps = Tachometer_Left_Steps;
    *right_tach = (Tachometer_Current_Right_Time - Tachometer_Previous_Right_Time);
    *right_dir = Tachometer_Right_Dir;
    *right_steps = Tachometer_Right_Steps;
}

uint16_t Average_of_Buffer(uint16_t *buffer, int buffer_length)
{
    uint32_t buffer_sum = 0;
    uint16_t buffer_average = 0;

    for (int i = 0; i < buffer_length; i++)
    {
        buffer_sum = buffer_sum + buffer[i];
    }

    buffer_average = buffer_sum / buffer_length;

    return buffer_average;
}
