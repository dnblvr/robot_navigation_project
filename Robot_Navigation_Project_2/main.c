
#define MAIN_1 1
//#define MAIN_2 1
//#define MAIN_3 1

#ifdef MAIN_1 // -----------------------------------------------------------------------


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/GPIO.h"
#include "inc/SysTick_Interrupt.h"


#include "inc/EUSCI_A3_UART.h"
#include "inc/RPLiDAR.h"



#define SMCLK_FREQ 48000000 // Example: SMCLK frequency (48 MHz)
#define TARGET_DELAY 1000   // Example: Delay of 1 second
#define TICK_PER_MS (SMCLK_FREQ / 1000) // Clock ticks per millisecond

volatile unsigned int counter = 0; // Global counter

void delay_ms(unsigned int ms) {

    // Use a software timer to delay for ms milliseconds
    counter = 0;
    while (counter < ms * TICK_PER_MS) {
        // Increment counter with each loop iteration
        counter++;
    }
}


uint8_t STOP [3]          = {0x25, 0, 0x80},
        RESET[3]          = {0x25, 0, 0xE5},
        SCAN [3]          = {0x20, 0, 0x85},

        // extended
        EXPRESS_SCAN[3]   = {0x82, 5, 0x22},

        FORCE_SCAN[3]     = {0x21, 0, 0x00},
        GET_INFO  [3]     = {0x50, 0, 0x84},
        GET_HEALTH[3]     = {0x52, 0, 0xF5},
        GET_SAMPLERATE[3] = {0x50, 0, 0xF7},
        GET_LIDAR_CONF[3] = {0x84, 0, 0xFC};



uint8_t RX_Buffer[BUFFER_LENGTH] = {};


int main(void)
{

    // stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;


    int buffer_idx = 0;


    // Initialize the 48 MHz Clock
    Clock_Init48MHz();


    // Initialize the built-in red LED and the RGB LEDs
    LED1_Init();
    LED2_Init();


    // Initialize the user buttons
    Buttons_Init();


    // Initialize the front and back LEDs on the chassis board
    Chassis_Board_LEDs_Init();


    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();


    // Initialize EUSCI_A0_UART
    EUSCI_A3_UART_Init();


    // Enable the interrupts used by the SysTick and Timer_A timers
//    EnableInterrupts();

    printf("test rig\n");
//    delay_ms(1000);


    for (buffer_idx = 0; buffer_idx < 256; buffer_idx++) {

        RX_Buffer[buffer_idx] = 0;
    }


//    printf("Single_Request_Single_Response() test\n");
    // GET_HEALTH[3]     = {0x50, 0, 0xF5},
    while (1) {
        Single_Request_Single_Response(&RX_Buffer[0], &GET_HEALTH[0]);
        delay(1);
    }


//    for (buffer_idx = 0; buffer_idx < 5; buffer_idx++) {
//
//        printf("0x%02X, ", RX_Buffer[buffer_idx]);
//    }



//    while (1)
//    {
//
//    }

    return 0;
}





#endif // MAIN_1


#ifdef MAIN_2 // -----------------------------------------------------------------------



/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file, part of the VL53L8CX Ultra Lite Driver,
* is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
*******************************************************************************/

/***********************************/
/*   VL53L5CX ULD basic example    */
/***********************************/
/*
* This example is the most basic. It initializes the VL53L5CX ULD, and starts
* a ranging to capture 10 frames.
*
* By default, ULD is configured to have the following settings :
* - Resolution 4x4
* - Ranging period 1Hz
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/GPIO.h"
#include "inc/SysTick_Interrupt.h"



//#include "inc/EUSCI_B0_I2C.h"
////#include "inc/EUSCI_B1_I2C.h"
//#include "inc/globals.h"
//#include "inc/platform.h"
//#include "inc/vl53l5cx_api.h"
//#include "inc/ToF_GPIO.h"



uint32_t Timer_A1_ms_elapsed    = 0;


// Initialize a global variable for SysTick to keep track of elapsed time in milliseconds
uint32_t SysTick_ms_elapsed = 0;

//void Timer_A1_Periodic_Task(void) {
//
//    Timer_A1_ms_elapsed++;
//    if (Timer_A1_ms_elapsed >= 200) {
//
//        P8->OUT ^= 0x40;
//        Timer_A1_ms_elapsed = 0;
//    }
//}

/**
 * @brief
 *
 * @param None
 *
 * @return None
 */
void SysTick_Handler(void)
{
    SysTick_ms_elapsed++;

    if (SysTick_ms_elapsed >= 500)
    {
        P8->OUT ^= 0x01;
        SysTick_ms_elapsed = 0;
    }
}




//void EUSCIB0_IRQHandler(void) {
//
//    if (EUSCI_B0->IFG &   0x0001) {
//        // clear the RXIFG
//        EUSCI_B0->IFG &= ~0x0001;
//
//        // perform task here
//        read_byte_counter++;
//    }
//}


int main(void)
{

     // stop watchdog timer
     WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;


     EUSCI_B0_I2C_Init();
 //     EUSCI_B1_I2C_Init();



     // configures the I2C_RST and LPn
     config_VL53L5CX_pins();
     VL53L5CX_wake();
     VL53L5CX_reset();
     VL53L5CX_no_reset();
//     VL53L5CX_sleep();



     // Initialize the 48 MHz Clock
     Clock_Init48MHz();


     // Initialize the built-in red LED and the RGB LEDs
     LED1_Init();
     LED2_Init();


     // Initialize the user buttons
 //    Buttons_Init();


     // Initialize the front and back LEDs on the chassis board
     Chassis_Board_LEDs_Init();


     // Initialize EUSCI_A0_UART
     EUSCI_A0_UART_Init_Printf();


     // Initialize the SysTick timer to generate periodic interrupts every 1 ms
     SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);


     // Enable the interrupts used by the SysTick and Timer_A timers
     EnableInterrupts();

    /*********************************/
    /*   VL53L5CX ranging variables  */
    /*********************************/

    uint8_t                 status, loop, isAlive, isReady, i;
    VL53L5CX_Configuration  Dev;            /* Sensor configuration */
    VL53L5CX_ResultsData    Results;        /* Results data from VL53L5CX */


    /*********************************/
    /*      Customer platform        */
    /*********************************/

    /* Fill the platform structure with customer's implementation. For this
    * example, only the I2C address is used.
    */



//    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    Dev.platform.address = 0x29;

    /* (Optional) Reset sensor toggling PINs (see platform, not in API) */
//    VL53L5CX_Reset_Sensor(&(Dev.platform));

    /* (Optional) Set a new I2C address if the wanted address is different
    * from the default one (filled with 0x20 for this example).
    */
//    status = vl53l5cx_set_i2c_address(&Dev, 0x20);


    /*********************************/
    /*   Power on sensor and init    */
    /*********************************/


    /* (Optional) Check if there is a VL53L5CX sensor connected */
    status = vl53l5cx_is_alive(&Dev, &isAlive);

    uint8_t condition = (!isAlive || status);
    if (condition) {
        printf("VL53L5CX not detected at requested address\n\tcondition =\t0x%02X\n", condition);
        return status;
    }
    else {
        printf("detected\n");
    }

    return 0; // stops here for testing purposes
    

    /* (Mandatory) Init VL53L5CX sensor */
    status = vl53l5cx_init(&Dev);
    if (status) {
        printf("VL53L5CX ULD Loading failed\n");
        return status;
    }

    printf("VL53L5CX ULD ready ! (Version : %s)\n",
            VL53L5CX_API_REVISION);


    /*********************************/
    /*         Ranging loop          */
    /*********************************/

    status = vl53l5cx_start_ranging(&Dev);

    loop = 0;
    while(loop < 10)
    {
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN A3
         * (GPIO 1) when a new measurement is ready */

        status = vl53l5cx_check_data_ready(&Dev, &isReady);

        if(isReady)
        {
            vl53l5cx_get_ranging_data(&Dev, &Results);

            /* As the sensor is set in 4x4 mode by default, we have a total
             * of 16 zones to print. For this example, only the data of first zone are
             * print */
            printf("Print data no : %3u\n", Dev.streamcount);
            for(i = 0; i < 16; i++)
            {
                printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
                    i,
                    Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*i],
                    Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*i]);
            }
            printf("\n");
            loop++;
        }

        /* Wait a few ms to avoid too high polling (function in platform
         * file, not in API) */
        VL53L5CX_WaitMs(&(Dev.platform), 5);
    }

    status = vl53l5cx_stop_ranging(&Dev);
    printf("End of ULD demo\n");
    return status;
}


#endif



#ifdef MAIN_3 // -----------------------------------------------------------------------


/**
 * @file main.c
 * @brief Main source code for the Tachometer program.
 *
 * This file contains the main entry point and function definitions for the Tachometer program.
 *
 * SysTick is used to check if a collision has been detected and toggles the LEDs on the chassis board.
 *
 * Then, it uses edge-triggered interrupts from the bumper switches to detect a collision.
 * After a collision has been detected, the motors should stop from running.
 *
 * Timer_A is used in this lab:
 *  - Timer A0: Used to generate PWM signals to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (2 kHz)
 *  - Timer A2: Used for input capture to generate interrupts on the rising edge of P8.0
 *  - Timer A3: Used for input capture to generate interrupts on the rising edge of P10.4 and P10.5
 *              and to calculate the period between captures
 *
 * @author Aaron Nanas
 */

#include <stdint.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/GPIO.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Bumper_Switches.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/Timer_A2_Capture.h"
#include "inc/Tachometer.h"

//#define EDGE_DETECT 1
#define MAIN_CONTROLLER 1

// Initialize a global variable for SysTick to keep track of elapsed time in milliseconds
uint32_t SysTick_ms_elapsed = 0;

// Global flag that gets set in Bumper_Switches_Handler.
// This is used to detect if any collisions occurred when any one of the bumper switches are pressed.
uint8_t collision_detected = 0;

// Initialize length of the tachometer buffers
#define BUFFER_LENGTH                 10

// Set the maximum RPM for both wheels
#define MAX_RPM                       120

// Set the minimum RPM for both wheels
#define MIN_RPM                       30

// Global flag used to indicate if EDGE_DETECT is enabled
uint8_t Edge_Detect_Enabled         = 0;

// Global flag used to indicate if Timer_A1 has finished generating pulses for Timer_A2
uint8_t Edge_Detect_Done            = 0;

// Global variable used to keep track of the number of edges
uint16_t Edge_Counter               = 0;

// Desired RPM for the left wheel
uint16_t Desired_RPM_Left           = 70;

// Desired RPM for the right wheel
uint16_t Desired_RPM_Right          = 70;

// Declare a global variable used to store the measured RPM by the left tachometer
uint16_t Actual_RPM_Left            = 0;

// Declare a global variable used to store the measured RPM by the right tachometer
uint16_t Actual_RPM_Right           = 0;

// Set initial duty cycle of the left wheel to 25%
uint16_t Duty_Cycle_Left            = 3750;

// Set initial duty cycle of the right wheel to 25%
uint16_t Duty_Cycle_Right           = 3750;

// Number of left wheel steps measured by the tachometer
int32_t Left_Steps                  = 0;

// Number of right wheel steps measured by the tachometer
int32_t Right_Steps                 = 0;

// Store tachometer period of the left wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Left[BUFFER_LENGTH];

// Store tachometer period of the right wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Right[BUFFER_LENGTH];

// Direction of the left wheel's rotation
enum Tachometer_Direction Left_Direction;

// Direction of the right wheel's rotation
enum Tachometer_Direction Right_Direction;

/**
 * @brief Interrupt service routine for the SysTick timer.
 *
 * The interrupt service routine for the SysTick timer increments the SysTick_ms_elapsed
 * global variable to keep track of the elapsed milliseconds. If collision_detected is 0, then
 * it checks if 500 milliseconds passed. It toggles the front yellow LEDs and turns off the back red LEDs
 * on the chassis board. Otherwise, if collision_detected is set, it turns off the front yellow LEDs
 * and turns on the back red LEDs on the chassis board.
 *
 * @param None
 *
 * @return None
 */
void SysTick_Handler(void)
{
    if (Edge_Detect_Enabled == 0)
    {
        SysTick_ms_elapsed++;
        if (collision_detected == 0)
        {
            if (SysTick_ms_elapsed >= 500)
            {
                P8->OUT &= ~0xC0;
                P8->OUT ^= 0x21;
                SysTick_ms_elapsed = 0;
            }
        }

        else
        {
            P8->OUT |= 0xC0;
            P8->OUT &= ~0x21;
        }
    }
}

/**
 * @brief Bumper switch interrupt handler function.
 *
 * This is the interrupt handler for the bumper switch interrupts. It is called when a falling edge event is detected on
 * any of the bumper switch pins. The function checks if a collision has already been detected; if not, it prints a collision
 * detection message along with the bumper switch state and sets the collision_detected flag to prevent further detections.
 *
 * @param bumper_switch_state An 8-bit unsigned integer representing the bumper switch states at the time of the interrupt.
 *
 * @return None
 */
void Bumper_Switches_Handler(uint8_t bumper_switch_state)
{
    if (collision_detected == 0)
    {
        printf("Collision Detected! Bumper Switch State: 0x%02X\n", bumper_switch_state);
        collision_detected = 1;
    }
}

/**
 * @brief This function handles collision events by instructing the robot to perform a sequence of actions.
 *
 * This function handles collision events by performing the following actions:
 * 1. Stops the motors to halt the robot's movement.
 * 2. Moves the motors backward to recover from the collision.
 * 3. Makes the robot turn right.
 * 4. Resets the collision detection flag.
 *
 * @param None
 *
 * @return None
 */
void Handle_Collision()
{
    // Stop the motors
    Motor_Stop();

    // Make a function call to Clock_Delay1ms(2000)
    Clock_Delay1ms(2000);

    // Move the motors backward with 30% duty cycle
    Motor_Backward(4500, 4500);

    // Make a function call to Clock_Delay1ms(2000)
    Clock_Delay1ms(2000);

    // Stop the motors
    Motor_Stop();

    // Make a function call to Clock_Delay1ms(1000)
    Clock_Delay1ms(1000);

    // Make the robot turn to the right with 10% duty cycle
    Motor_Right(1500, 1500);

    // Make a function call to Clock_Delay1ms(4000)
    Clock_Delay1ms(4000);

    // Stop the motors
    Motor_Stop();

    // Make a function call to Clock_Delay1ms(2000)
    Clock_Delay1ms(1000);

    // Set the collision_detected flag to 0
    collision_detected = 0;
}

/**
 * @brief Update desired RPM based on button presses.
 *
 * This function updates the desired RPM (revolutions per minute) for the robot's left and right motors
 * based on button presses. It checks the status of two buttons and increases the desired RPM by 10
 * if the corresponding button is pressed. If the desired RPM exceeds a maximum threshold, it wraps around
 * to the minimum RPM value.
 *
 * @return None
 */
void Update_Desired_RPM()
{
    uint8_t button_status = Get_Buttons_Status();

    // If Button 1 has been pressed, increment the desired right RPM by 10
    // and limit it when it reaches the maximum RPM value
    if (button_status == 0x10)
    {
        Desired_RPM_Right = Desired_RPM_Right + 10;
        if (Desired_RPM_Right > MAX_RPM)
        {
            Desired_RPM_Right = MIN_RPM;
        }
    }

    // If Button 2 has been pressed, increment the desired left RPM by 10
    // and limit it when it reaches the maximum RPM value
    if (button_status == 0x02)
    {
        Desired_RPM_Left = Desired_RPM_Left + 10;
        if (Desired_RPM_Left > MAX_RPM)
        {
            Desired_RPM_Left = MIN_RPM;
        }
    }
}

/**
 * @brief User-defined function executed by Timer A1 using a periodic interrupt. if the Edge detection is enabled
 *      and the edge counter counts less than 8 pulses then the output toggles. if we've detected enough
 *      edges through Edge_Counter, then the Edge_Detect_Done is set and the output of P8.0 is cleared.
 *
 * @param None
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void) {

    if (Edge_Detect_Enabled == 1) {
        if (Edge_Counter < 8) {

            P8->OUT ^=  0x01;
            Edge_Detect_Done = 0;

        } else {

            P8->OUT &= ~0x01;
            Edge_Detect_Done = 1;
        }
    }
}

/**
 * @brief user-defined function executed by the Timer A2 in interrupt capture mode as an edge-detector.
 *      It counts the number of interrupt pulses that the encoder has generated.
 *
 * @param uint16_t time
 *
 * @return None
 */
void Detect_Edge(uint16_t time)
{
    Edge_Counter = Edge_Counter + 1;
    LED2_Output(RGB_LED_BLUE);
}

int main(void)
{
    int buffer_idx = 0;

    // Initialize the collision_detected flag
    collision_detected = 0;


    // Initialize the 48 MHz Clock
    Clock_Init48MHz();


    // Initialize the built-in red LED and the RGB LEDs
    LED1_Init();
    LED2_Init();


    // Initialize the user buttons
    Buttons_Init();


    // Initialize the front and back LEDs on the chassis board
    Chassis_Board_LEDs_Init();


    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();


    // Initialize the bumper switches which will be used to generate external I/O-triggered interrupts
    Bumper_Switches_Init(&Bumper_Switches_Handler);


    // Initialize the SysTick timer to generate periodic interrupts every 1 ms
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);


    // Initialize Timer A1 periodic interrupts every 0.5 ms
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);


    // Initialize Timer A2 in Capture mode
    Timer_A2_Capture_Init(&Detect_Edge);


    // Initialize the tachometers
    Tachometer_Init();


    // Initialize the DC motors
    Motor_Init();


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();


    while (1)
    {
        int i;

        // Ensure that the motors are not running at the beginning
        Motor_Stop();

        // The bumper switches will be used to initiate the start of the motors
        // If the bumper switches haven't been pressed, then the RPM can be updated
        // by pressing the user buttons
        while (collision_detected == 0)
        {
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_RED);
            Update_Desired_RPM();
            printf("Desired_RPM_Left: %d | Desired_RPM_Right: %d\n", Desired_RPM_Left, Desired_RPM_Right);
            Clock_Delay1ms(200);
        }

        // Flash the LEDs to indicate exit from while-loop (i.e. bumper switches have been pressed)
        for (i = 0; i < 5; i++)
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_OFF);
            Clock_Delay1ms(200);
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_GREEN);
            Clock_Delay1ms(200);
        }

        // After exiting the while-loop, Bumper_Sensors_Handler sets collision_detected to 1
        // Clear the collision_detected flag here
        collision_detected = 0;

        // Calculate the actual RPM and move the motors forward with updated duty cycle values
        // if there is no collision event detected
        while (collision_detected == 0)
        {

            // Get the measurements made by the tachometers
            Tachometer_Get(&Tachometer_Buffer_Left[buffer_idx], &Left_Direction, &Left_Steps, &Tachometer_Buffer_Right[buffer_idx], &Right_Direction, &Right_Steps);
            buffer_idx = buffer_idx + 1;

            if (buffer_idx >= BUFFER_LENGTH)
            {
                buffer_idx = 0;

                // (1/Tachometer Step/Cycles) * (12,000,000 Cycles / Second) * (60 Second / Minute) * (1/360 Rotation/Step)
                Actual_RPM_Left = 2000000 / (Average_of_Buffer(Tachometer_Buffer_Left, BUFFER_LENGTH));
                Actual_RPM_Right = 2000000 / (Average_of_Buffer(Tachometer_Buffer_Right, BUFFER_LENGTH));

                // If the actual RPM measured on the left wheel is greater than the desired RPM,
                // then decrease the duty cycle on the left wheel
                if ((Actual_RPM_Left > (Desired_RPM_Left + 3)) && (Duty_Cycle_Left > 100))
                {
                    Duty_Cycle_Left = Duty_Cycle_Left - 100;
                }

                // Otherwise, if the actual RPM is less than the desired RPM,
                // then increase the duty cycle on the left wheel
                else if ((Actual_RPM_Left < (Desired_RPM_Left - 3)) && (Duty_Cycle_Left < 14898))
                {
                    Duty_Cycle_Left = Duty_Cycle_Left + 100;
                }

                // If the actual RPM measured on the right wheel is greater than the desired RPM,
                // then decrease the duty cycle on the right wheel
                if ((Actual_RPM_Right > (Desired_RPM_Right + 3)) && (Duty_Cycle_Right > 100))
                {
                    Duty_Cycle_Right = Duty_Cycle_Right - 100;
                }

                // Otherwise, if the actual RPM is less than the desired RPM,
                // then increase the duty cycle on the right wheel
                else if ((Actual_RPM_Right < (Desired_RPM_Right - 3)) && (Duty_Cycle_Right < 14898))
                {
                    Duty_Cycle_Right = Duty_Cycle_Right + 100;
                }

                // Move the motors forward with the updated duty cycle
                Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);

                // Compare the desired RPM and actual RPM values using the serial terminal
                printf("Desired_RPM_Left: %d | Desired_RPM_Right: %d\n", Desired_RPM_Left, Desired_RPM_Right);
                printf("Actual_RPM_Left: %d | Actual_RPM_Right: %d\n", Actual_RPM_Left, Actual_RPM_Right);
                printf("Left_Steps: %d | Right Steps: %d\n\n", Left_Steps, Right_Steps);
            }
            Clock_Delay1ms(100);
        }

        // Handle the collision event
        while (collision_detected == 1)
        {
            LED2_Output(RGB_LED_RED);
            Handle_Collision();
        }

    }
}



#endif
