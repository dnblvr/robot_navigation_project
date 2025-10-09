/**
 * @file main.c
 * @brief Main source code for the Tachometer program.
 *
 * This file contains the main entry point and function definitions for the
 *      Tachometer program.
 *
 * SysTick is used to check if a collision has been detected and toggles the
 *      LEDs on the chassis board.
 *
 * Then, it uses edge-triggered interrupts from the bumper switches to detect a
 *      collision. After a collision has been detected, the motors should stop
 *      from running.
 *
 * Timer_A is used in this lab:
 *  - Timer A0: Used to generate PWM signals to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (2 kHz)
 *  - Timer A3: Used for input capture to generate interrupts on the rising
 *      edge of P10.4 and P10.5 and to calculate the period between captures.
 *
 * @author Aaron Nanas
 */

#include <stdint.h>
#include "msp.h"
//#include "inc/"
#include "inc/RPLiDAR_A2_UART.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include <inc/BLE_A3_UART.h>
#include "inc/GPIO.h"
//#include "inc/SysTick_Interrupt.h"
#include "inc/Bumper_Switches.h"
#include "inc/Motor.h"
//#include "inc/Timer_A0_Interrupt.h"
#include "inc/Timer_A1_Interrupt.h"
//#include "inc/Timer_A2_Capture.h"
#include "inc/Tachometer.h"



// ----------------------------------------------------------------------------
//
//  HIGHER-LEVEL RPLiDAR FUNCTIONS
//
// ----------------------------------------------------------------------------


/**
 * @brief Command definitions for the RPLiDAR C1
 * @details These commands are used to control the RPLiDAR C1 and retrieve
 *          information.
 *          For the single-request, no-response commands, the format is in:
 *              {command, byte-length, time}
 */

// Single-Request, No-Response
const uint16_t     STOP[3] = {0x25,  0,  10},
                  RESET[3] = {0x40,  0, 500};

// Single-Request, Single-Response
const uint8_t  GET_INFO[2] = {0x50, 20},
             GET_HEALTH[2] = {0x52,  8},
         GET_SAMPLERATE[2] = {0x59,  0},
         GET_LIDAR_CONF[2] = {0x84,  0},

// Single-Request, Multiple-Response
                   SCAN[2] = {0x20,  5},
           EXPRESS_SCAN[2] = {0x82,  5};


RPLiDAR_Config cfg = {.skip_factor = 4};

/**
 * @brief instance of the struct that only lives in the RPLiDAR source file.
 */
//RPLiDAR_Config config;


//volatile int    lidar_isr_counter,
//                lidar_print_counter;

volatile int message_length;

/**
 * @brief variables which indicate if the RPLiDAR C1 is operational.
 */
uint8_t is_operational = 0;

/**
 * @brief RPLiDAR C1 RX data buffer.
 */
uint8_t RPLiDAR_RX_Data[RPLiDAR_UART_BUFFER_SIZE] = {0};

/**
 * @brief RPLiDAR C1 output data buffer.
 */
float output[FLOAT_BUFFER][3] = {0};


/**
 * @brief Initialize the RPLiDAR C1.
 *
 * This function initializes the RPLiDAR C1 by sending the appropriate commands
 *      to the device and configuring the UART A2 interface.
 *
 * @return None
 */
void initialize_RPLiDAR_C1(RPLiDAR_Config *config) {

    // @todo: process for using SCAN command:
    //  1. turn on the UART TX/RX interrupt enable
    //  2. record our out-characters onto an array until it fills up?
    //  3. turn off the UART TX/RX interrupt enable
    //  4. process the data

    // Initialize UART communications
    EUSCI_A2_UART_Init(config, RPLiDAR_RX_Data);

    // printf("SRNR: STOP\n");
    Single_Request_No_Response(STOP);
    Clock_Delay1ms(1);

    // printf("SRNR: RESET\n");
    Single_Request_No_Response(RESET);
    Clock_Delay1ms(1);


    // printf("SRSR: GET_HEALTH\n");
    Single_Request_Single_Response(GET_HEALTH, RPLiDAR_RX_Data);
    Clock_Delay1ms(1);


    // printf("SRMR: SCAN\n");
    Single_Request_Multiple_Response(SCAN, RPLiDAR_RX_Data);
    Clock_Delay1ms(200);

}


// ----------------------------------------------------------------------------
//
//  MOTOR-OPERATION FUNCTIONS
//
// ----------------------------------------------------------------------------


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
 * @brief instance of the MotorCommand function pointer
 */
static MotorCommand command;

/**
 * @brief   Wrapper function to stop the motors. In order to offer compatibility with the
 *      MotorCommand function prototype, the parameters are unused.
 * 
 * @param[in] left_speed    The speed of the left motor.
 * @param[in] right_speed   The speed of the right motor.
 */
void Motor_Stop_Wrapper(uint16_t left_speed, uint16_t right_speed)
{
    Motor_Stop();
}




// ----------------------------------------------------------------------------
//
//  TACHOMETER FUNCTIONS
//
// ----------------------------------------------------------------------------


// Initialize length of the tachometer buffers
#define TACHOMETER_BUFFER_LEN         4

// Set the maximum RPM for both wheels
//#define MAX_RPM                       120

// Set the minimum RPM for both wheels
//#define MIN_RPM                       30

// Desired RPM for the left wheel
uint16_t Desired_RPM_Left           = 70;

// Desired RPM for the right wheel
uint16_t Desired_RPM_Right          = 70;

// Declare a global variable used to store the measured RPM by the left
// tachometer
uint16_t Actual_RPM_Left            = 0;

// Declare a global variable used to store the measured RPM by the right
// tachometer
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
uint16_t Tachometer_Buffer_Left[TACHOMETER_BUFFER_LEN];

// Store tachometer period of the right wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Right[TACHOMETER_BUFFER_LEN];

// Direction of the left wheel's rotation
enum Tachometer_Direction Left_Direction;

// Direction of the right wheel's rotation
enum Tachometer_Direction Right_Direction;

// Buffer index
int buffer_idx = 0;

#define MOTOR_CONTROL_SYSTEM 1

void Get_Odometry()
{
//    char buffer[40];

#ifdef MOTOR_CONTROL_SYSTEM

    static uint32_t counter = 0;

    buffer_idx = buffer_idx + 1;

    counter++;

    // the incremental control system is updated when the buffer is filled.
    // from there, it takes the average
    if (buffer_idx >= TACHOMETER_BUFFER_LEN)
    {

        // reset the buffer index
        buffer_idx = 0;

        // (1/Tachometer Step/Cycles) * (12,000,000 Cycles / Second) * (60 Second / Minute) * (1/360 Rotation/Step)

        Actual_RPM_Left     = 2000000 / ( Average_of_Buffer(Tachometer_Buffer_Left,  TACHOMETER_BUFFER_LEN) );
        Actual_RPM_Right    = 2000000 / ( Average_of_Buffer(Tachometer_Buffer_Right, TACHOMETER_BUFFER_LEN) );


        // If the actual RPM measured on the left wheel is greater than
        // the desired RPM, then decrease the duty cycle on the left
        // wheel
        if (        (Actual_RPM_Left > (Desired_RPM_Left + 3))
                &&  (Duty_Cycle_Left > 100)) {

            Duty_Cycle_Left = Duty_Cycle_Left - 100;


        // Otherwise, if the actual RPM is less than the desired RPM,
        // then increase the duty cycle on the left wheel
        } else if (     (Actual_RPM_Left < (Desired_RPM_Left - 3))
                    &&  (Duty_Cycle_Left < 14898)) {

            Duty_Cycle_Left = Duty_Cycle_Left + 100;

        }


        // If the actual RPM measured on the right wheel is greater
        // than the desired RPM, then decrease the duty cycle on the
        // right wheel
        if (        (Actual_RPM_Right > (Desired_RPM_Right + 3))
                &&  (Duty_Cycle_Right > 100)) {

            Duty_Cycle_Right = Duty_Cycle_Right - 100;


        // Otherwise, if the actual RPM is less than the desired RPM,
        // then increase the duty cycle on the right wheel
        } else if (     (Actual_RPM_Right < (Desired_RPM_Right - 3))
                    &&  (Duty_Cycle_Right < 14898)) {

            Duty_Cycle_Right = Duty_Cycle_Right + 100;
        }

    }


    // Move the motors using the function pointer "command" with the updated duty cycle
    if (command != NULL) {

        command(Duty_Cycle_Left, Duty_Cycle_Right);

    } else {

        Motor_Stop();
    }

#endif  // defined(MOTOR_CONTROL_SYSTEM)

}


// ----------------------------------------------------------------------------
//
//  BLUETOOTH COMMUNICATION FUNCTIONS
//
// ----------------------------------------------------------------------------


/**
 * @brief processes UART feed based on if they're similar to
 *
 * @param BLE_UART_Buffer
 */
void Process_BLE_UART_Data(volatile char BLE_UART_Buffer[])
{

    command = NULL;

    if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B10")) {

        command             = &Motor_Stop_Wrapper;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        LED2_Output(RGB_LED_WHITE);

        // Initialize the RPLiDAR C1
//        initialize_RPLiDAR_C1();

        is_operational = 1;

        LED2_Output(RGB_LED_OFF);

        BLE_UART_OutString("INIT\n");


    // 2: RECORD DATA when de-pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B20")) {

        command             = &Motor_Stop_Wrapper;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        if (is_operational) {

            LED2_Output(RGB_LED_RED);

            // commented, but make sure
            // Gather_LiDAR_Data(&cfg, 1, RPLiDAR_RX_Data, output);

            BLE_UART_OutString("DATA REC\n");

            // uint8_t matrix_multiply(
            //         uint8_t a_rows, uint8_t a_cols, float a[a_rows][a_cols],
            //         uint8_t b_rows, uint8_t b_cols, float b[b_rows][b_cols],
            //         float result[a_rows][b_cols]);
        }


    // 5: UP is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B51")) {

        command             = &Motor_Forward;
        Desired_RPM_Left    = forward_speed;
        Desired_RPM_Right   = forward_speed;

        LED2_Output(RGB_LED_GREEN);


    // 6: DOWN is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B61")) {

        command             = &Motor_Backward;
        Desired_RPM_Left    = backward_speed;
        Desired_RPM_Right   = backward_speed;

        LED2_Output(RGB_LED_PINK);


    // 7: LEFT is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B71")) {

        // update duty cycles
        command             = &Motor_Left;
        Desired_RPM_Left    = rotation_speed;
        Desired_RPM_Right   = rotation_speed;

        LED2_Output(RGB_LED_YELLOW);


    // 8: RIGHT is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B81")) {

        // update duty cycles
        command             = &Motor_Right;
        Desired_RPM_Left    = rotation_speed;
        Desired_RPM_Right   = rotation_speed;

        LED2_Output(RGB_LED_BLUE);


    } else {

        command             = &Motor_Stop_Wrapper;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        LED2_Output(RGB_LED_OFF);

    }
}



void BLE_Contact(volatile char UART_Buffer[])
{

    // Initialize a buffer that will be used to receive command string from the
    // BLE module
//    static char BLE_UART_Buffer[BLE_UART_BUFFER_SIZE] = {0};

    int j;  // internal counter variable

    // read the data from the bluetooth module
    // int string_size = BLE_UART_InString(BLE_UART_Buffer, BLE_UART_BUFFER_SIZE);
    int string_size = message_length;

    if (string_size < 4)
        return;

    // printf("BLE UART Data: ");

    for (j = 0; j < string_size; j++) {
        printf("%c", UART_Buffer[j]);

    }

     printf("\n");

     Process_BLE_UART_Data(UART_Buffer);

}

void process_rplidar_data() {

    j = 0;
    for (i = start; i < end; i += MESSAGE_LENGTH*cfg->skip_factor) {

        uint8_t is_nonzero;

        if (j >= FLOAT_BUFFER)
            break;

        // convert the data to distance and angle
        is_nonzero = to_angle_distance( &RX_Data[i], distance_angle );

        // if zero radius, then disregard the data
        if (!is_nonzero)
            continue;

        // convert the polar coordinates to Cartesian coordinates
        polar_to_cartesian( distance_angle, out[j] );


#ifdef RPLIDAR_DEBUG
        // print the distance and angle
        fprintf(stdout, "%i\t%3.2f rad. @ %5.2f mm\n",
                j, distance_angle[1], distance_angle[0]);

#endif

        // print the position
        fprintf(stdout, "%i\t%10.2f %10.2f\n",
                j, out[j][0], out[j][1]);

        ++j;

    }

}


void print_results(void) {

    static int  l   = 0;
    int         k;

//    printf("%5d\n", isr_counter);


    // Compare the desired RPM and actual RPM values using the
    // serial terminal
//    printf("Desired: %5d | %5d\n", Desired_RPM_Left, Desired_RPM_Right);
//    printf("Actual:  %5d | %5d\n", Actual_RPM_Left, Actual_RPM_Right);
//    printf("Steps:   %5d | %5d\n\n", Left_Steps, Right_Steps);

//    sprintf(buffer, "%4d, %5d, %5d\n", counter, Left_Steps, Right_Steps);

//    BLE_UART_OutString(buffer);


    // print the first n resutls
//    if (l < 25) {
//    //    for (k = 0; k < cfg.print_counter; k += 5) {
//        for (k = 0; k < 25*MESSAGE_LENGTH; k += 5) {
//            printf("%02X%02X%02X%02X%02X %i; ",
//                   RPLiDAR_RX_Data[k+0],
//                   RPLiDAR_RX_Data[k+1],
//                   RPLiDAR_RX_Data[k+2],
//                   RPLiDAR_RX_Data[k+3],
//                   RPLiDAR_RX_Data[k+4],
//                   pattern(RPLiDAR_RX_Data + k));
//
//        }
//        printf("\n\n");
//
//        l++;
//    }

    if (l < 25) {
//        for (k = 0; k < cfg.print_counter; k += 5) {
        for (k = 0; k < RPLiDAR_UART_BUFFER_SIZE; k += 5) {
//        for (k = 0; k < 9*MESSAGE_LENGTH; k += 5) {
            printf("%i", pattern(RPLiDAR_RX_Data + k));
        }
        printf("\n\n");

        l++;
    }

//    printf("%6d %6d\n", cfg.print_counter, cfg.isr_counter);
}


// ----------------------------------------------------------------------------
//
//  HIGHER-LEVEL RPLiDAR FUNCTIONS
//
// ----------------------------------------------------------------------------


volatile uint8_t    task_flag       = 0,
                    tick_counter    = 0;

#define MAX_TICK_COUNTER    20

#define TASK_0_FLAG     0x01 << 0
#define TASK_1_FLAG     0x01 << 1
#define TASK_2_FLAG     0x01 << 2
#define TASK_3_FLAG     0x01 << 3
#define TASK_4_FLAG     0x01 << 4
#define TASK_5_FLAG     0x01 << 5
#define TASK_6_FLAG     0x01 << 6
#define TASK_7_FLAG     0x01 << 7

#define TASK_0_DIV_FREQ 1
#define TASK_1_DIV_FREQ 1
#define TASK_2_DIV_FREQ 1
#define TASK_3_DIV_FREQ 1
#define TASK_4_DIV_FREQ 1
#define TASK_5_DIV_FREQ 1
#define TASK_6_DIV_FREQ 1
#define TASK_7_DIV_FREQ 20

/**
 * @brief Task selector function
 * 
 */
void Task_Selector(void) {

    // Increment and keep the counter in the loop
    tick_counter++;
    if (tick_counter >= MAX_TICK_COUNTER)
        tick_counter    = 0;


    // task 0
    if ( (tick_counter % TASK_0_DIV_FREQ) == 0 )
        task_flag  |= TASK_0_FLAG;

    // task 1
    if ( (tick_counter % TASK_1_DIV_FREQ) == 0 )
        task_flag  |= TASK_1_FLAG;

    // task 2
    if ( (tick_counter % TASK_2_DIV_FREQ) == 0 )
        task_flag  |= TASK_2_FLAG;

    // task 3
    if ( (tick_counter % TASK_3_DIV_FREQ) == 0 )
        task_flag  |= TASK_3_FLAG;

    // task 4
    if ( (tick_counter % TASK_4_DIV_FREQ) == 0 )
        task_flag  |= TASK_4_FLAG;

    // task 5
    if ( (tick_counter % TASK_5_DIV_FREQ) == 0 )
        task_flag  |= TASK_5_FLAG;

    // task 6
    if ( (tick_counter % TASK_6_DIV_FREQ) == 0 )
        task_flag  |= TASK_6_FLAG;

    // task 7 - send results
    if ( (tick_counter % TASK_7_DIV_FREQ) == 0 )
        task_flag  |= TASK_7_FLAG;

}

// ----------------------------------------------------------------------------
//
//  CONFIGURATION and MAIN LOOP
//
// ----------------------------------------------------------------------------

int main(void)
{

    // Initialize a buffer that will be used to receive command string from the
    // BLE module
    volatile char BLE_UART_Buffer[BLE_UART_BUFFER_SIZE] = {0};


    // Initialize the 48 MHz Clock
    Clock_Init48MHz();


    // Initialize the built-in red LED and the RGB LEDs
//    LED1_Init();
//    LED2_Init();


    // Initialize the user buttons
//    Buttons_Init();


    // Initialize the front and back LEDs on the chassis board
    Chassis_Board_LEDs_Init();


    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();



    // BlueTooth module initialization ---------------------------------

    BLE_UART_Init(BLE_UART_Buffer);

    // Provide a short delay after initialization and reset the BLE module
    Clock_Delay1ms(1000);
    BLE_UART_Reset();

    // Send a message to the BLE module to check if the connection is stable
    BLE_UART_OutString("BLE UART Active\r\n");
    Clock_Delay1ms(1000);


    // BlueTooth module initialization end -----------------------------


    // Initialize the bumper switches which will be used to generate external
    // I/O-triggered interrupts
//    Bumper_Switches_Init(&Bumper_Switches_Handler);


    // Initialize the SysTick timer to generate periodic interrupts every 1 ms
//    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);


    // use Timer_A1_Interrupt to get the odometry measurements
    Timer_A1_Interrupt_Init(
            &Task_Selector,
            60000); // in clock ticks


//    // Initialize Timer A2 in Capture mode
//    Timer_A2_Capture_Init(&Detect_Edge);


    // Initialize the tachometers as a function of Timer A3
    Tachometer_Init();


    // Initialize the DC motors
    Motor_Init();


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();




    printf("test scan -------\n\n");

//    int l;

    // RPLiDAR C1 initialization ---------------------------------------

    EUSCI_A2->IE       |=  0x03;
    EUSCI_A0->IE       |=  0x03;

    initialize_RPLiDAR_C1(&cfg);

    printf("C1 initialized\n");

//    printf("%02X\n", cfg.print_counter);
//
//    // this procedure analyzes the current ISR
//    while (cfg.print_counter < 15) {
//
//        printf("%c\n", cfg.print_counter);
//
//        EUSCI_A2->IE       |=  0x03;
//        EUSCI_A0->IE       |=  0x03;
//
//        // wait-for-interrupt macro
////        __WFI();
//        Clock_Delay1ms(1);
//    }
//
//    int k;
//    for (k = 0; k > cfg.print_counter; ++k) {
//        printf("%02X", RPLiDAR_RX_Data[k]);
//    }
//
//    printf("\n");

    // RPLiDAR C1 initialization end -----------------------------------


//    DisableInterrupts();

//    printf("done\n");

//    return 0;



    while (1)
    {


        /**
         * @brief enabling RXIE for all UARTs used affirm that the interrupt is ON.
         *      By itself, it turns off.
         * @note Turning on the UART RX (b0) and TX (b1) disables RX for whatever
         *      reason.
         */
//        EUSCI_A0->IE       |=  0x0003;
//        EUSCI_A2->IE       |=  0x0003;

        EUSCI_A0->IE       |=  0x0001;
        EUSCI_A2->IE       |=  0x0001;
        EUSCI_A3->IE       |=  0x0001;


//        printf("in main loop\n\n");

        if (task_flag & TASK_0_FLAG) {
            // clear the flag
            task_flag  &= ~TASK_0_FLAG;

//            BLE_Contact(BLE_UART_Buffer);
        }


        // Get the measurements made by the tachometers and update the buffers
//        if (task_flag & TASK_1_FLAG) {
            task_flag  &= ~TASK_1_FLAG;
//
//            Tachometer_Get(
//                    &Tachometer_Buffer_Left[buffer_idx],
//                    &Left_Direction,
//                    &Left_Steps,
//
//                    &Tachometer_Buffer_Right[buffer_idx],
//                    &Right_Direction,
//                    &Right_Steps);
//        }



//        if (task_flag & TASK_2_FLAG) {
            task_flag  &= ~TASK_2_FLAG;
//
////            Get_Odometry();
//        }



        if (task_flag & TASK_3_FLAG) {
            task_flag  &= ~TASK_3_FLAG;

//            Gather_LiDAR_Data(&cfg, 1, RPLiDAR_RX_Data, output);
            cfg.record_data = 1;
        }



        if (task_flag & TASK_7_FLAG) {
            task_flag  &= ~TASK_7_FLAG;

            print_results();
        }


        /**
         * @brief   wait-for-interrupt macro. This function allows the loop to
         *          proceed if any interrupt is triggered.
         */
        __WFI();


    }
}
