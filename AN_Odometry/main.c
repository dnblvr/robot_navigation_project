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
 *  - Timer A2: Used for input capture to generate interrupts on the rising
 *      edge of P8.0
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

//#define EDGE_DETECT 1
#define MAIN_CONTROLLER 1

// Initialize a global variable for SysTick to keep track of elapsed time in
// milliseconds
uint32_t SysTick_ms_elapsed = 0;

// Global flag that gets set in Bumper_Switches_Handler.
// This is used to detect if any collisions occurred when any one of the bumper
// switches are pressed.
uint8_t collision_detected = 0;

// Initialize length of the tachometer buffers
#define TACHOMETER_BUFFER_LEN         4

// Set the maximum RPM for both wheels
#define MAX_RPM                       120

// Set the minimum RPM for both wheels
#define MIN_RPM                       30

// Global flag used to indicate if EDGE_DETECT is enabled
uint8_t Edge_Detect_Enabled         = 0;

// Global flag used to indicate if Timer_A1 has finished generating pulses for
// Timer_A2
uint8_t Edge_Detect_Done            = 0;

// Global variable used to keep track of the number of edges
uint16_t Edge_Counter               = 0;

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



/**
 * @brief Bumper switch interrupt handler function.
 *
 * This is the interrupt handler for the bumper switch interrupts. It is called
 *      when a falling edge event is detected on any of the bumper switch pins.
 *      The function checks if a collision has already been detected; if not,
 *      it prints a collision detection message along with the bumper switch
 *      state and sets the collision_detected flag to prevent further detec-
 *      tions.
 *
 * @param bumper_switch_state An 8-bit unsigned integer representing the bumper
 *      switch states at the time of the interrupt.
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
 * @brief This function handles collision events by instructing the robot to
 *      perform a sequence of actions.
 *
 * This function handles collision events by performing the following actions:
 *      1. Stops the motors to halt the robot's movement.
 *      2. Moves the motors backward to recover from the collision.
 *      3. Makes the robot turn right.
 *      4. Resets the collision detection flag.
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
 * This function updates the desired RPM (revolutions per minute) for the
 *      robot's left and right motors based on button presses. It checks the
 *      status of two buttons and increases the desired RPM by 10 if the
 *      corresponding button is pressed. If the desired RPM exceeds a maximum
 *      threshold, it wraps around to the minimum RPM value.
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
 * @brief User-defined function executed by Timer A1 using a periodic
 *      interrupt. if the Edge detection is enabled and the edge counter counts
 *      less than 8 pulses then the output toggles. if we've detected enough
 *      edges through Edge_Counter, then the Edge_Detect_Done is set and the
 *      output of P8.0 cleared.
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
 * @brief user-defined function executed by the Timer A2 in interrupt capture
 *      mode as an edge-detector.
 *
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


int buffer_idx = 0;



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


RPLiDAR_Config cfg = {.skip = 4};

/**
 * @brief variables which indicate if the RPLiDAR C1 is operational.
 */
uint8_t is_operational = 0;

/**
 * @brief RPLiDAR C1 RX data buffer.
 */
uint8_t RPLiDAR_RX_Data[BUFFER_LENGTH] = {0};

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
void initialize_RPLiDAR_C1(void) {

    // @todo: process for using SCAN command:
    //  1. turn on the UART TX/RX interrupt enable
    //  2. record our out-characters onto an array until it fills up?
    //  3. turn off the UART TX/RX interrupt enable
    //  4. process the data

    // Initialize UART communications
    EUSCI_A2_UART_Init();

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

/**
 * @brief Motor speed settings
 */
uint16_t forward_speed  = 500, // forward duty cycle
         backward_speed = 300, // reverse duty cycle
         rotation_speed = 300; // rotation duty cycle

/**
 * @brief MotorCommand function pointer type 
 */
typedef void (*MotorCommand)(uint16_t left_speed, uint16_t right_speed);

/**
 * @brief instance of the MotorCommand function pointer
 */
static MotorCommand command;


/**
 * @brief   Wrapper function to stop the motors.
 * @details the parameters are unused.
 * 
 * @param[in] left_speed    The speed of the left motor.
 * @param[in] right_speed   The speed of the right motor.
 */
void Motor_Stop_Wrapper(uint16_t left_speed, uint16_t right_speed)
{
    Motor_Stop();
}

/**
 * @brief 
 * 
 * @param BLE_UART_Buffer 
 */
void Process_BLE_UART_Data(char BLE_UART_Buffer[])
{
    
    command = NULL;

    // turn on GREEN LED when typed onto a UART terminal
//    if (Check_BLE_UART_Data(BLE_UART_Buffer, "RGB LED GREEN")) {
//
//        command = &Motor_Stop_Wrapper;
//        Desired_RPM_Left    = 0;
//        Desired_RPM_Right   = 0;
//        LED2_Output(RGB_LED_GREEN);


    // turn off RGB LED when typed onto a UART terminal
//    } else if ( Check_BLE_UART_Data(BLE_UART_Buffer, "RGB LED OFF") ) {
//
//        command = &Motor_Stop_Wrapper;
//        Desired_RPM_Left    = 0;
//        Desired_RPM_Right   = 0;
//        LED2_Output(RGB_LED_OFF);


    // send Q15.16 Data when typed onto a UART terminal
//    } else if ( Check_BLE_UART_Data(BLE_UART_Buffer, "SEND Q15.16 DATA") ) {
//
//        uint8_t i;
//
//        int32_t data[3] = { 21340,  //  0.3256225586
//                           -279035, // -4.2577362061
//                           -8923};  // -0.1361541748
//
//        for (i = 0; i < 3; i++) {
//            BLE_UART_OutFixed( data[i] );
//            BLE_UART_OutChar('\n');     // 0x0A
//
//        }
//
//        command = &Motor_Stop_Wrapper;
//        Desired_RPM_Left    = 0;
//        Desired_RPM_Right   = 0;


    // 1: RE/START RPLiDAR C1 when de-pressed
//    } else
    if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B10")) {

        command = &Motor_Stop_Wrapper;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        LED2_Output(RGB_LED_WHITE);

        // Initialize the RPLiDAR C1
        initialize_RPLiDAR_C1();

        is_operational = 1;

        LED2_Output(RGB_LED_OFF);

        BLE_UART_OutString("INIT\n");


    // 2: RECORD DATA when de-pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B20")) {

        command = &Motor_Stop_Wrapper;
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

        // Motor_Forward(forward_speed, forward_speed);

        command = &Motor_Forward;
        Desired_RPM_Left    = forward_speed;
        Desired_RPM_Right   = forward_speed;

        LED2_Output(RGB_LED_GREEN);


    // 6: DOWN is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B61")) {

        // Motor_Backward(backward_speed, backward_speed);

        command = &Motor_Backward;
        Desired_RPM_Left    = backward_speed;
        Desired_RPM_Right   = backward_speed;

        LED2_Output(RGB_LED_PINK);


    // 7: LEFT is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B71")) {

        // Motor_Left(rotation_speed, rotation_speed);
        command = &Motor_Left;

        // update duty cycles
        Desired_RPM_Left    = rotation_speed;
        Desired_RPM_Right   = rotation_speed;

        LED2_Output(RGB_LED_YELLOW);


    // 8: RIGHT is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B81")) {

        // Motor_Right(rotation_speed, rotation_speed);
        command = &Motor_Right;

        // update duty cycles
        Desired_RPM_Left    = rotation_speed;
        Desired_RPM_Right   = rotation_speed;

        LED2_Output(RGB_LED_BLUE);


    } else {

        command = &Motor_Stop_Wrapper;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        LED2_Output(RGB_LED_OFF);

    }
}




void some_function() {

}


void BLE_Contact()
{


    // Initialize a buffer that will be used to receive command string from the
    // BLE module
    static char BLE_UART_Buffer[BLE_UART_BUFFER_SIZE] = {0};

    int j;  // internal counter variable

    // read the data from the bluetooth module
    int string_size = BLE_UART_InString(BLE_UART_Buffer, BLE_UART_BUFFER_SIZE);

    printf("BLE UART Data: ");

    for (j = 0; j < string_size; j++) {
        printf("%c", BLE_UART_Buffer[j]);

    }

    printf("\n");

    Process_BLE_UART_Data(BLE_UART_Buffer);

}




#define MOTOR_CONTROL_SYSTEM 1

void Get_Odometry()
{

    char buffer[20];

#ifdef MOTOR_CONTROL_SYSTEM
    static uint32_t counter = 0;
#endif  // MOTOR_CONTROL_SYSTEMS

    // Get the measurements made by the tachometers and update the buffers
    Tachometer_Get(
            &Tachometer_Buffer_Left[buffer_idx],
            &Left_Direction,
            &Left_Steps,

            &Tachometer_Buffer_Right[buffer_idx],
            &Right_Direction,
            &Right_Steps);


#ifdef MOTOR_CONTROL_SYSTEM
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


    // Compare the desired RPM and actual RPM values using the
    // serial terminal
//    printf("Desired: %5d | %5d\n", Desired_RPM_Left, Desired_RPM_Right);
//    printf("Actual:  %5d | %5d\n", Actual_RPM_Left, Actual_RPM_Right);
//    printf("Steps:   %5d | %5d\n\n", Left_Steps, Right_Steps);

//    sprintf(buffer, "%4d, %5d, %5d\n", counter, Left_Steps, Right_Steps);

//    BLE_UART_OutString(buffer);
}

volatile int isr_counter = 0;



int main(void)
{


    // Initialize a buffer that will be used to receive command string from the
    // BLE module
    volatile static char BLE_UART_Buffer[BLE_UART_BUFFER_SIZE] = {0};

    // Initialize the collision_detected flag
    collision_detected = 0;


    // Initialize the 48 MHz Clock
    Clock_Init48MHz();


    // Initialize the built-in red LED and the RGB LEDs
//    LED1_Init();
//    LED2_Init();


    // Initialize the user buttons
//    Buttons_Init();


    // Initialize the front and back LEDs on the chassis board
//    Chassis_Board_LEDs_Init();


    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();



    // bluetooth module initialization ---------------------------------

    BLE_UART_Init(BLE_UART_Buffer);
//    BLE_UART_Init();

//    __enable_irq();

    // Provide a short delay after initialization and reset the BLE module
    Clock_Delay1ms(1000);
    BLE_UART_Reset();

    // Send a message to the BLE module to check if the connection is stable
    BLE_UART_OutString("BLE UART Active\r\n");
    Clock_Delay1ms(1000);


    // bluetooth module initialization end -----------------------------


    // Initialize the bumper switches which will be used to generate external
    // I/O-triggered interrupts
//    Bumper_Switches_Init(&Bumper_Switches_Handler);


    // Initialize the SysTick timer to generate periodic interrupts every 1 ms
//    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);


    // use Timer_A1_Interrupt to get the odometry measurements
    Timer_A1_Interrupt_Init(
            &some_function,   1,
            &Get_Odometry,  1,

            60000); // in clock ticks


//    // Initialize Timer A2 in Capture mode
//    Timer_A2_Capture_Init(&Detect_Edge);


    // Initialize the tachometers as a function of Timer A3
    Tachometer_Init();


    // Initialize the DC motors
    Motor_Init();


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();
//    __set_PRIMASK(0);    // Clear PRIMASK
//    __enable_irq();      // Enable interrupts
//    __ASM volatile ("cpsie i" : : : "memory");  // Assembly enable


//    int j = 0;
    int k;

    printf("test scan -------\n");

    while (1)
    {

//        BLE_Contact();

        printf("%5d\n", isr_counter);


//        char c;
//
//
//        if ((EUSCI_A3->IFG & 0x0001) == 1) {
//
//
//            c   = (char)(EUSCI_A3->RXBUF);
//            EUSCI_A3->IFG  &=  ~0x0001;
//
////            printf("RX flag set but ISR not called\n");
//            printf("%c", c);
//
//            j++;
//        }
//
//        if (j > 5) {
//            j = 0;
//            printf("\n");
//        }
//        BLE_Contact();


//        printf("BLE: ");
//
//        for (k = 0; k < 4; k++) {
//            printf("%c ", BLE_UART_Buffer[k]);
//
//
//        }
//
//        printf("\n");

        Clock_Delay1ms(100);

    }
}
