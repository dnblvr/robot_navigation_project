
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
#include "inc/RPLiDAR_A2_UART.h"

#include "inc/matrices.h"




// Initialize a global variable for SysTick to keep track of elapsed time in milliseconds
uint32_t SysTick_ms_elapsed         = 0;
uint32_t SysTick_seconds_elapsed    = 0;

// Global flag used to indicate if EDGE_DETECT is enabled
uint8_t Edge_Detect_Enabled         = 0;

// Global variable used to keep track of the number of edges
uint16_t Edge_Counter               = 0;

/**
 * @brief Interrupt service routine for the SysTick timer.
 *
 * The interrupt service routine for the SysTick timer increments the SysTick_ms_elapsed
 * global variable to keep track of the elapsed milliseconds. If collision_detected is 0, then
 * it checks if 1000 milliseconds passed. 
 *
 * @param None
 *
 * @return None
 */
void SysTick_Handler(void)
{
    SysTick_ms_elapsed++;
    if (SysTick_ms_elapsed >= 500) {

        SysTick_ms_elapsed = 0;
    }
}



/**
 * @brief Command definitions for the RPLiDAR C1
 * @details These commands are used to control the RPLiDAR C1 and retrieve information.
 *          for the single-request, no-response commands, the format is in:
 *          {command, byte-length, time}
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


uint8_t RPLiDAR_RX_Data[BUFFER_LENGTH] = {0};

float output[FLOAT_BUFFER][3] = {0};





int main(void)
{

    // stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;


    DisableInterrupts();


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


    // Initialize EUSCI_A2_UART
    EUSCI_A2_UART_Init();


    // Enable the interrupts used by the SysTick and Timer_A timers
    EnableInterrupts();



    Clock_Delay1ms(1000);

    RPLiDAR_Config cfg = {.skip = 4};

    printf("test start ---------------\n");




//    printf("SRNR: STOP\n");
    Single_Request_No_Response(STOP);
    Clock_Delay1ms(1);

//    printf("SRNR: RESET\n");
    Single_Request_No_Response(RESET);
    Clock_Delay1ms(1);


    // printf("SRSR: GET_INFO\n");
    // Single_Request_Single_Response(GET_INFO, RPLiDAR_RX_Data);
    // Clock_Delay1ms(1);

//    printf("SRSR: GET_HEALTH\n");
    Single_Request_Single_Response(GET_HEALTH, RPLiDAR_RX_Data);
    Clock_Delay1ms(1);

    // printf("SRSR: GET_SAMPLERATE\n");
    // Single_Request_Single_Response(GET_SAMPLERATE, RPLiDAR_RX_Data);
    // Clock_Delay1ms(1);

//    printf("SRSR: GET_LIDAR_CONF\n");
//    Single_Request_Single_Response(GET_LIDAR_CONF, RPLiDAR_RX_Data);
//    Clock_Delay1ms(1);




//    printf("SRMR: SCAN\n");
    Single_Request_Multiple_Response(SCAN, RPLiDAR_RX_Data);
    Clock_Delay1ms(200);

    // printf("SRMR: EXPRESS_SCAN\n");
    // Single_Request_Multiple_Response(EXPRESS_SCAN, RPLiDAR_RX_Data);
    // Clock_Delay1ms(1);
    
    uint32_t counter = 6;

    while (counter) {
        // process for using SCAN command:
        // 1. turn on  the UART TX/RX interrupt enable
        // 2. record our out-characters onto an array until it fills up?
        // 3. turn off the UART TX/RX interrupt enable
        // 4. process the data and

//        Clock_Delay1ms(1000);

//        EUSCI_A2_UART_Restart();



        // to use express scan command:
        // according to the document, the sources recommend:
        // 1. using the GET_LIDAR_CONF command (specifically with
        // configuration entry 0x7C) to obtain the "Typical Scan Mode"
        // ID and then using this ID in the working_mode field of the
        // EXPRESS_SCAN request to make the LIDAR work under its best performance


        Gather_LiDAR_Data(&cfg, 1, RPLiDAR_RX_Data, output);

//        uint8_t matrix_multiply(
//                uint8_t a_rows, uint8_t a_cols, float a[a_rows][a_cols],
//                uint8_t b_rows, uint8_t b_cols, float b[b_rows][b_cols],
//                float result[a_rows][b_cols]);




        counter--;


        printf("---------------\n");
    }


    printf("end ---------------\n");

    return 0;

}





#endif // MAIN_1


#ifdef MAIN_2 // -----------------------------------------------------------------------

#include "inc/matrices.h"
#include "inc/coordinate_transform.h"
#include "inc/ransac.h"
#include "inc/graphslam.h"
#include "inc/rrt_star.h"

// #include "Motor.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <time.h>


typedef struct {
    float x_min, x_max, y_min, y_max;
} Obstacle_GS;


/**
 * @brief   Generate a random number from a Gaussian distribution
 * @details This applies a Box-Muller transform for Gaussian noise
 * 
 * @param mean 
 * @param stddev 
 * @return float 
 */
float randn(
        float mean,
        float stddev)
{
    float u1 = (rand() + 1.0f)/(RAND_MAX + 2.0f);
    float u2 = (rand() + 1.0f)/(RAND_MAX + 2.0f);

    return mean + stddev*sqrtf( -2.0f*logf(u1) )*cosf( 2.0f*M_PI*u2 );
}


/**
 * @brief This function checks if a point (x, y) is inside a given Obstacle
 * 
 * @param x, @param y (x, y) coordinates of the point
 * @param  obs  obstacle structure containing the bounds
 * @return int  0 if the point is outside the Obstacle; 1 if it is inside
 */
int is_in_Obstacle(
        float x, float y,

        Obstacle_GS *obs)
{
    return    (x >= obs->x_min) && (x <= obs->x_max)
           && (y >= obs->y_min) && (y <= obs->y_max);
}


void example_rectangle_room_with_inner_Obstacle(void) {

    // counter variables
    int     i, w, l;

    float   outer_x_min, outer_x_max,
            outer_y_min, outer_y_max;

    Obstacle_GS inner_room = {8.0f, 12.0f,
                              8.0f, 12.0f};

    srand((unsigned int)time(NULL));

    // --- Room and Obstacle Setup ---
    outer_x_min = 0.0f;     outer_x_max = 20.0f;
    outer_y_min = 0.0f;     outer_y_max = 20.0f;


    // --- Landmarks: corners of outer and inner room ---
    LandmarkMeasurement* landmarks[MAX_LANDMARKS] = {0};
    int num_landmarks = 0;

    // Outer room corners
    float outer_corners[4][2] = {
        {outer_x_min, outer_y_min},
        {outer_x_max, outer_y_min},
        {outer_x_max, outer_y_max},
        {outer_x_min, outer_y_max}
    };
    
    for (i = 0; i < 4; ++i) {
        landmarks[num_landmarks]     = malloc(sizeof(LandmarkMeasurement));
        landmarks[num_landmarks]->id = num_landmarks;
        landmarks[num_landmarks]->x  = outer_corners[i][0];
        landmarks[num_landmarks]->y  = outer_corners[i][1];
        num_landmarks++;
    }

    // Inner room corners
    float inner_corners[4][2] = {
        {inner_room.x_min, inner_room.y_min},
        {inner_room.x_max, inner_room.y_min},
        {inner_room.x_max, inner_room.y_max},
        {inner_room.x_min, inner_room.y_max}
    };

    for (i = 0; i < 4; ++i) {
        landmarks[num_landmarks]     = malloc(sizeof(LandmarkMeasurement));
        landmarks[num_landmarks]->id = num_landmarks;
        landmarks[num_landmarks]->x  = inner_corners[i][0];
        landmarks[num_landmarks]->y  = inner_corners[i][1];
        num_landmarks++;
    }

    // --- Path: Four hallways around the inner room ---
    // Start at (2,2), go right, up, left, down (around the inner room)
    int num_poses = 0;
    GraphNode *head = malloc(sizeof(GraphNode));

    head->pose.x        = 2.0f;
    head->pose.y        = 2.0f;
    head->pose.theta    = 0.0f;

    head->num_observations  = 0;
    head->observations      = NULL;
    head->next              = NULL;
    GraphNode *current      = head;

    num_poses++;

    // Define waypoints for the four hallways (with some noise)
    float waypoints[5][3] = {
        {16.0f,  2.0f,    0.0f},    // right hallway
        {16.0f, 16.0f,  M_PI_2},    // up hallway
        { 2.0f, 16.0f,  M_PI  },    // left hallway
        { 2.0f,  2.0f, -M_PI_2},  	// down hallway (back to start)
        { 2.0f,  2.0f,    0.0f}     // loop closure
    };

    for (w = 0; w < 5; ++w) {

        float dx     = waypoints[w][0] - current->pose.x;
        float dy     = waypoints[w][1] - current->pose.y;
        float dtheta = waypoints[w][2] - current->pose.theta;


        // Add noise to motion
        OdometryEdge control;
        control.dx      = randn(0.0f, 0.10f) + dx;
        control.dy      = randn(0.0f, 0.10f) + dy;
        control.dtheta  = randn(0.0f, 0.02f) + dtheta;


        // Predict motion and create new node
        GraphNode *new_node = predict_motion(current, control);
        current = new_node;
        num_poses++;


        // Add observations to visible landmarks (simulate range/bearing with noise)
        current->num_observations   = 0;
        current->observations       = NULL;
        for (l = 0; l < num_landmarks; ++l) {

            float lx = landmarks[l]->x;
            float ly = landmarks[l]->y;

            // Only observe landmarks not blocked by the inner room
            if ( !is_in_Obstacle(
                    (current->pose.x + lx)/2,
                    (current->pose.y + ly)/2,
                    &inner_room) )
            {
                float dx_lm     = lx - current->pose.x;
                float dy_lm     = ly - current->pose.y;

                float range     =   sqrtf(dx_lm*dx_lm + dy_lm*dy_lm)
                                  + randn(0.0f, 0.05f);

                float bearing   =   atan2f(dy_lm, dx_lm)
                                  - current->pose.theta
                                  + randn(0.0f, 0.01f);

                Observation obs;
                obs.landmark        = landmarks[l];
                obs.landmark_index  = l;
                obs.range           = range;
                obs.bearing         = bearing;

                add_observation(current, obs);
            }
        }
    }

    // --- Run SLAM ---
    gauss_newton_slam(head, landmarks, num_poses, num_landmarks, 10, 1e-4f);

    // --- Print results ---
    printf("Final poses:\n");
    GraphNode *node = head;
    int idx = 0;
    while (node != NULL) {
        printf("Pose %d: x=%.2f, y=%.2f, theta=%.2f\n",
               idx,
               node->pose.x,
               node->pose.y,
               node->pose.theta);

        node = node->next;
        idx++;
    }

    printf("Final landmarks:\n");
    for (l = 0; l < num_landmarks; ++l) {
        printf("Landmark %d: x=%.2f, y=%.2f\n",
               l,
               landmarks[l]->x,
               landmarks[l]->y);
    }

    // --- Free memory ---
    node = head;
    while (node != NULL) {
        GraphNode *next = node->next;

        if (node->observations) {
            free(node->observations);
        }

        free(node);
        node = next;
    }

    for (l = 0; l < num_landmarks; ++l) {
        free(landmarks[l]);
    }

}



// void example_ransac_2(void);
// void example_ransac(void);
// void example_ransac_2(void);

int main(void) {

	// rrt_star_demo();
    example_rectangle_room_with_inner_Obstacle();

    // example_coord_transform();
	// example_coord_transform_2();


	// example_ransac();
	// example_ransac_2();

  	return 0;
}


#endif // MAIN_2


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


