/*
 * BLE.c
 *
 */

#include "../inc/BLE.h"


// ----------------------------------------------------------------------------
//
//  MOTOR-OPERATION FUNCTIONS
//
// ----------------------------------------------------------------------------


// forward duty cycle
#define FORWARD_SPEED   ((uint16_t)(0.30f * TIMER_A0_PERIOD_CONSTANT))

// reverse duty cycle
#define BACKWARD_SPEED  ((uint16_t)(0.10f * TIMER_A0_PERIOD_CONSTANT))

// rotation duty cycle
#define ROTATION_SPEED  ((uint16_t)(0.05f * TIMER_A0_PERIOD_CONSTANT))


// Desired RPM for the left wheel
uint16_t Desired_RPM_Left    = 0;

// Desired RPM for the right wheel
uint16_t Desired_RPM_Right   = 0;

/**
 * @brief variable that tracks
 */
enum Tachometer_Direction current_state = STOPPED;


void Process_BLE_UART_Data(volatile char BLE_UART_Buffer[])
{

    // internal counter variable
    static MotorCommand command = NULL;


    if (Check_UART_Data(BLE_UART_Buffer, "!B11")) {

        command             = &Motor_Forward;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        LED2_Output(RGB_LED_OFF);


    // 2: begin operation
    } else if (Check_UART_Data(BLE_UART_Buffer, "!B21")) {

        command             = &Motor_Forward;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        LED2_Output(RGB_LED_OFF);


    // 5: UP is activated when pressed
    } else if (Check_UART_Data(BLE_UART_Buffer, "!B51")) {


        if (current_state == STOPPED) {

            current_state       = FORWARD;

            command             = &Motor_Forward;
            Desired_RPM_Left   += FORWARD_SPEED;
            Desired_RPM_Right  += FORWARD_SPEED;

            LED2_Output(RGB_LED_GREEN);

        // if forward, stop and reset desired RPM
        } else if (current_state == FORWARD) {

            current_state       = STOPPED;

            command             = &Motor_Forward;
            Desired_RPM_Left    = 0;
            Desired_RPM_Right   = 0;

            LED2_Output(RGB_LED_OFF);

        }

        // common: state transition from STOPPED to FORWARD and vice versa
        // allowed only.


    // 6: DOWN is activated when pressed
    } else if (Check_UART_Data(BLE_UART_Buffer, "!B61")) {


        if (current_state == STOPPED) {

            current_state       = REVERSE;

            command             = &Motor_Backward;
            Desired_RPM_Left   += BACKWARD_SPEED;
            Desired_RPM_Right  += BACKWARD_SPEED;

            LED2_Output(RGB_LED_PINK);

        // if reversed, stop and reset desired RPM
        } else if (current_state == REVERSE) {

            current_state       = STOPPED;

            command             = &Motor_Backward;
            Desired_RPM_Left    = 0;
            Desired_RPM_Right   = 0;

            LED2_Output(RGB_LED_OFF);

        }

        // common: state transition from STOPPED to REVERSE and vice versa
        // allowed only.


    // 7: LEFT is activated when pressed
    } else if (Check_UART_Data(BLE_UART_Buffer, "!B71")) {

        // update duty cycles
        Desired_RPM_Left   -= ROTATION_SPEED;
        Desired_RPM_Right  += ROTATION_SPEED;

        LED2_Output(RGB_LED_YELLOW);


    // 8: RIGHT is activated when pressed
    } else if (Check_UART_Data(BLE_UART_Buffer, "!B81")) {

        // update duty cycles
        Desired_RPM_Left   += ROTATION_SPEED;
        Desired_RPM_Right  -= ROTATION_SPEED;

        LED2_Output(RGB_LED_BLUE);

    }


    // Move the motors using the function pointer "command" with the updated duty cycle
    if (command != NULL) {

        command(clamp(Desired_RPM_Left,  0, TIMER_A0_PERIOD_CONSTANT),
                clamp(Desired_RPM_Right, 0, TIMER_A0_PERIOD_CONSTANT));

    }

}

uint16_t clamp(
        uint16_t value,
        uint16_t min,
        uint16_t max)
{
    if (value > max) return max;

    if (value > min) return value;

    return min;
}
