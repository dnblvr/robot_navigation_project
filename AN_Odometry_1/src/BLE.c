/*
 * BLE.c
 *
 */

#include <string.h>
#include "../inc/BLE.h"


// ----------------------------------------------------------------------------
//
//  MOTOR-OPERATION FUNCTIONS
//
// ----------------------------------------------------------------------------


// Desired RPM for the left wheel
extern uint16_t Desired_RPM_Left    = 70;

// Desired RPM for the right wheel
extern uint16_t Desired_RPM_Right   = 70;

int begin_state = 0;


void Motor_Stop_Wrapper(uint16_t left_speed, uint16_t right_speed) {

    Motor_Stop();
}


/**
 * @brief local instance of message_length globalized from BLE_A3_UART.c
 */
extern volatile int message_length;



void Process_BLE_UART_Data(volatile char BLE_UART_Buffer[])
{
//    printf("len = %2d\n", message_length);

    // internal counter variable
    int j;
    MotorCommand command;

    if (message_length < 4)
        return;

    printf("BLE UART Data: ");

    for (j = 0; j < message_length; j++) {
        printf("%c", BLE_UART_Buffer[j]);
    }

    printf("\n");


    command = NULL;

    if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B11")) {

        command             = &Motor_Stop_Wrapper;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;


    // 2: begin operation
    } else
    if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B21")) {

        command             = &Motor_Stop_Wrapper;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        begin_state = 1;

        LED2_Output(RGB_LED_WHITE);


    // 5: UP is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B51")) {

        command             = &Motor_Forward;
        Desired_RPM_Left    = forward_speed;
        Desired_RPM_Right   = forward_speed;

        LED2_Output(RGB_LED_GREEN);


    // 6: DOWN is activated when pressed
    } else
    if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B61")) {

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

//        LED2_Output(RGB_LED_YELLOW);


    // 8: RIGHT is activated when pressed
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "!B81")) {

        // update duty cycles
        command             = &Motor_Right;
        Desired_RPM_Left    = rotation_speed;
        Desired_RPM_Right   = rotation_speed;

//        LED2_Output(RGB_LED_BLUE);


    } else {

        command             = &Motor_Stop_Wrapper;
        Desired_RPM_Left    = 0;
        Desired_RPM_Right   = 0;

        LED2_Output(RGB_LED_OFF);

    }

    // to apply the command only once, clear the buffer memory
    memset((void*)BLE_UART_Buffer, 0, message_length);
    message_length = 0;


    // Move the motors using the function pointer "command" with the updated duty cycle
    if (command != NULL) {

        command(Desired_RPM_Left, Desired_RPM_Right);

    } else {

        Motor_Stop();
    }

}
