

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
#include "inc/I2C.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/GPIO.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/vl53l5cx_api.h"


//#define TRUE_MAIN 1

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




#if defined TRUE_MAIN // -----------------------------------------------------------------------


/**
 * main.c
 */
void main(void) {
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, )


}

#else // #if defined TRUE_MAIN -----------------------------------------------------------------------



int main(void)
{
    // stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;


    // Initialize the 48 MHz Clock
    Clock_Init48MHz();


    // Initialize the built-in red LED and the RGB LEDs
    LED1_Init();
    LED2_Init();


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

    config_I2C();


    Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

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
    if(!isAlive || status)
    {
        printf("VL53L5CX not detected at requested address\n");
        return status;
    }

    /* (Mandatory) Init VL53L5CX sensor */
    status = vl53l5cx_init(&Dev);
    if(status)
    {
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

#endif // #if defined TRUE_MAIN
