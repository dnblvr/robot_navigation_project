/*
 * RPLiDAR_C1.c
 *
 */

#include "inc/RPLiDAR_C1.h"


/**
 * @note this is where the cfg struct is actually defined
 */
RPLiDAR_Config cfg = {.skip_factor = 4};


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


void initialize_RPLiDAR_C1(
        RPLiDAR_Config *config,
        uint8_t        *RPLiDAR_RX_Data)
{

    // @todo: process for using SCAN command:
    //  1. turn on the UART TX/RX interrupt enable
    //  2. record our out-characters onto an array until it fills up?
    //  3. turn off the UART TX/RX interrupt enable
    //  4. process the data

    // configure the structure
    configure_RPLiDAR_struct(config, RPLiDAR_RX_Data);


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



void process_rplidar_data(
        uint8_t RX_DATA[OUTPUT_BUFFER],
        float   out[OUTPUT_BUFFER][3])
{

    int i, j;

    float distance_angle[2] = {0};

    for (i = 0; i < OUTPUT_BUFFER; i++) {

        uint8_t is_nonzero;

        // convert the data to distance and angle
        is_nonzero = to_angle_distance( RX_DATA + i, distance_angle );


        printf("%3.2f %3.2f\n", distance_angle[0], distance_angle[1]);

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
//        fprintf(stdout, "%3i %10.2f %10.2f\n",
//                j, out[j][0], out[j][1]);

//        ++j;

    }

}
