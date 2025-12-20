/**
 * @file    RPLiDAR_C1.c
 * @brief   higher-level functions that are used to interface with the C1
 *              regardless of the microcontroller involved
 */

#include "./inc/RPLiDAR_C1.h"


/**
 * @note local definition of RPLiDAR `cfg` struct
 */
RPLiDAR_Config cfg;

extern RPLiDAR_Config* config;

extern uint8_t*  RX_POINTER;

extern uint32_t* INTERM_POINTER;


// Single-Request, No-Response
const No_Response  STOP = {0x25,  0,  10},
                  RESET = {0x40,  0, 500};


const Single_Response \
// Single-Request, Single-Response
               GET_INFO = {0x50, 20},
             GET_HEALTH = {0x52,  8},
         GET_SAMPLERATE = {0x59,  0},
         GET_LIDAR_CONF = {0x84,  0},

// Single-Request, Multiple-Response
                   SCAN = {0x20,  5},
           EXPRESS_SCAN = {0x82,  5};


/**
 * @brief
 */
//typedef void (*Timer_Command)(void);


// ----------------------------------------------------------------------------
//
//  INITIALIZATION
//
// ----------------------------------------------------------------------------

void Initialize_RPLiDAR_C1(const RPLiDAR_Config*    config) {

    // @details The process for using SCAN command:
    //  1. turn on the UART TX/RX interrupt enable
    //  2. set up C1 using the RESET, GET_HEALTH, and SCAN commands for
    //      continuous scan
    //  3. read characters at a set time determined by an external timer
    //  4. record our out-characters onto an array until it fills up
    //  5. process the data
    //  6. go to step 3

    // configure the structure
    Configure_RPLiDAR_Struct(config);


    // Initialize UART communications
    EUSCI_A2_UART_Init();


    // printf("SRNR: STOP\n");
    Single_Request_No_Response(&STOP);
    Clock_Delay1ms(1);


    // printf("SRNR: RESET\n");
    Single_Request_No_Response(&RESET);
    Clock_Delay1ms(1);


    // printf("SRSR: GET_HEALTH\n");
    Single_Request_Single_Response(&GET_HEALTH, RX_POINTER);
    Clock_Delay1ms(1);


    // printf("SRMR: SCAN\n");
    Single_Request_Single_Response(&SCAN, RX_POINTER);
    Clock_Delay1ms(200);

}


void Process_RPLiDAR_Data(PointCloud*   output) {

#define STATEMENT (j == 2 || j == 4)

    // persistent counter
    static uint32_t j = 0;

    // for-loop counters
    uint32_t i, k;
    uint32_t limits = config->interm_buffer_counter - 1;


#ifdef DEBUG_OUTPUT
    print_buffer_u32(STATEMENT, limits);
#endif // #ifdef DEBUG_OUTPUT


    // sort packed data by angle; expects angle as most MS 2 bytes
#if (SKIP_FACTOR != 1)
    binary_insertion_u32(INTERM_POINTER, limits);
#endif

    k = 0;
    for (i = 0; i < limits; i += SKIP_FACTOR) {

        float angle_r   = (DTR/64.f) * (INTERM_POINTER[i] >>     16);
        float distance  = (1.f/ 4.f) * (INTERM_POINTER[i] &  0xFFFF);

#ifdef DEBUG_OUTPUT
//        if (STATEMENT) {
//            printf("  %i\t%3.2f rad. @ %5.2f mm\n",
//                   i, angle_r, distance);
//        }
#endif

        /**
         * @note The angle is negated i.e....
         *     > cosf(-angle_r) =>  cosf(angle_r)
         *     > sinf(-angle_r) => -sinf(angle_r)
         *  ... to correct the flipped output on `pointcloud_visualizer`
         *
         * @note if the scanner is mounted upside down, undo this modification
         */
        output->points[k].x = distance *  cosf(angle_r);
        output->points[k].y = distance * -sinf(angle_r);

        k++;
    }

    output->num_pts = k;

    j++;

}


// ----------------------------------------------------------------------------
//
//  HIGHER-LEVEL RPLiDAR FUNCTIONS
//
// ----------------------------------------------------------------------------

void Single_Request_No_Response(
        const No_Response* cmd)
{

    EUSCI_A2_UART_OutChar(         0xA5 );    // start flag
    EUSCI_A2_UART_OutChar( cmd->command );


    // wait for the RPLiDAR to process the command
    Clock_Delay1ms(cmd->byte_length);

}


uint8_t Single_Request_Single_Response(
        const Single_Response*  cmd,
              uint8_t           RX_DATA_BUFFER[])
{

    char start_flag_1, start_flag_2;

//    uint32_t result;
//    uint8_t send_mode = 0, datatype = 0;


    // send commands
    EUSCI_A2_UART_OutChar(         0xA5 );    // start flag
    EUSCI_A2_UART_OutChar( cmd->command );

    // read the response descriptor from the RPLiDAR C1
    start_flag_1    = EUSCI_A2_UART_InChar();
    start_flag_2    = EUSCI_A2_UART_InChar();


#ifdef RPLIDAR_DEBUG
    printf("0x%02X; 0x%02X\n", start_flag_1, start_flag_2);
#endif

    // if the response descriptor identifies the correct sequence
    if ( (start_flag_1 == 0xA5) && (start_flag_2 == 0x5A) ) {

        // for-loop is too slow to use on the RPLiDAR C1
        RX_DATA_BUFFER[0]   = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[1]   = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[2]   = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[3]   = EUSCI_A2_UART_InChar();
        RX_DATA_BUFFER[4]   = EUSCI_A2_UART_InChar();

//        result  =   (uint32_t)(RX_DATA_BUFFER[3] << 22)
//                  | (uint32_t)(RX_DATA_BUFFER[2] << 14)
//                  | (uint32_t)(RX_DATA_BUFFER[1] <<  6)
//                  | (uint32_t)(RX_DATA_BUFFER[0] >>  2);

        // Extract the send mode bits and the datatype (2 bits) from the last
        // response byte
//        send_mode   = (RX_DATA_BUFFER[3] << 2) & 0x3;
//        datatype    =  RX_DATA_BUFFER[4];

#ifdef RPLIDAR_DEBUG

        print_binary_sequence(RX_DATA_BUFFER, cmd->byte_length);

        printf("RPLiDAR Response: 0x%08X\n", result);
        printf("RPLiDAR Response type: 0x%02X\n", send_mode);
        printf("RPLiDAR Response datatype: 0x%02X\n", datatype);
#endif

        return 1;


    // if the response descriptor IDs the incorrect sequence
    } else {

#ifdef RPLIDAR_DEBUG
        printf("Invalid Response\n");
#endif

        // wait for some time before resetting the RPLiDAR
        Clock_Delay1ms(5000);
        return 0;
    }
}


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

#if (SKIP_FACTOR != 1)

void binary_insertion_u32(
        uint32_t    polar_data[],
        uint32_t    point_count)
{
    // 4
    uint32_t i, j, left, right, mid;
    uint32_t key;

    for (i = 1; i < point_count; i++) {
        key = polar_data[i];
        left = 0;
        right = i;

        // Binary search for insertion position
        while (left < right) {
            mid = left + (right - left) / 2;
            if (polar_data[mid] <= key)
                left = mid + 1;
            else
                right = mid;
        }

        // Shift elements to make room
        for (j = i; j > left; j--) {
            polar_data[j] = polar_data[j - 1];
        }

        // Insert key at correct position
        polar_data[left] = key;
    }
}

#endif  // #if (SKIP_FACTOR != 1)



#ifdef DEBUG_OUTPUT

void print_buffer_u32(uint8_t boolean, uint32_t limits) {

    uint32_t i;

    if (boolean) {
        for (i = 0; i < limits; i++)
            printf("  0x%.8X\n", INTERM_POINTER[i]);

        printf("\n\tlimits = %5u\n\n", limits);
    }
}

#endif


