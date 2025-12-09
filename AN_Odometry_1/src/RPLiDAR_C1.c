/**
 * @file RPLiDAR_C1.c
 * @
 */

#include "./inc/RPLiDAR_C1.h"

#include "../inc/Project_Config.h"


/**
 * @note local definition of RPLiDAR cfg struct
 */
RPLiDAR_Config cfg = {.skip_factor = 2};


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



// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------


static uint8_t to_distance_angle(
        const uint8_t*  base_ptr,
        float           distance_angle[2])
{

    uint32_t    lower   = 0,    upper       = 0,
                angle   = 0,    distance    = 0;

    // takes up the first seven bits of byte 1
    lower = *(base_ptr + 1) & 0x7F;

    angle       = ( *(base_ptr + 2) << 7 ) | lower;
    distance    = ( *(base_ptr + 4) << 8 ) | ( *(base_ptr + 3));


    // early return if the distance is zero
    if (distance == 0)
        return 0;

    distance_angle[0]   = (float)(distance / 4.f);
    distance_angle[1]   = (float)((DEG_TO_RAD * angle) / 64.f);


#ifdef DEBUG_OUTPUTS
    // Debug: Show raw bytes, distance and angle
    // Only print if distance is 0 (invalid) or for first few points
//    static int debug_count = 0;
//    if (distance == 0 || debug_count < 5) {
//        printf("RAW[%02X %02X %02X %02X %02X] -> dist=%lu ang=%.1f deg\n",
//               base_ptr[0], base_ptr[1], base_ptr[2], base_ptr[3], base_ptr[4],
//               distance, (float)angle / 64.f);
//        debug_count++;
//        if (debug_count >= 100) debug_count = 0;  // Reset periodically
//    }
#endif

    return 1;
}


void insertion(
        float   polar_data[][2],
        int     point_count)
{
    int i, j;
    float key_dist, key_angle;

    for (i = 1; i < point_count; i++) {
        // Store the current element
        key_dist    = polar_data[i][0];
        key_angle   = polar_data[i][1];

        j = i - 1;

        // Move elements greater than key one position ahead
        while (j >= 0 && polar_data[j][1] > key_angle) {
            polar_data[j + 1][0] = polar_data[j][0];
            polar_data[j + 1][1] = polar_data[j][1];
            j--;
        }

        // Insert key at correct position
        polar_data[j + 1][0] = key_dist;
        polar_data[j + 1][1] = key_angle;
    }

}



int partition(
        float   polar_data[][2],
        int     low,
        int     high)
{
    // counter
    int j;

    int i = low - 1;
    float temp_dist, temp_angle;


    // Use last element as pivot
    float pivot_angle   = polar_data[high][1];


    for (j = low; j < high; j++) {

        if (polar_data[j][1] <= pivot_angle) {
            i++;

            // Swap distance
            temp_dist = polar_data[i][0];
            polar_data[i][0] = polar_data[j][0];
            polar_data[j][0] = temp_dist;

            // Swap angle
            temp_angle = polar_data[i][1];
            polar_data[i][1] = polar_data[j][1];
            polar_data[j][1] = temp_angle;
        }
    }

    // Swap pivot into correct position
    temp_dist   = polar_data[i + 1][0];
    polar_data[i + 1][0]    = polar_data[high][0];
    polar_data[high ][0]    = temp_dist;

    temp_angle  = polar_data[i + 1][1];
    polar_data[i + 1][1]    = polar_data[high][1];
    polar_data[high ][1]    = temp_angle;

    return i + 1;
}


void quicksort(float polar_data[][2], int low, int high)
{
    if (low < high) {
        int pi = partition(polar_data, low, high);

        // Recursively sort elements before and after partition
        quicksort(polar_data, low, pi - 1);
        quicksort(polar_data, pi + 1, high);
    }
}


// -------------------------------------------------------------------------------------
//
//  INITIALIZATION
//
// -------------------------------------------------------------------------------------


void Initialize_RPLiDAR_C1(
        const RPLiDAR_Config*   config,
        uint8_t*                RPLiDAR_RX_Data)
{

    // @details The process for using SCAN command:
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




void Process_RPLiDAR_Data(
        const uint8_t   RX_DATA[RPLiDAR_UART_BUFFER_SIZE],
        float           out[OUTPUT_BUFFER][3],
        uint32_t*       point_count)
{
    // counter
    int i = 0;

    float distance_angle[2] = {0};
    
    // Temporary array to hold all polar coordinates before sorting and skipping
    float polar_data[INTERMEDIARY_BUFFER][2];  // [distance, angle]
    int polar_count = 0;  // Number of valid polar points before skipping


    // Conversion index
    int output_index;


#ifdef DEBUG_OUTPUTS

    float       min_angle =  INFINITY,
                max_angle = -INFINITY;
#endif

    // Extract all distance-angle pairs
    for (i = 0; i < RPLiDAR_UART_BUFFER_SIZE; i+= MSG_LENGTH) {

        // valid until proven otherwise
        uint8_t is_valid = 1;

        // convert the data to distance and angle
        is_valid    = to_distance_angle( RX_DATA + i, distance_angle );

        if (!is_valid)
            continue;
        
        // Store in temporary array
        polar_data[polar_count][0] = distance_angle[0];  // distance
        polar_data[polar_count][1] = distance_angle[1];  // angle
        polar_count++;
    }
    

    // sort by angle
    quicksort(polar_data, 0, polar_count - 1);
    // insertion(polar_data, polar_count);
    

    // Convert sorted polar coordinates to Cartesian with skipping
    output_index = 0;
    for (   i = 0;
            i < polar_count && output_index < OUTPUT_BUFFER;
            i += SKIP_FACTOR)
    {
        
        // Retrieve sorted distance and angle
        distance_angle[0] = polar_data[i][0];
        distance_angle[1] = polar_data[i][1];

#ifdef DEBUG_OUTPUTS

        // Track min and max angles
        if (distance_angle[1] < min_angle)
            min_angle = distance_angle[1];
        if (distance_angle[1] > max_angle)
            max_angle = distance_angle[1];

#endif

        // convert the polar coordinates to Cartesian coordinates
        polar_to_cartesian( distance_angle, out[output_index] );

#ifdef RPLIDAR_DEBUG
        // print the distance and angle
        fprintf(stdout, "%i\t%3.2f rad. @ %5.2f mm\n",
                output_index, distance_angle[1], distance_angle[0]);

#endif

        output_index++;
    }

    // Return valid number of points written to output buffer
    *point_count = output_index;

#ifdef DEBUG_OUTPUTS

    // printf("Scan angles: min=%.2f max=%.2f rad (%.1f to %.1f deg), %d points sorted, %d output\n",
    //        min_angle, max_angle,
    //        min_angle * 57.2958f, max_angle * 57.2958f,
    //        point_count, output_index);

#endif

}

