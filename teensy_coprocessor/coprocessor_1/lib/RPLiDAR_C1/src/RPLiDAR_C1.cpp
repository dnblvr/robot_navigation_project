/**
 * @file RPLiDAR_C1.cpp
 * @brief platform-agnostic higher-level RPLiDAR C1 functions.
 *
 * @details Direct port of RPLiDAR_C1.c from the MSP432 FW_RPLiDAR_C1
 *  project.
 *
 *
 * @author Gian Fajardo
 */

#include "RPLiDAR_C1.h"


// ----------------------------------------------------------------------------
//
//  INITIALIZATION
//
// ----------------------------------------------------------------------------

void Initialize_RPLiDAR_C1(C1_States* config_in)
{
    // ---- Command descriptors ---------------

    const Single_Response   GET_HEALTH  = {0x52, 8};
    const Single_Response   SCAN        = {0x20, 5};

    // ---- Configure the shared state struct --------------------------------
    Configure_RPLiDAR_Struct(config_in);

    // ---- Open UART --------------------------------------------------------
    RPLiDAR_UART_Init();

    // ---- Protocol sequence ------------------------------------------------
    Single_Request_No_Response(STOP);

    Single_Request_No_Response(RESET);

    Single_Request_Multiple_Response(GET_HEALTH, RX_POINTER);

    Single_Request_Multiple_Response(SCAN, RX_POINTER);
    delay(200);
}


// ============================================================================
//
//  DATA PROCESSING
//
// ============================================================================

void Process_RPLiDAR_Data(
        const se2_t         pose,
              PointCloud*   output)
{

#ifdef DEBUG_OUTPUT
    start_free_running_timer();
#endif

    #define STATEMENT (j == 2 || j == 4)

    if (config->current_state != PROCESSING)
        return;

    float c, s;
    if (pose.theta < 1e-3f) {
        c = 1.f;  s = 0.f;

    } else {

    #ifdef __FAST_MATH__
        arm_sin_cos_f32(pose.theta, &s, &c);
    #else 
        c = cosf(pose.theta);
        s = sinf(pose.theta);
    #endif

    }

    // Pre-compute the SE(2) transformation matrix for the current `action`
    float T[TOTAL] = {
            c,  -s,   pose.x,
            s,   c,   pose.y,
            0.f, 0.f, 1.f};

    // persistent counter
    static uint32_t j = 0;

    uint32_t i, k;
    uint32_t limits = config->interm_buffer_counter - 1;

#ifdef DEBUG_OUTPUT
    // print_buffer_u32(STATEMENT, limits);
#endif

    // Optionally sort packed data by angle (MS 16 bits carry angle)
#if (SKIP_FACTOR != 1)
    binary_insertion_u32(INTERM_POINTER, limits);
#endif


/**
 * @brief fast math implementation: 176 us / 91 us processing for 400 points on the Teensy 4.0, contributing to an almost 2x speedup
 */
#ifdef __FAST_MATH__

    #define CONVERT_DEG (1.f / 64.f)

    float32_t distance_array[limits]    = {0};
    float32_t angle                     =  0;
    float32_t x_array[limits]           = {0};
    float32_t y_array[limits]           = {0};

    k = 0;
    for (i = 0; i < limits; i += SKIP_FACTOR) {
        // distance_array[i]   = 0.25f * (float)(INTERM_POINTER[i] & 0xFFFF);
        // angle_array[i]      = CONVERT_DEG * (float)(INTERM_POINTER[i] >> 16);
        
        distance_array[k]   = 0.25f * (float)(INTERM_POINTER[i] & 0xFFFF);
        angle               = CONVERT_DEG * (float)(INTERM_POINTER[i] >> 16);

        // PRINTF("angle_array[%u] = %f\n", k, angle_array[i]);

        arm_sin_cos_f32(angle, &y_array[k], &x_array[k]);
        k++;
    }

    output->num_pts = k;

    arm_mult_f32(x_array, distance_array, x_array, output->num_pts);
    arm_mult_f32(y_array, distance_array, y_array, output->num_pts);


    for (i = 0; i < output->num_pts; i++) {

        // Transform point from sensor frame to global frame using the SE(2)
        // transformation matrix T
        output->points[i].x = (float32_t)(     T[R_00]*x_array[i]
                                           +   T[R_01]*y_array[i]
                                           +   T[T_x_]);
        output->points[i].y = (float32_t)(     T[R_10]*x_array[i]
                                           +   T[R_11]*y_array[i]
                                           +   T[T_y_]);
    }

#else

    k = 0;
    for (i = 0; i < limits; i += SKIP_FACTOR) {

        float distance =    0.25f
                         * (float)(INTERM_POINTER[i] & 0xFFFF);

        float angle_r  =   ((M_PI / 180.f) / 64.f)
                         * (float)(INTERM_POINTER[i] >> 16);

        /**
         * @note The y-component is negated to correct the flipped output on
         *  `pointcloud_visualizer`. Undo the negation if the scanner is mounted
         *  upside-down.
         */
        float x = distance *  cosf(angle_r);
        float y = distance * -sinf(angle_r);

        // Transform point from sensor frame to global frame using the SE(2)
        // transformation matrix T
        output->points[k].x = T[R_00]*x + T[R_01]*y + T[T_x_];
        output->points[k].y = T[R_10]*x + T[R_11]*y + T[T_y_];

    #ifdef DEBUG_OUTPUT
        // print all angles
        // Serial.printf("\t%u\n", (uint32_t)((INTERM_POINTER[i] >> 22)));
    #endif

        k++;
    }

    output->num_pts = k;

#endif  // __FAST_MATH__


    j++;

#ifdef DEBUG_OUTPUT
    Serial.printf("\n");

    uint32_t dt = get_elapsed_time_us();

    PRINTF("Processing time: %u us\n", dt);
#endif

}


// ============================================================================
//
//  PROTOCOL HELPERS
//
// ============================================================================

void Single_Request_No_Response(const No_Response cmd)
{
    RPLiDAR_UART_OutChar(0xA5);
    RPLiDAR_UART_OutChar(cmd.command);

    delay(cmd.time);
}


uint8_t Single_Request_Multiple_Response(
        const Single_Response   cmd,
              uint8_t           RX_DATA_BUFFER[])
{
    uint8_t start_flag_1, start_flag_2;

    // Send command
    RPLiDAR_UART_OutChar(0xA5);
    RPLiDAR_UART_OutChar(cmd.command);

    // Read response descriptor
    start_flag_1 = RPLiDAR_UART_InChar();
    start_flag_2 = RPLiDAR_UART_InChar();

#ifdef RPLIDAR_DEBUG
    Serial.printf("resp: 0x%02X 0x%02X\n", start_flag_1, start_flag_2);
#endif

    if ((start_flag_1 == 0xA5) && (start_flag_2 == 0x5A)) {

        // Read exactly 5 descriptor bytes
        RX_DATA_BUFFER[0] = RPLiDAR_UART_InChar();
        RX_DATA_BUFFER[1] = RPLiDAR_UART_InChar();
        RX_DATA_BUFFER[2] = RPLiDAR_UART_InChar();
        RX_DATA_BUFFER[3] = RPLiDAR_UART_InChar();
        RX_DATA_BUFFER[4] = RPLiDAR_UART_InChar();

        return 1;

    } else {

#ifdef RPLIDAR_DEBUG
        Serial.println("RPLiDAR: invalid response descriptor");
#endif
        delay(5000);
        return 0;

    }

}


// ============================================================================
//
//  HELPER FUNCTIONS
//
// ============================================================================

#if (SKIP_FACTOR != 1)

void binary_insertion_u32(
        uint32_t    polar_data[],
        uint32_t    point_count)
{
    uint32_t i, j, left, right, mid;
    uint32_t key;

    for (i = 1; i < point_count; i++) {

        key   = polar_data[i];
        left  = 0;
        right = i;

        while (left < right) {
            mid = left + (right - left) / 2;
            if (polar_data[mid] <= key)
                left = mid + 1;
            else
                right = mid;
        }

        for (j = i; j > left; j--)
            polar_data[j] = polar_data[j - 1];

        polar_data[left] = key;
    }
}

#endif  // SKIP_FACTOR != 1


#ifdef DEBUG_OUTPUT

void print_buffer_u32(uint8_t boolean, uint32_t limits)
{
    uint32_t i;

    if (boolean) {
        for (i = 0; i < limits; i++)
            Serial.printf("  0x%.8X\n", INTERM_POINTER[i]);

        Serial.printf("\n\tlimits = %5u\n\n", limits);
    }
}

#endif
