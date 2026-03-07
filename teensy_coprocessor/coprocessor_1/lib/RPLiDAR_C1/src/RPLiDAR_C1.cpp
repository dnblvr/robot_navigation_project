/**
 * @file RPLiDAR_C1.cpp
 * @brief Higher-level RPLiDAR C1 functions — platform-agnostic.
 *
 * @details Direct port of RPLiDAR_C1.c from the MSP432 FW_RPLiDAR_C1
 *  project.  The only substitutions made:
 *
 *   | MSP432                        | This file                     |
 *   |-------------------------------|-------------------------------|
 *   | EUSCI_A2_UART_Init()          | RPLiDAR_UART_Init()           |
 *   | EUSCI_A2_UART_OutChar(b)      | RPLiDAR_UART_OutChar(b)       |
 *   | EUSCI_A2_UART_InChar()        | RPLiDAR_UART_InChar()         |
 *   | Clock_Delay1ms(n)             | delay(n)                      |
 *
 * @author Gian Fajardo
 */

#include "RPLiDAR_C1.h"


// --------------------------------------------------------------------------
//  config, RX_POINTER and INTERM_POINTER are defined in
//  RPLiDAR_Arduino_UART.cpp and declared extern in RPLiDAR_Arduino_UART.h
//  (pulled in via RPLiDAR_C1.h).  No local storage needed here.
// --------------------------------------------------------------------------


// ============================================================================
//
//  INITIALISATION
//
// ============================================================================

void Initialize_RPLiDAR_C1(const C1_States* config_in)
{
    // ---- Command descriptors (identical to MSP432 version) ---------------

    const No_Response  STOP  = {0x25, 0,  10};
    const No_Response  RESET = {0x40, 0, 500};

    const Single_Response GET_HEALTH = {0x52, 8};
    const Single_Response SCAN       = {0x20, 5};

    // ---- Configure the shared state struct --------------------------------
    Configure_RPLiDAR_Struct(config_in);

    // ---- Open UART --------------------------------------------------------
    RPLiDAR_UART_Init();

    // ---- Protocol sequence ------------------------------------------------
    Single_Request_No_Response(&STOP);
    delay(1);

    Single_Request_No_Response(&RESET);
    delay(1);

    Single_Request_Multiple_Response(&GET_HEALTH, RX_POINTER);
    delay(1);

    Single_Request_Multiple_Response(&SCAN, RX_POINTER);
    delay(200);
}


// ============================================================================
//
//  DATA PROCESSING
//
// ============================================================================

void Process_RPLiDAR_Data(PointCloud* output)
{
#define STATEMENT (j == 2 || j == 4)

    // persistent counter (mirrors original)
    static uint32_t j = 0;

    uint32_t i, k;
    uint32_t limits = config->interm_buffer_counter - 1;

#ifdef DEBUG_OUTPUT
    print_buffer_u32(STATEMENT, limits);
#endif

    // Optionally sort packed data by angle (MS 16 bits carry angle)
#if (SKIP_FACTOR != 1)
    binary_insertion_u32(INTERM_POINTER, limits);
#endif

    k = 0;
    for (i = 0; i < limits; i += SKIP_FACTOR) {

        float angle_r  = (((float)M_PI / 180.f) / 64.f)
                         * (float)(INTERM_POINTER[i] >> 16);

        float distance = 0.25f
                         * (float)(INTERM_POINTER[i] & 0xFFFF);

        /**
         * @note The angle is negated to correct the flipped output on the
         *   pointcloud_visualiser (same comment as the MSP432 version).
         *   Undo the negation if the scanner is mounted upside-down.
         */
        output->points[k].x = distance *  cosf(angle_r);
        output->points[k].y = distance * -sinf(angle_r);

        k++;
    }

    output->num_pts = k;

    j++;
}


// ============================================================================
//
//  PROTOCOL HELPERS
//
// ============================================================================

void Single_Request_No_Response(const No_Response* cmd)
{
    RPLiDAR_UART_OutChar(0xA5);
    RPLiDAR_UART_OutChar(cmd->command);

    delay(cmd->time);
}


uint8_t Single_Request_Multiple_Response(
        const Single_Response*  cmd,
              uint8_t           RX_DATA_BUFFER[])
{
    uint8_t start_flag_1, start_flag_2;

    // Send command
    RPLiDAR_UART_OutChar(0xA5);
    RPLiDAR_UART_OutChar(cmd->command);

    // Read response descriptor
    start_flag_1 = RPLiDAR_UART_InChar();
    start_flag_2 = RPLiDAR_UART_InChar();

#ifdef RPLIDAR_DEBUG
    Serial.printf("resp: 0x%02X 0x%02X\n", start_flag_1, start_flag_2);
#endif

    if ((start_flag_1 == 0xA5) && (start_flag_2 == 0x5A)) {

        // Read exactly 5 descriptor bytes (same unrolled loop as MSP432)
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

void binary_insertion_u32(uint32_t polar_data[], uint32_t point_count)
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
