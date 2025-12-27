/**
 * @file GPS_UART.h
 * @brief Header file for the GPS UART driver with NMEA parsing.
 *
 * This file contains the function declarations for GPS UART communication
 * and NMEA sentence parsing capabilities.
 *
 * @note Uses EUSCI_A1 module for GPS communication at 9600 baud
 * @note Pins P9.6 (RX) and P9.7 (TX) are used for GPS UART
 *
 * @author Gian Fajardo
 */

#ifndef GPS_A1_UART_H_
#define GPS_A1_UART_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "msp.h"

// GPS UART Buffer sizes
#define GPS_BUFFER_SIZE         82      // NMEA sentences max 82 characters
#define GPS_MAX_FIELDS          20      // Max fields in NMEA sentence

/**
 * @brief NMEA sentence types
 */
typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_GPGGA,     // Global Positioning System Fix Data
    NMEA_GPRMC,     // Recommended Minimum Specific GPS/Transit Data
    NMEA_GPGSV,     // GPS Satellites in View
    NMEA_GPGSA,     // GPS DOP and Active Satellites
    NMEA_GPVTG      // Track Made Good and Ground Speed
} NMEA_Type;

/**
 * @brief GPS Fix status
 */
typedef enum {
    GPS_NO_FIX = 0,
    GPS_FIX,
    GPS_DIFFERENTIAL_FIX
} GPS_Fix_Status;


/**
 * @brief GPS Data structure
 */
typedef struct {

    // Position data
    float latitude;         // Decimal degrees
    float longitude;        // Decimal degrees
    float altitude;         // Meters above sea level
    
    // Time data
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    
    // Quality data
    GPS_Fix_Status fix_status;
    uint8_t satellites_used;
    float hdop;             // Horizontal Dilution of Precision
    
    // Speed and course
    float speed_kmh;        // Speed in km/h
    float course_degrees;   // Course over ground in degrees
    
    // Status flags
    uint8_t data_valid;     // 1 if GPS data is valid
} GPS_Data;



void GPS_UART_Init(void);
char GPS_UART_InChar(void);
void GPS_UART_OutChar(char letter);
void GPS_UART_OutString(char *pt);

// NMEA parsing functions
uint8_t GPS_Read_NMEA_Sentence(char *buffer, uint16_t max_length);
NMEA_Type GPS_Parse_NMEA_Type(const char *sentence);
uint8_t GPS_Parse_NMEA_Sentence(const char *sentence, GPS_Data *gps_data);
uint8_t GPS_Calculate_Checksum(const char *sentence);
uint8_t GPS_Verify_Checksum(const char *sentence);

// Utility functions
float GPS_Convert_DDMM_to_Decimal(const char *ddmm_str, char direction);
void GPS_Print_Data(const GPS_Data *gps_data);

// Global GPS data
extern GPS_Data current_gps_data;

#endif /* GPS_A1_UART_H_ */
