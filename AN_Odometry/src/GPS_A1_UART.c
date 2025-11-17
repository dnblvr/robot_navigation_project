/**
 * @file GPS_UART.c
 * @brief Source code for the GPS UART driver with NMEA parsing.
 *
 * This file contains the function definitions for GPS UART communication
 * and NMEA sentence parsing.
 *
 * @note Uses EUSCI_A3 module (P9.6 RX, P9.7 TX) at 9600 baud
 *
 * @author Gian Fajardo
 */

#include "../inc/GPS_A1_UART.h"

// Global GPS data structure
GPS_Data current_gps_data = {0};

/**
 * @brief Initialize GPS UART (EUSCI_A3) for 9600 baud communication
 */
void GPS_UART_Init(void) {
    


    // Configure pins P2.2 (PM_UCA2RXD) and P2.3 (PM_UCA2TXD) to use the primary module function:
    //    - by  setting  Bits 3 and 2 in the SEL0 register for P2
    //    - and clearing Bits 3 and 2 in the SEL1 register for P2
    P2->SEL0 |=  0x0C;
    P2->SEL1 &= ~0x0C;


    // Hold the EUSCI_A1 module in the reset state by setting the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A1->CTLW0    |=  0x01;


    // Clear all of the bits in the Modulation Control Word (MCTLW) register
    EUSCI_A1->MCTLW    &= ~0xFF;


    // Disable the parity bit by clearing the UCPEN bit (Bit 15) in the CTLW0 register
    EUSCI_A1->CTLW0    &= ~0x8000;


    // Select odd parity for the parity bit by clearing the UCPAR bit (Bit 14) in the CTLW0 register
    // Note that the UCPAR bit is not used when parity is disabled
    EUSCI_A1->CTLW0    &= ~0x4000;


    // Set the bit order to Most Significant Bit (MSB) first by setting the UCMSB bit (Bit 13) in
    // the CTLW0 register
//    EUSCI_A1->CTLW0    |=  0x2000;

    // Set the bit order to Least Significant Bit (MSB) first by clearing the UCMSB bit (Bit 13) in
    // the CTLW0 register
    EUSCI_A1->CTLW0    &= ~0x2000;


    // Select 8-bit character length by clearing the UC7BIT bit (Bit 12) in the CTLW0 register
    EUSCI_A1->CTLW0    &= ~0x1000;


    // Select one stop bit by clearing the UCSPB bit (Bit 11) in the CTLW0 register
    EUSCI_A1->CTLW0    &= ~0x0800;


    // Enable UART mode by writing 00b to the UCMODEx field (Bits 10-9) in the CTLW0 register
    EUSCI_A1->CTLW0    &= ~0x0600;


    // Disable synchronous mode by clearing the UCSYNC bit (Bit 8) in the CTLW0 register
    EUSCI_A1->CTLW0    &= ~0x0100;


    // Configure the EUSCI_A1 module to use SMCLK as the clock source by
    // writing a value of 10b to the UCSSELx field (Bits 7 to 6) in the CTLW0 register
    EUSCI_A1->CTLW0    |=  0x00C0;



    // Set the baud rate value by writing to the UCBRx field (Bits 15 to 0)
    // in the BRW register
    // N = (Clock Frequency) / (Baud Rate) = (12,000,000 / 460,800) = 26.0416666667
    // Use only the integer part, so N = 26
    EUSCI_A1->BRW       =  26;  // 460,800



    // Disable the following interrupts by clearing the
    // corresponding bits in the IE register:
    // - Transmit Complete Interrupt (UCTXCPTIE, Bit 3)
    // - Start Bit Interrupt         (UCSTTIE,   Bit 2)
    EUSCI_A1->IE       &= ~0x0C;


    // Enable the following interrupts by setting the
    // corresponding bits in the IE register
    // - Transmit Interrupt (UCTXIE, Bit 1)
    // - Receive Interrupt  (UCRXIE, Bit 0)
    EUSCI_A1->IE       |=  0x03;


    // Release the EUSCI_A1 module from the reset state by clearing the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A1->CTLW0 &= ~0x01;

}

/**
 * @brief Read a character from GPS UART
 * @return Received character
 */
char GPS_UART_InChar(void) {

    // Check the Receive Interrupt flag (UCRXIFG, Bit 0)
    // in the IFG register and wait if the flag is not set
    // If the UCRXIFG is set, then the Receive Buffer (UCAxRXBUF) has
    // received a complete character
    while((EUSCI_A1->IFG & 0x01) == 0);


    // Return the data from the Receive Buffer (UCAxRXBUF)
    // Reading the UCAxRXBUF will reset the UCRXIFG flag
    return EUSCI_A1->RXBUF;
}

/**
 * @brief Send a character to GPS UART
 * @param letter Character to send
 */
void GPS_UART_OutChar(char letter) {

    // Check the Transmit Interrupt flag (UCTXIFG, Bit 1)
    // in the IFG register and wait if the flag is not set
    // If the UCTXIFG is set, then the Transmit Buffer (UCAxTXBUF) is empty
    while((EUSCI_A1->IFG & 0x02) == 0);


    // Write the data to the Transmit Buffer (UCAxTXBUF)
    // Writing to the UCAxTXBUF will clear the UCTXIFG flag
    EUSCI_A1->TXBUF = letter;
}

/**
 * @brief Send a string to GPS UART
 * @param pt Pointer to null-terminated string
 */
void GPS_UART_OutString(char *pt) {
    
    while (*pt) {
        GPS_UART_OutChar(*pt);
        pt++;
    }
}

/**
 * @brief Read a complete NMEA sentence from GPS
 * @param buffer Buffer to store the sentence
 * @param max_length Maximum buffer length
 * @return 1 if valid sentence received, 0 otherwise
 */
uint8_t GPS_Read_NMEA_Sentence(char *buffer, uint16_t max_length) {
    uint16_t index = 0;
    char c;
    
    // Clear buffer
    memset(buffer, 0, max_length);
    
    // Look for start of NMEA sentence ('$')
    do {
        c = GPS_UART_InChar();
    } while (c != '$' && index < max_length - 1);
    
    if (c != '$') return 0;
    
    // Store the '$' character
    buffer[index++] = c;
    
    // Read until end of sentence ('\n') or buffer full
    do {
        c = GPS_UART_InChar();
        buffer[index++] = c;
    } while (c != '\n' && index < max_length - 1);
    
    // Null terminate
    buffer[index] = '\0';
    
    // Verify we have a complete sentence
    if (c == '\n' && index > 6) {
        return GPS_Verify_Checksum(buffer);
    }
    
    return 0;
}

/**
 * @brief Parse NMEA sentence type
 * @param sentence NMEA sentence string
 * @return NMEA sentence type
 */
NMEA_Type GPS_Parse_NMEA_Type(const char *sentence) {
    if (strncmp(sentence, "$GPGGA", 6) == 0) return NMEA_GPGGA;
    if (strncmp(sentence, "$GPRMC", 6) == 0) return NMEA_GPRMC;
    if (strncmp(sentence, "$GPGSV", 6) == 0) return NMEA_GPGSV;
    if (strncmp(sentence, "$GPGSA", 6) == 0) return NMEA_GPGSA;
    if (strncmp(sentence, "$GPVTG", 6) == 0) return NMEA_GPVTG;
    
    return NMEA_UNKNOWN;
}

/**
 * @brief Convert DDMM.MMMM format to decimal degrees
 * @param ddmm_str String in DDMM.MMMM format
 * @param direction 'N'/'S' for latitude, 'E'/'W' for longitude
 * @return Decimal degrees
 */
float GPS_Convert_DDMM_to_Decimal(const char *ddmm_str, char direction) {
    if (strlen(ddmm_str) == 0) return 0.0f;
    
    float ddmm = atof(ddmm_str);
    
    // Extract degrees and minutes
    int degrees = (int)(ddmm / 100);
    float minutes = ddmm - (degrees * 100);
    
    // Convert to decimal degrees
    float decimal = degrees + (minutes / 60.0f);
    
    // Apply direction
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

/**
 * @brief Calculate NMEA checksum
 * @param sentence NMEA sentence
 * @return Calculated checksum
 */
uint8_t GPS_Calculate_Checksum(const char *sentence) {
    uint8_t checksum = 0;
    int i = 1; // Skip the '$'
    
    // XOR all characters between '$' and '*'
    while (sentence[i] != '*' && sentence[i] != '\0') {
        checksum ^= sentence[i];
        i++;
    }
    
    return checksum;
}

/**
 * @brief Verify NMEA sentence checksum
 * @param sentence NMEA sentence
 * @return 1 if checksum valid, 0 otherwise
 */
uint8_t GPS_Verify_Checksum(const char *sentence) {
    char *asterisk = strchr(sentence, '*');
    if (asterisk == NULL) return 0;
    
    // Calculate checksum
    uint8_t calculated = GPS_Calculate_Checksum(sentence);
    
    // Extract checksum from sentence
    uint8_t received = 0;
    sscanf(asterisk + 1, "%2hhx", &received);
    
    return (calculated == received) ? 1 : 0;
}

/**
 * @brief Parse NMEA sentence and update GPS data
 * @param sentence NMEA sentence string
 * @param gps_data Pointer to GPS data structure
 * @return 1 if successfully parsed, 0 otherwise
 */
uint8_t GPS_Parse_NMEA_Sentence(const char *sentence, GPS_Data *gps_data) {
    NMEA_Type type = GPS_Parse_NMEA_Type(sentence);
    char *fields[GPS_MAX_FIELDS];
    char sentence_copy[GPS_BUFFER_SIZE];
    int field_count = 0;
    
    // Make a copy for strtok
    strncpy(sentence_copy, sentence, GPS_BUFFER_SIZE - 1);
    sentence_copy[GPS_BUFFER_SIZE - 1] = '\0';
    
    // Tokenize the sentence
    char *token = strtok(sentence_copy, ",");
    while (token != NULL && field_count < GPS_MAX_FIELDS) {
        fields[field_count++] = token;
        token = strtok(NULL, ",");
    }
    
    switch (type) {
        case NMEA_GPGGA:
            // $GPGGA,time,lat,lat_dir,lon,lon_dir,quality,satellites,hdop,altitude,alt_unit,height,height_unit,time_diff,station_id*checksum
            if (field_count >= 15) {
                // Parse time (HHMMSS.SSS)
                if (strlen(fields[1]) >= 6) {
                    char time_str[3];
                    strncpy(time_str, fields[1], 2); time_str[2] = '\0';
                    gps_data->hours = atoi(time_str);
                    strncpy(time_str, fields[1] + 2, 2); time_str[2] = '\0';
                    gps_data->minutes = atoi(time_str);
                    strncpy(time_str, fields[1] + 4, 2); time_str[2] = '\0';
                    gps_data->seconds = atoi(time_str);
                }
                
                // Parse position
                if (strlen(fields[2]) > 0 && strlen(fields[4]) > 0) {
                    gps_data->latitude = GPS_Convert_DDMM_to_Decimal(fields[2], fields[3][0]);
                    gps_data->longitude = GPS_Convert_DDMM_to_Decimal(fields[4], fields[5][0]);
                }
                
                // Parse fix quality
                gps_data->fix_status = (GPS_Fix_Status)atoi(fields[6]);
                
                // Parse satellites and HDOP
                gps_data->satellites_used = atoi(fields[7]);
                gps_data->hdop = atof(fields[8]);
                
                // Parse altitude
                gps_data->altitude = atof(fields[9]);
                
                gps_data->data_valid = (gps_data->fix_status > GPS_NO_FIX) ? 1 : 0;
                return 1;
            }
            break;
            
        case NMEA_GPRMC:
            // $GPRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir*checksum
            if (field_count >= 12) {
                // Parse status ('A' = active, 'V' = void)
                if (fields[2][0] == 'A') {
                    // Parse time
                    if (strlen(fields[1]) >= 6) {
                        char time_str[3];
                        strncpy(time_str, fields[1], 2); time_str[2] = '\0';
                        gps_data->hours = atoi(time_str);
                        strncpy(time_str, fields[1] + 2, 2); time_str[2] = '\0';
                        gps_data->minutes = atoi(time_str);
                        strncpy(time_str, fields[1] + 4, 2); time_str[2] = '\0';
                        gps_data->seconds = atoi(time_str);
                    }
                    
                    // Parse position
                    if (strlen(fields[3]) > 0 && strlen(fields[5]) > 0) {
                        gps_data->latitude = GPS_Convert_DDMM_to_Decimal(fields[3], fields[4][0]);
                        gps_data->longitude = GPS_Convert_DDMM_to_Decimal(fields[5], fields[6][0]);
                    }
                    
                    // Parse speed (knots to km/h)
                    gps_data->speed_kmh = atof(fields[7]) * 1.852f;
                    
                    // Parse course
                    gps_data->course_degrees = atof(fields[8]);
                    
                    gps_data->data_valid = 1;
                    return 1;
                }
            }
            break;
            
        default:
            // Unsupported sentence type
            break;
    }
    
    return 0;
}

/**
 * @brief Print GPS data to console
 * @param gps_data Pointer to GPS data structure
 */
void GPS_Print_Data(const GPS_Data *gps_data) {
    printf("=== GPS Data ===\n");
    printf("Time: %02d:%02d:%02d\n", gps_data->hours, gps_data->minutes, gps_data->seconds);
    printf("Position: %.6f, %.6f\n", gps_data->latitude, gps_data->longitude);
    printf("Altitude: %.1f m\n", gps_data->altitude);
    printf("Fix Status: %d\n", gps_data->fix_status);
    printf("Satellites: %d\n", gps_data->satellites_used);
    printf("HDOP: %.1f\n", gps_data->hdop);
    printf("Speed: %.1f km/h\n", gps_data->speed_kmh);
    printf("Course: %.1f°\n", gps_data->course_degrees);
    printf("Data Valid: %s\n", gps_data->data_valid ? "Yes" : "No");
    printf("================\n\n");
}