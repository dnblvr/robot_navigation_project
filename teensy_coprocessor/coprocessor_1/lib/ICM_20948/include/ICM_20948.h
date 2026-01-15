/**
 * @file ICM_20948.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-12-27
 */

#ifndef __INC_ICM_20948_H__
#define __INC_ICM_20948_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "ICM_20948_Registers.h"

// Forward declaration for C++ TwoWire class
#ifdef __cplusplus
class TwoWire;
#endif

#ifdef __cplusplus
extern "C" {
#endif


// ----------------------------------------------------------------------------
//
//  DATA STRUCTURES & CONSTANTS
//
// ----------------------------------------------------------------------------

/**
 * @brief Adddress of the ICM-20948 sensor when AD0 is low
 */
#define ICM20948_ADDR_ACCEL_GYRO_0    0x68


/**
 * @brief Adddress of the ICM-20948 sensor when AD0 is high
 */
#define ICM20948_ADDR_ACCEL_GYRO_1    0x69

/**
 * @brief Address of the AK09916 magnetometer sensor
 */
#define ICM20948_ADDR_MAG           0x0C

/**
 * @brief 
 */
typedef struct icm20948_config {

    // usual addr
    uint8_t    i2c_address;
    
    // Pointer to Wire instance (TwoWire*)
    void*      wire_instance;

    // Pointer to USB Serial instance (usb_serial_class*)
    void*      serial_usb;

} sensor_config_t;


// ----------------------------------------------------------------------------
//
//  INITIALIZATION FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @param icm_config 
 * @param mag_config 
 * @param serial_usb 
 * @return int8_t 
 */
int8_t icm20948_init(
        sensor_config_t*    icm_config,
        sensor_config_t*    mag_config,
        void*              serial_usb);


/**
 * @brief 
 * 
 * @param config 
 * @param mode 
 */
void icm20948_set_mag_rate(
        sensor_config_t*    config,
        uint8_t             mode);


/** ---------------------------------------------------------------------------
 * CALIBRATION FUNCTIONS
 */

void icm20948_cal_gyro(sensor_config_t*    config, int16_t gyro_bias[3]);
void icm20948_cal_accel(sensor_config_t*    config, int16_t accel_bias[3]);
void icm20948_cal_mag_simple(sensor_config_t*    config, int16_t mag_bias[3]);

// ----------------------------------------------------------------------------
//
//  READ FUNCTIONS
//
// ----------------------------------------------------------------------------

/** ---------------------------------------------------------------------------
 * READ RAW DATA FUNCTIONS
 */

void icm20948_read_raw_accel(sensor_config_t*    config, int16_t accel[3]);
void icm20948_read_raw_gyro(sensor_config_t*    config, int16_t gyro[3]);
void icm20948_read_raw_temp(sensor_config_t*    config, int16_t *temp);
void icm20948_read_raw_mag(sensor_config_t*    config, int16_t mag[3]);

/** ---------------------------------------------------------------------------
 * READ CALIBRATED DATA FUNCTIONS
 */

/**
 * @brief 
 * 
 * @param config 
 * @param gyro 
 * @param bias 
 */
void icm20948_read_cal_gyro(
        sensor_config_t *config,
        int16_t gyro[3],
        int16_t bias[3]);

/**
 * @brief 
 * 
 * @param config 
 * @param accel 
 * @param bias 
 */
void icm20948_read_cal_accel(
        sensor_config_t *config,
        int16_t accel[3],
        int16_t bias[3]);


/**
 * @brief 
 * 
 * @param config 
 * @param mag 
 * @param bias 
 */
void icm20948_read_cal_mag(
        sensor_config_t *config,
        int16_t mag[3],
        int16_t bias[3]);


/**
 * @brief 
 * 
 * @param config 
 * @param temp 
 */
void icm20948_read_temp_c(
        sensor_config_t *config,
        float* temp);


#ifdef __cplusplus
}
#endif

#endif /* __INC_ICM_20948_H__ */
