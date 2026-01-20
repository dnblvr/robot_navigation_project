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

#include <helper_3dmath.h>

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
 * @brief Simple vector structure, in fixed point integers
 * 
 * @param x X-axis component
 * @param y Y-axis component
 * @param z Z-axis component
 */
typedef struct {

    int16_t x, y, z;

} vector_int_t;


/**
 * @brief Simple vector structure, in flaots
 * 
 * @param x X-axis component
 * @param y Y-axis component
 * @param z Z-axis component
 */
typedef struct {

    float x, y, z;

} vector_float_t;


/**
 * @brief ICM-20948 data container
 * 
 * @param accel  Accelerometer data
 * @param gyro   Gyroscope data
 * @param temp   Temperature data
 * @param counts Sample count or timestamp
 */
typedef struct {

    vector_int_t    accel;
    vector_int_t    gyro;
    int16_t         temp;
    uint32_t        counts;

} icm_data_t;


/**
 * @brief AK09916 magnetometer data container
 * 
 * @param mag    Magnetometer data
 * @param counts Sample count or timestamp
 */
typedef struct {

    vector_int_t    mag;
    uint32_t        counts;

} ak_data_t;


/**
 * @brief Generic data frame container
 * 
 * @param accel  Accelerometer data
 */
typedef struct {

    vector_int_t    accel;
    vector_int_t    gyro;
    int16_t         temp;
    vector_int_t    mag;
    Quaternion      quat;
    uint32_t        counts;

} dataframe_t;


/**
 * @brief Address of the ICM-20948 sensor when AD0 is low
 */
#define ICM20948_ADDR_ACCEL_GYRO_0  0x68


/**
 * @brief Address of the ICM-20948 sensor when AD0 is high
 */
#define ICM20948_ADDR_ACCEL_GYRO_1  0x69


/**
 * @brief Address of the AK09916 magnetometer sensor
 */
#define ICM20948_ADDR_MAG           0x0C


/**
 * @brief Sensor configuration structure
 * 
 * @param i2c_address       I2C address of the sensor
 * @param wire_instance     Pointer to TwoWire instance (cast to void*)
 * @param offset_instance   Pointer to offset structure (cast to void*)
 */
typedef struct {

    uint8_t i2c_address;
    void*   wire_instance;
    void*   offset_instance;

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
        void*               serial_usb);


/**
 * @brief 
 * 
 * @param config 
 * @param mode 
 */
void icm20948_set_mag_rate(
        sensor_config_t*    config,
        uint8_t             mode);


/* ----------------------------------------------------------------------------
 * CALIBRATION FUNCTIONS
 */


void icm20948_load_calibration(
        sensor_config_t*    config,
        vector_int_t        gyro_bias,
        vector_int_t        accel_bias);


void ak09916_load_calibration(
        sensor_config_t*    config,
        vector_int_t        mag_bias);


// void icm20948_cal_gyro(
//         sensor_config_t*    config,
//         int16_t             gyro_bias[3]);
// void icm20948_cal_accel(
//         sensor_config_t*    config,
//         int16_t             accel_bias[3]);
// void icm20948_cal_mag_simple(
//         sensor_config_t*    config,
//         int16_t             mag_bias[3]);


// ----------------------------------------------------------------------------
//
//  READ FUNCTIONS
//
// ----------------------------------------------------------------------------

/* ----------------------------------------------------------------------------
 * READ RAW DATA FUNCTIONS
 */

// void icm20948_read_raw_accel(sensor_config_t*    config, int16_t accel[3]);
// void icm20948_read_raw_gyro(sensor_config_t*    config, int16_t gyro[3]);
// void icm20948_read_raw_temp(sensor_config_t*    config, int16_t *temp);
// void icm20948_read_raw_mag(sensor_config_t*    config, int16_t mag[3]);

/* ----------------------------------------------------------------------------
 * READ CALIBRATED DATA FUNCTIONS
 */


uint8_t icm20948_record_data(
        sensor_config_t*    config,
        dataframe_t*        data);


uint8_t ak09916_record_data(
        sensor_config_t*    config,
        dataframe_t*        data);


/**
 * @brief 
 * 
 * @param config 
 * @param gyro 
 * @param bias 
 */
// void icm20948_read_cal_gyro(
//         sensor_config_t *config,
//         int16_t gyro[3],
//         int16_t bias[3]);


/**
 * @brief 
 * 
 * @param config 
 * @param accel 
 * @param bias 
 */
// void icm20948_read_cal_accel(
//         sensor_config_t *config,
//         int16_t accel[3],
//         int16_t bias[3]);


/**
 * @brief 
 * 
 * @param config 
 * @param mag 
 * @param bias 
 */
// void icm20948_read_cal_mag(
//         sensor_config_t *config,
//         int16_t mag[3],
//         int16_t bias[3]);


/**
 * @brief 
 * 
 * @param config 
 * @param temp 
 */
// void icm20948_read_temp_c(
//         sensor_config_t *config,
//         float* temp);


#ifdef __cplusplus
}
#endif

#endif /* __INC_ICM_20948_H__ */
