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

#include "Project_Config.h"
#include "EUSCI_B1_I2C.h"
#include "ICM_20948_Registers.h"

#include "ICM_20948.h"

#include "Clock.h"

//#define OLD_SYSTEM 1

#ifdef OLD_SYSTEM

// ----------------------------------------------------------------------------
//
//  DATA STRUCTURES & CONSTANTS
//
// ----------------------------------------------------------------------------

/**
 * @brief Adddress of the ICM-20948 sensor when AD0 is low
 */
#define ICM20948_ADDR_ACCEL_GYRO    0x68


/**
 * @brief Address of the AK09916 magnetometer sensor
 */
#define ICM20948_ADDR_MAG           0x0C

/**
 * @brief
 *
 */
typedef struct icm20948_config {

    // usual addr
    // addr_accel_gyro:  0x68
    // addr_mag:         0x0C
    uint8_t    addr_accel_gyro;
    uint8_t    addr_mag;

} icm20948_config_t;


/**
 * @brief
 *
 * @details indices 0, 1 & 2 represent x, y, & z dimensions
 */
typedef struct {

    //
    int16_t accel_raw[3];
    int16_t accel_bias[3];

    int16_t gyro_raw[3];
    int16_t gyro_bias[3];

    int16_t mag_raw[3];
    int16_t mag_bias[3];
    float temp_c;

} icm20984_data_t;


// ----------------------------------------------------------------------------
//
//  INITIALIZATION FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief
 *
 * @param config
 * @return int8_t
 */
int8_t icm20948_init(icm20948_config_t *config);

/**
 * @brief
 *
 * @param config
 * @param mode
 */
void icm20948_set_mag_rate(icm20948_config_t *config, uint8_t mode);

// ----------------------------------------------------------------------------
//
//  READ FUNCTIONS
//
// ----------------------------------------------------------------------------

/** ---------------------------------------------------------------------------
 * READ RAW DATA FUNCTIONS
 */

void icm20948_read_raw_accel(icm20948_config_t *config, int16_t accel[3]);
void icm20948_read_raw_gyro(icm20948_config_t *config, int16_t gyro[3]);
void icm20948_read_raw_temp(icm20948_config_t *config, int16_t *temp);
void icm20948_read_raw_mag(icm20948_config_t *config, int16_t mag[3]);

/** ---------------------------------------------------------------------------
 * CALIBRATION FUNCTIONS
 */

void icm20948_cal_gyro(icm20948_config_t *config, int16_t gyro_bias[3]);
void icm20948_cal_accel(icm20948_config_t *config, int16_t accel_bias[3]);
void icm20948_cal_mag_simple(icm20948_config_t *config, int16_t mag_bias[3]);

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
        icm20948_config_t *config,
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
        icm20948_config_t *config,
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
        icm20948_config_t *config,
        int16_t mag[3],
        int16_t bias[3]);


/**
 * @brief
 *
 * @param config
 * @param temp
 */
void icm20948_read_temp_c(
        icm20948_config_t *config,
        float* temp);


#else



// Forward declaration for C++ class
#ifdef __cplusplus
class TwoWire;
#endif

#ifdef __cplusplus
extern "C" {
#endif



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
        uint8_t    icm_address_inst,
        uint8_t    mag_address_inst);


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


//void icm20948_load_calibration(
//        sensor_config_t*    config,
//        vector_int_t        gyro_bias,
//        vector_int_t        accel_bias);
//
//
//void ak09916_load_calibration(
//        sensor_config_t*    config,
//        vector_int_t        mag_bias);



// ----------------------------------------------------------------------------
//
//  READ FUNCTIONS
//
// ----------------------------------------------------------------------------



/* ----------------------------------------------------------------------------
 * READ CALIBRATED DATA FUNCTIONS
 */


//uint8_t icm20948_record_data(
//        sensor_config_t*    config,
//        dataframe_t*        data);
//
//
//uint8_t ak09916_record_data(
//        sensor_config_t*    config,
//        dataframe_t*        data);


#ifdef __cplusplus
}
#endif

#endif

#endif /* __INC_ICM_20948_H__ */
