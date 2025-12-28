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
#include "EUSCI_B1_I2C.h"
#include "ICM_20948_Registers.h"


// ----------------------------------------------------------------------------
//
//  DATA STRUCTURES & CONSTANTS
//
// ----------------------------------------------------------------------------

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
    // example
    // i2c_inst_t icm20948_i2c = {i2c0_hw, false}
    i2c_inst_t *i2c;
} icm20948_config_t;


/**
 * @brief 
 * 
 */
typedef struct icm20984_data {
    // 0: x, 1: y, 2: z
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

#endif /* __INC_ICM_20948_H__ */