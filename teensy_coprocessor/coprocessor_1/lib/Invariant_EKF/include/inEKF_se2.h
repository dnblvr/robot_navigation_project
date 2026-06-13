/**
 * @file inEKF_se2.h
 * @author your name (you@domain.com)
 * @brief port specialized for the Teensy 4.0
 * @version 0.1
 * @date 2026-03-01
 */
#ifndef __INC_INEKF_SE2_H__
#define __INC_INEKF_SE2_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <state_se2.h>

#ifdef __cplusplus
extern "C" {
#endif

// ————————————————————————————————————————————————————————————————————————————
//
//  DATA STRUCTURES
//
// ————————————————————————————————————————————————————————————————————————————


/**
 * @brief struct to hold all variables related to the invariant EKF for `SE(2)`
 *  state estimation and specifically for a differential-drive robot with a 2D
 *  magnetometer for heading measurements
 * 
 * @details The state is stored as a 1D array for ease of use in C, but the
 *  order of the elements is [x, y, theta] to match the `se2_t` struct
 */
typedef struct {

    // —— invariant filter parameters —————————————————————————————————————————
    float   dt;

    // pre-computed inverse of `dt` for efficiency
    float   inv_dt;

    // length of the differential-drive robot's wheelbase
    float   L;

    // pre-computed inverse of the wheelbase for efficiency
    float   inv_L; 

    // complementary filter parameter for fusing gyro and encoder measurements
    float   alpha;

    // limits for magnetometer norm to reject outliers
    float   mag_norm_min;
    float   mag_norm_max;


    // —— filter variables ————————————————————————————————————————————————————
    se2_t state;

    // process noise covariance matrix, aka `Q`
    // se2_t process_noise;
    float process_noise[TOTAL];

    // innovation gate threshold for outlier rejection
    float chi2_threshold;

    // covariance in tangent space, aka `P`
    // se2_t covariance;
    float covariance[TOTAL];

    // measurement noise covariance matrix, aka `R`
    // se2_t measurement_noise;
    float mag_noise;

} InEKF_SE2_t;


// ————————————————————————————————————————————————————————————————————————————
//
//  INVARIANT EKF FUNCTIONS
//
// ————————————————————————————————————————————————————————————————————————————

/**
 * @brief initializing function 
 * 
 * @details variables that are internally declared:
 * 
 *  - alpha = complementary filter fusion of gyro and encoder measurements
 * 
 *  - start starter state as identity, or (x, y, \theta) = (0, 0, 0)
 *  - covariance matrix P as diagonal with somewhat large values, e.g. 0.1 for
 *      position
 * 
 * @param[in] filter `InEKF_SE2_t` struct to initialize
 * @param[in] dt     Time step for the filter
 * @param[in] L      Differential-drive robot's wheelbase length (mm)
 * @param[in] process_noise  Process noise covariance
 * @param[in] mag_noise      Magnetometer noise covariance
 * @param[in] chi2_threshold Chi-squared threshold for outlier rejection
 */
void inEKF_SE2_init(
        InEKF_SE2_t*    filter,

        float           dt,
        float           L,
        se2_t*    process_noise,
        float           mag_noise,
        float           chi2_threshold);


/**
 * @brief InEKF prediction step along the SE(2) manifold for a differential-
 *  drive robot with gyro and encoder
 * 
 * @param[inout] filter  InEKF_SE2_t struct containing the current state
 *  estimate, covariance, and other filter parameters
 * @param[in] v_L        left wheel velocity from encoders
 * @param[in] v_R        right wheel velocity from encoders
 * @param[in] omega_gyro IMU angular velocity measurement from gyro
 */
void inEKF_SE2_predict(
        InEKF_SE2_t* filter,

        float   v_L,
        float   v_R,
        float   omega_gyro);


/**
 * @brief InEKF update step using magnetometer measurements
 * 
 * @param[inout] filter InEKF_SE2_t struct containing the current state
 *  estimate, covariance, and other filter parameters
 * @param[in] theta_mag Magnetometer heading measurement
 * @param[in] mag_norm Magnetometer measurement norm
 * 
 * @return `uint8_t` boolean flag indicating whether the magnetometer
 *  measurement was rejected as an outlier or accepted
 * 
 * @retval 
 *  - `0` if update successful
 *  - `1` if magnetometer measurement rejected as an outlier
 */
uint8_t inEKF_SE2_update_mag(
        InEKF_SE2_t* filter,

        float   theta_mag,
        float   mag_norm);
        

/** ---------------------------------------------------------
 *  HELPER FUNCTIONS - lie group operations
 */


/**
 * @brief retrieves the current state estimate from the filter struct
 * 
 * @param[in] filter `InEKF_SE2_t` struct containing the current state
 *      estimate, covariance, and other filter parameters
 * @param[out] state_out `se2_t` struct to store the retrieved state
 *      estimate
 */
inline void inEKF_SE2_get_state(
        InEKF_SE2_t*    filter,
        se2_t*    state_out)
{
    state_out->x      = filter->state.x;
    state_out->y      = filter->state.y;
    state_out->theta  = filter->state.theta;
}

// void inEKF_SE2_get_covariance_trace(
//         InEKF_SE2_t* filter,
//         float trace);





#ifdef __cplusplus
}
#endif

#endif /* __INC_INEKF_SE2_H__ */
