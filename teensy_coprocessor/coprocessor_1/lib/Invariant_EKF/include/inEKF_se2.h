/**
 * @file inEKF_se2.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __INC_INEKF_SE2_H__
#define __INC_INEKF_SE2_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define DIMS 3
#define TOTAL DIMS*DIMS

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief SE(2) state struct for a differential-drive robot with a 2D
 *  magnetometer for heading measurements
 * 
 * @param x x-position in the plane
 * @param y y-position in the plane
 * @param theta heading angle in the plane
 */
typedef struct {
    float x;
    float y;
    float theta;
} state_se2_t;


#define IDENTITY_SE2 {0.0f, 0.0f, 0.0f}

/**
 * @brief common indices for the rotation matrix elements in the SE(2) exponential map and adjoint map
 */
#define R_00 (3*0 + 0)  // 0
#define R_01 (3*0 + 1)  // 1
#define R_10 (3*1 + 0)  // 3
#define R_11 (3*1 + 1)  // 4

#define T_x  (3*0 + 2)  // 2
#define T_y  (3*1 + 2)  // 5

#define Z_20 (3*2 + 0)  // 6
#define Z_21 (3*2 + 1)  // 7
#define I_22 (3*2 + 2)  // 8

#define SMALL_ANGLE(angle) (fabsf(angle) < 1e-6f)

/**
 * @brief struct to hold all variables related to the invariant EKF for SE(2)
 *  state estimation and specifically for a differential-drive robot with a 2D
 *  magnetometer for heading measurements
 * 
 * @details The state is stored as a 1D array for ease of use in C, but the
 *  order of the elements is [x, y, theta] to match the state_se2_t struct
 */
typedef struct {

    // invariant filter parameters -------------------------------------------
    float dt;

    // length of the differential-drive robot's wheelbase
    float L; 

    // complementary filter parameter for fusing gyro and encoder measurements
    float alpha;


    // limits for magnetometer norm to reject outliers
    float mag_norm_min; // usually 2000 for AK09916
    float mag_norm_max; // usually 6500 for AK09916


    // filter variables -------------------------------------------
    state_se2_t state;

    // process noise covariance matrix, aka Q
    state_se2_t process_noise;

    // innovation gate threshold for outlier rejection
    float chi2_threshold;

    // covariance in tangent space, aka P
    state_se2_t covariance;

    // measurement noise covariance matrix, aka R
    // state_se2_t measurement_noise;
    float mag_noise;

} InEKF_SE2_t;


// ----------------------------------------------------------------------------
//
//  LIE GROUP HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief hat operator for SE(2) Lie algebra
 * 
 * @details maps the Lie algebra element tau in the tangent space to a matrix
 *  in the Lie group SE(2) with generators corresponding to the order of the
 *  state vector, i.e. [x, y, theta]
 * 
 * @note tau_wedge is a 3x3 skew-symmetric matrix in the Lie group SE(2)
 * 
 * @param tau state_se2_t vector in the tangent space, aka the Lie algebra se(2)
 * @param tau_wedge 3x3 matrix in the Lie group SE(2)
 * 
 */
inline void wedge_se2(
        state_se2_t*    tau,
        float           tau_wedge[TOTAL]);


/**
 * @brief vee operator for SE(2) Lie algebra, a.k.a. the inverse of the hat
 *  operator
 * 
 * @details maps a matrix in the Lie group SE(2) to a vector in the tangent
 *  space, i.e. the Lie algebra se(2)
 * 
 * @param tau_wedge 3x3 matrix in the Lie group SE(2)
 * @param tau state_se2_t vector in the tangent space, aka the Lie algebra se(2)
 */
inline void vee_se2(
        float           tau_wedge[TOTAL],
        state_se2_t*    tau);


/**
 * @brief exponential map for SE(2) Lie algebra
 * 
 * @details maps a vector in the tangent space, i.e. the Lie algebra se(2), to
 *  a matrix in the Lie group SE(2)
 * 
 * @param tau state_se2_t vector in the tangent space, aka the Lie algebra se(2)
 * @param exp_tau 3x3 matrix in the Lie group SE(2)
 */
void exp_se2(
        state_se2_t*    tau,
        float           exp_tau[TOTAL]);


/**
 * @brief logarithm map for SE(2) Lie algebra
 * 
 * @details maps a matrix in the Lie group SE(2) to a vector in the tangent
 *  space, i.e. the Lie algebra se(2)
 * 
 * @param exp_tau 3x3 matrix in the Lie group SE(2)
 * @param tau state_se2_t vector in the tangent space, aka the Lie algebra se(2)
 */
void log_se2(
        float           exp_tau[TOTAL],
        state_se2_t*    tau);


/**
 * @brief adjoint map for SE(2) Lie algebra
 * 
 * @details maps a matrix in the Lie group SE(2) to its adjoint representation
 *  in the Lie algebra se(2)
 * 
 * @param exp_tau 3x3 matrix in the Lie group SE(2)
 * @param adj_exp_tau 3x3 matrix representing the adjoint in the se(2) Lie
 *  algebra 
 */
void adjoint_se2(
        float           exp_tau[TOTAL],
        float           adj_exp_tau[TOTAL]);


// ----------------------------------------------------------------------------
//
//  INVARIANT EKF FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief initializing function
 * 
 * @details variables that are internally declared:
 * 
 *  - alpha = complementary filter fusion of gyro and encoder measurements
 *  - L     = length of the differential-drive robot's wheelbase
 * 
 *  - start starter state as identity, or (x, y, \theta) = (0, 0, 0)
 *  - covariance matrix P as diagonal with somewhat large values, e.g. 0.1 for
 *      position
 * 
 * @param filter InEKF_SE2_t struct to initialize
 * @param dt Time step for the filter
 * @param process_noise Process noise covariance
 * @param mag_noise Magnetometer noise covariance
 * @param chi2_threshold Chi-squared threshold for outlier rejection
 */
void inEKF_SE2_init(
        InEKF_SE2_t*    filter,

        float           dt,
        state_se2_t*    process_noise,
        float           mag_noise,
        float           chi2_threshold);

/**
 * @brief InEKF prediction step along the SE(2) manifold for a differential-
 *  drive robot with gyro and encoder
 * 
 * @param filter InEKF_SE2_t struct containing the current state estimate, 
 *  covariance, and other filter parameters
 * @param v_L left wheel velocity from encoders
 * @param v_R right wheel velocity from encoders
 * @param omega_gyro IMU angular velocity measurement from gyro
 */
void inEKF_SE2_predict(
        InEKF_SE2_t* filter,

        float   v_L,
        float   v_R,
        float   omega_gyro);

/**
 * @brief InEKF update step using magnetometer measurements
 * 
 * @param filter InEKF_SE2_t struct containing the current state estimate, 
 *  covariance, and other filter parameters
 * @param theta_mag Magnetometer heading measurement
 * @param mag_norm Magnetometer measurement norm
 * 
 * @return uint8_t boolean flag indicating whether the magnetometer measurement
 *  was rejected as an outlier (1) or accepted (0)
 * 
 * @retval 0 if update successful, 1 if magnetometer measurement rejected as an
 *  outlier
 */
uint8_t inEKF_SE2_update_mag(
        InEKF_SE2_t* filter,

        float   theta_mag,
        float   mag_norm);

/** ---------------------------------------------------------
 *  HELPER FUNCTIONS
 */

/**
 * @brief retrieves the current state estimate from the filter struct
 * 
 * @param filter InEKF_SE2_t struct containing the current state estimate, 
 *  covariance, and other filter parameters
 * @param state_out state_se2_t struct to store the retrieved state estimate
 */
void inEKF_SE2_get_state(
        InEKF_SE2_t*    filter,
        state_se2_t*    state_out);


// void inEKF_SE2_get_covariance_trace(
//         InEKF_SE2_t* filter,
//         float trace);


/**
 * @brief wraps an angle to the range [-pi, pi]
 * 
 * @param theta pointer to the angle to be wrapped
 */
inline void inEKF_SE2_wrap_angle(
        float*  theta);

#ifdef __cplusplus
}
#endif

#endif /* __INC_INEKF_SE2_H__ */
