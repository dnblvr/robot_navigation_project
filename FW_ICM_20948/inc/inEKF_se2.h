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
#include <string.h>
#include <math.h>

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

#define M_PI_F 3.1415927f


#define IDENTITY_SE2 {0.0f, 0.0f, 0.0f}

#define IDENTITY {1.f, 0.f, 0.f, \
                  0.f, 1.f, 0.f, \
                  0.f, 0.f, 1.f}

/**
 * @brief common indices for the rotation matrix elements in the SE(2) exponential map and adjoint map
 */
#define R_00 (3*0 + 0)  // 0
#define R_01 (3*0 + 1)  // 1
#define R_10 (3*1 + 0)  // 3
#define R_11 (3*1 + 1)  // 4

#define T_x_  (3*0 + 2)  // 2
#define T_y_  (3*1 + 2)  // 5

#define Z_20 (3*2 + 0)  // 6
#define Z_21 (3*2 + 1)  // 7
#define I_22 (3*2 + 2)  // 8

/**
 * @brief indices for a 3x3 matrix in row-major order, for ease of use in C
 */
#define M_00 (3*0 + 0)  // 0
#define M_01 (3*0 + 1)  // 1
#define M_02 (3*0 + 2)  // 2

#define M_10 (3*1 + 0)  // 3
#define M_11 (3*1 + 1)  // 4
#define M_12 (3*1 + 2)  // 5

#define M_20 (3*2 + 0)  // 6
#define M_21 (3*2 + 1)  // 7
#define M_22 (3*2 + 2)  // 8

/**
 * @brief macro to check if an angle is small enough to use the first-order
 *  Taylor expansion for the exponential map
 */
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

    // pre-computed inverse of dt for efficiency
    float inv_dt;

    // length of the differential-drive robot's wheelbase
    float L;

    // pre-computed inverse of the wheelbase for efficiency
    float inv_L; 

    // complementary filter parameter for fusing gyro and encoder measurements
    float alpha;


    // limits for magnetometer norm to reject outliers
    float mag_norm_min; // usually 2000 for AK09916
    float mag_norm_max; // usually 6500 for AK09916


    // filter variables -------------------------------------------
    state_se2_t state;

    // process noise covariance matrix, aka Q
    // state_se2_t process_noise;
    float process_noise[TOTAL];

    // innovation gate threshold for outlier rejection
    float chi2_threshold;

    // covariance in tangent space, aka P
    // state_se2_t covariance;
    float covariance[TOTAL];

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
 * @param filter            InEKF_SE2_t struct to initialize
 * @param dt                Time step for the filter
 * @param L                 Time step for the filter
 * @param process_noise     Process noise covariance
 * @param mag_noise         Magnetometer noise covariance
 * @param chi2_threshold    Chi-squared threshold for outlier rejection
 */
void inEKF_SE2_init(
        InEKF_SE2_t*    filter,

        float           dt,
        float           L,
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

void matrix_to_state(
        float           state_matrix[TOTAL],
        state_se2_t*    state);

/**
 * @brief  
 * 
 * @param A 
 * @param B 
 * @param state_out 
 */
void compose_SE2(
        state_se2_t     A,
        state_se2_t     B,
        state_se2_t*    state_out);


/**
 * @brief matrix inverse for 3x3 matrices
 * 
 * @note this function, if implemented, will be heavily simplified and
 *  optimized. otherwise, I will add this for future use
 * 
 * @param A 
 * @param A_inv 
 */
void inverse_3x3(
        float   A[TOTAL],
        float   A_inv[TOTAL]);


/**
 * @brief simple matrix multiplication for 3x3 matrices
 * 
 * @param A 
 * @param B 
 * @param AB 
 */
void matmul_3x3(
        float   A[TOTAL],
        float   B[TOTAL],
        float   AB[TOTAL]);


/**
 * @brief element-wise addition of two 3x3 matrices
 * 
 * @param A 
 * @param B 
 * @param AB 
 */
inline void matadd_3x3(
        float   A[TOTAL],
        float   B[TOTAL],
        float   AB[TOTAL]);


/**
 * @brief matrix multiplication for the specific 3x1 by 1x3 case
 * 
 * @note used for calculating K*R*K^T in the covariance update step of the
 *  magnetometer measurement update
 */
void matmul_3_1x1_3(
        float   A[DIMS],
        float   B[DIMS],
        float   AB[TOTAL]);


/**
 * @brief replicates the matrix product A*B*A^T for 3x3 matrices
 * 
 * @param A 
 * @param B 
 * @param ABA_T 
 */
void congruence_3x3(
        float   A[TOTAL],
        float   B[TOTAL],
        float   ABA_T[TOTAL]);


/**
 * @brief helper function to transpose a 3x3 se(2) matrix
 * 
 * @note this function will be heavily simplified and optimized for the
 *  specific structure of the matrices we are working with, by only transposing
 *  the relevant elements and skipping diagonal elements.
 * 
 * @param[in] matrix_in input matrix to be transposed
 * @param[out] matrix_out output transposed matrix
 */
inline void transpose_3x3(
        float   matrix_in[TOTAL],
        float   matrix_out[TOTAL]);


/**
 * @brief retrieves the current state estimate from the filter struct
 * 
 * @param[in] filter `InEKF_SE2_t` struct containing the current state
 *      estimate, covariance, and other filter parameters
 * @param[out] state_out `state_se2_t` struct to store the retrieved state
 *      estimate
 */
void inEKF_SE2_get_state(
        InEKF_SE2_t*    filter,
        state_se2_t*    state_out);


// void inEKF_SE2_get_covariance_trace(
//         InEKF_SE2_t* filter,
//         float trace);


/**
 * @brief  helper function to wrap an angle to the range [-pi, pi]
 * 
 * @note this function will be used in the composition of the state increment along with 
 * 
 * @param[in] theta angle in radians
 * @return float wrapped angle in radians
 * @retval wrapped angle in the range [-pi, pi]
 */
inline float _wrap_angle(
        float  theta);

#ifdef __cplusplus
}
#endif

#endif /* __INC_INEKF_SE2_H__ */
