/**
 * @file inEKF_se2.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-01
 * 
 * @copyright Copyright (c) 2026
 */

 #include "inEKF_se2.h"

// ----------------------------------------------------------------------------
//
//  LIE GROUP HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------


void wedge_se2(
        state_se2_t*    tau,
        float           tau_wedge[TOTAL])
{
    tau_wedge[3*0 + 0] = 0.0f;
    tau_wedge[3*0 + 1] = -tau->theta;
    tau_wedge[3*0 + 2] = tau->x;

    tau_wedge[3*1 + 0] = tau->theta;
    tau_wedge[3*1 + 1] = 0.0f;
    tau_wedge[3*1 + 2] = tau->y;

    tau_wedge[3*2 + 0] = 0.0f;
    tau_wedge[3*2 + 1] = 0.0f;
    tau_wedge[3*2 + 2] = 0.0f;

}


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
void vee_se2(
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
        InEKF_SE2_t* filter,
        float dt,
        state_se2_t process_noise,
        float mag_noise,
        float chi2_threshold)
{
    
}

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

        float v_L,
        float v_R,
        float omega_gyro);

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
        float theta_mag,
        float mag_norm);

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
        InEKF_SE2_t* filter,
        state_se2_t* state_out);


// void inEKF_SE2_get_covariance_trace(
//         InEKF_SE2_t* filter,
//         float trace);


/**
 * @brief wraps an angle to the range [-pi, pi]
 * 
 * @param theta pointer to the angle to be wrapped
 */
inline void inEKF_SE2_wrap_angle(
        float* theta);

