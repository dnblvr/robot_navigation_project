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
    tau_wedge[R_00]  =  0.f;
    tau_wedge[R_01]  = -tau->theta;
    tau_wedge[R_10]  =  tau->theta;
    tau_wedge[R_11]  =  0.f;
    
    tau_wedge[T_x]   =  tau->x;
    tau_wedge[T_y]   =  tau->y;

    tau_wedge[Z_20]  =  0.f;
    tau_wedge[Z_21]  =  0.f;
    tau_wedge[I_22]  =  0.f;

}


inline void vee_se2(
        float           tau_wedge[TOTAL],
        state_se2_t*    tau)
{
    tau->x      = tau_wedge[T_x];
    tau->y      = tau_wedge[T_y];
    tau->theta  = tau_wedge[R_10];
}


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
        float           exp_tau[TOTAL])
{
    float v_x   = tau->x;
    float v_y   = tau->y;
    float omega = tau->theta;


    // for a small angular rate, use first-order Taylor expansion
    if ( SMALL_ANGLE(omega) ) {

        exp_tau[R_00]   =  1.f;
        exp_tau[R_01]   = -omega;
        exp_tau[R_10]   =  omega;
        exp_tau[R_11]   =  1.f;

        exp_tau[T_x]    =  v_x;
        exp_tau[T_y]    =  v_y;

    
    } else {

        float s = sinf(omega);
        float c = cosf(omega);
                
        // calculate the V matrix for the translation part of the exponential
        // map. to see the origins, it is well known in the literature
        float V[4] = {        s/omega, -(1.f - c)/omega,
                      (1.f - c)/omega,          s/omega};
        
        // calculate the velocity vector in the tangent space
        float v_vec[2] = {v_x,
                          v_y};

        
        // fill in the exponential map matrix:
        //  - rotational part: R(theta)
        exp_tau[R_00]   =  c;
        exp_tau[R_01]   = -s;
        exp_tau[R_10]   =  s;
        exp_tau[R_11]   =  c;
        
        //  - translation part: V @ v_vec
        exp_tau[T_x]    = V[0*2 + 0]*v_vec[0] + V[0*2 + 1]*v_vec[1];
        exp_tau[T_y]    = V[1*2 + 0]*v_vec[0] + V[1*2 + 1]*v_vec[1];
        
    }

    // pre-fill the exponential map matrix with the common elements
    exp_tau[Z_20]   =  0.f;
    exp_tau[Z_21]   =  0.f;
    exp_tau[I_22]   =  1.f;

}


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
        state_se2_t*    tau)
{
    
    float omega = atan2f(exp_tau[R_10], exp_tau[R_00]);

    // translation part of the logarithm map
    float t_x = exp_tau[T_x];
    float t_y = exp_tau[T_y];

    
    // for a small angular rate, use first-order Taylor expansion
    if ( SMALL_ANGLE(omega) ) {

        tau->x      = t_x;
        tau->y      = t_y;

    } else {

        // common factors for the logarithm map
        float s = sinf(omega);
        float c = cosf(omega);
        float f = omega / (2*(1.f - c));

        // calculate the V_inv matrix for the translation part of the logarithm
        // map. to see the origins, it is well known in the literature
        float V_inv[4] = { f*s,         f*(1.f - c),
                          -f*(1.f - c), f*s         };

        
        // fill in the logarithm map / tangent space element:
        //  - translational part: V_inv @ t_vec
        tau->x  = V_inv[0*2 + 0]*t_x + V_inv[0*2 + 1]*t_y;
        tau->y  = V_inv[1*2 + 0]*t_x + V_inv[1*2 + 1]*t_y;

    }

    // pre-fill the exponential map matrix with the common elements
    tau->theta  = omega;

}


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
        float           adj_exp_tau[TOTAL])
{
    // rotation matrix part of the adjoint map is the same as the rotation
    // matrix part of the exponential map
    adj_exp_tau[R_00]  =  exp_tau[R_00];
    adj_exp_tau[R_01]  =  exp_tau[R_01];
    adj_exp_tau[R_10]  =  exp_tau[R_10];
    adj_exp_tau[R_11]  =  exp_tau[R_11];

    // translation part of the adjoint map is given by the skew-symmetric matrix
    // formed by the translation part of the exponential map
    adj_exp_tau[T_x]   = -exp_tau[T_y];
    adj_exp_tau[T_y]   =  exp_tau[T_x];

    // pre-fill the adjoint map matrix with the common elements
    adj_exp_tau[Z_20]  =  0.f;
    adj_exp_tau[Z_21]  =  0.f;
    adj_exp_tau[I_22]  =  1.f;

}


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

        float           dt,
        state_se2_t*    process_noise,
        float           mag_noise,
        float           chi2_threshold)
{

    filter->dt = dt;

    // length of the differential-drive robot's wheelbase
    filter->L = 0.3f; // m;
    
    filter->alpha = 0.5f; // complementary filter parameter for fusing gyro and
                          // encoder measurements; can be tuned based on the
                          // expected noise characteristics of the sensors

    filter->mag_norm_min = 2000.0f; // minimum magnetometer norm for outlier
    filter->mag_norm_max = 6500.0f; // maximum magnetometer norm for outlier


    filter->chi2_threshold = chi2_threshold;

    // initialize state to identity, or (x, y, \theta) = (0, 0, 0)
    filter->state.x      = 0.0f;
    filter->state.y      = 0.0f;
    filter->state.theta  = 0.0f;

    // set process noise covariance matrix Q as diagonal with provided values
    filter->process_noise.x     = process_noise->x;
    filter->process_noise.y     = process_noise->y;
    filter->process_noise.theta = process_noise->theta;


    // set measurement noise covariance matrix R as diagonal with provided
    // magnetometer noise for theta
    

}

/**
 * @brief InEKF prediction step along the SE(2) manifold for a differential-
 *  drive robot with gyro and encoder
 * 
 * @param[inout] filter InEKF_SE2_t struct containing the current state estimate, 
 *  covariance, and other filter parameters
 * @param[in] v_L left wheel velocity from encoders
 * @param[in] v_R right wheel velocity from encoders
 * @param[in] omega_gyro IMU angular velocity measurement from gyro
 */
void inEKF_SE2_predict(
        InEKF_SE2_t* filter,

        float   v_L,
        float   v_R,
        float   omega_gyro)
{

}

/**
 * @brief InEKF update step using magnetometer measurements
 * 
 * @param[inout] filter InEKF_SE2_t struct containing the current state estimate, 
 *  covariance, and other filter parameters
 * @param[in] theta_mag Magnetometer heading measurement
 * @param[in] mag_norm Magnetometer measurement norm
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
 * @param[inout] filter InEKF_SE2_t struct containing the current state estimate, 
 *  covariance, and other filter parameters
 * @param[out] state_out state_se2_t struct to store the retrieved state estimate
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
        float* theta);

