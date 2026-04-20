/**
 * @file inEKF_se2.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-01
 * 
 */

 #include "../inc/inEKF_se2.h"

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
    
    tau_wedge[T_x_]  =  tau->x;
    tau_wedge[T_y_]  =  tau->y;

    tau_wedge[Z_20]  =  0.f;
    tau_wedge[Z_21]  =  0.f;
    tau_wedge[I_22]  =  0.f;

}


inline void vee_se2(
        float           tau_wedge[TOTAL],
        state_se2_t*    tau)
{
    tau->x      = tau_wedge[T_x_];
    tau->y      = tau_wedge[T_y_];
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
    // 
    float v_x   = tau->x;
    float v_y   = tau->y;
    float omega = tau->theta;


    // for a small angular rate, use first-order Taylor expansion
    if ( SMALL_ANGLE(omega) ) {

        exp_tau[R_00]   =  1.f;
        exp_tau[R_01]   = -omega;
        exp_tau[R_10]   =  omega;
        exp_tau[R_11]   =  1.f;

        exp_tau[T_x_]   =  v_x;
        exp_tau[T_y_]   =  v_y;

    
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
        exp_tau[T_x_]    = V[0*2 + 0]*v_vec[0] + V[0*2 + 1]*v_vec[1];
        exp_tau[T_y_]    = V[1*2 + 0]*v_vec[0] + V[1*2 + 1]*v_vec[1];
        
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
    float t_x   = exp_tau[T_x_];
    float t_y   = exp_tau[T_y_];

    
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
    adj_exp_tau[T_x_]   = -exp_tau[T_y_];
    adj_exp_tau[T_y_]   =  exp_tau[T_x_];

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
        float           L,
        state_se2_t*    process_noise,
        float           mag_noise,
        float           chi2_threshold)
{
    // time step for the filter
    filter->dt      = dt;
    filter->inv_dt  = 1.0f / dt;


    // length of the differential-drive robot's wheelbase
    filter->L       = L; // m;
    filter->inv_L   = 1.0f / L;

    
    // complementary filter parameter for fusing gyro and encoder measurements;
    // can be tuned based on the expected noise characteristics of the sensors
    filter->alpha = 0.5f;
    

    // minimum & maximum magnetometer norm for outlier
    filter->mag_norm_min = 2000.0f; 
    filter->mag_norm_max = 6500.0f; 


    filter->chi2_threshold = chi2_threshold;
    
    
    // set process noise covariance matrix Q as diagonal with provided values
    float* Q    = &filter->process_noise[0];

    Q[M_00] = process_noise->x      * process_noise->x;
    Q[M_11] = process_noise->y      * process_noise->y;
    Q[M_22] = process_noise->theta  * process_noise->theta;

    Q[M_01] = Q[M_10] = 0.f; // no covariance between x and y
    Q[M_02] = Q[M_20] = 0.f; // no covariance between x and theta
    Q[M_12] = Q[M_21] = 0.f; // no covariance between y and theta
    
    
    /**
     * Set measurement noise covariance matrix R as diagonal with provided
     * magnetometer noise for theta
     * 
     * @note this is a scalar representation since the update is only for the
     *  heading measurement. otherwise, it would be like this: 
     */
    // float *R = &filter->measurement_noise[0];
    filter->mag_noise = mag_noise*mag_noise;

    
    // initialization of the covariance matrix P; can be set to a diagonal
    // matrix
    float* P    = &filter->covariance[0];

    P[M_00] = 0.1f; // initial variance for x
    P[M_11] = 0.1f; // initial variance for y
    P[M_22] = 0.1f; // initial variance for theta
    
    P[M_01] = P[M_10] = 0.f; // no initial covariance between x and y
    P[M_02] = P[M_20] = 0.f; // no initial covariance between x and theta
    P[M_12] = P[M_21] = 0.f; // no initial covariance between y and theta


    // initialize state to identity, or (x, y, \theta) = (0, 0, 0)
    filter->state.x      = 0.0f;
    filter->state.y      = 0.0f;
    filter->state.theta  = 0.0f;

}

void inEKF_SE2_predict(
        InEKF_SE2_t* filter,

        float   v_L,
        float   v_R,
        float   omega_gyro)
{

    float v;              // linear velocity
    float omega_encoders; // angular velocity from encoders
    float omega;          // fused angular velocity


    // control input u
    state_se2_t u = {0};

    
    float X_delta[TOTAL] = {0}; // state increment in the Lie algebra se(2)

    // adjoint and adjoint-transpose matrices for covariance propagation
    float Ad[TOTAL]     = {0};

    float c, s; // cosine and sine of the angular increment for the exponential map

    // control input ----------------------------------------------------------
    // for the dynamics of a differential-drive robot:
    //  - linear velocity v is the average of the left and right wheel
    //      velocities
    //  - angular velocity omega is the difference of the right and left wheel
    //      velocities divided by the wheelbase

    v               = (v_L + v_R) * 0.5f;
    omega_encoders  = (v_R - v_L) * filter->inv_L;
    omega           =   filter->alpha * omega_gyro
                      + (1.0f - filter->alpha) * omega_encoders;
    
    // u consists of [v*dt, 0, omega*dt] from the local robot frame
    u.x     = v * filter->dt;
    u.y     = 0.0f;
    u.theta = omega * filter->dt;

    // translating the control input into the Lie algebra se(2)
    exp_se2(&u, X_delta);


    // state propagation ------------------------------------------------------
    /**
     * update the state estimate by composing the current state with the state
     *  increment
     * 
     * @note similar to composing the left-invariant error with the current
     *  state estimate but using expressions instead of matrix multiplication
     */
    c   = cosf(filter->state.theta);
    s   = sinf(filter->state.theta);

    filter->state.x     += c*X_delta[T_x_] - s*X_delta[T_y_];
    filter->state.y     += s*X_delta[T_x_] + c*X_delta[T_y_];
    filter->state.theta += u.theta; // directly added to the current heading since the angular increment is already in the local frame


    // covariance propagation -------------------------------------------------
    // first, compute the adjoint and adjoint-transpose of the state increment
    adjoint_se2(X_delta, Ad);

    // then, propagate the covariance `P` in the tangent space using the ad-
    // joint representation of the state increment and adding process noise
    congruence_3x3(Ad,
                   filter->covariance,
                   filter->covariance);

    matadd_3x3(filter->covariance,
               filter->process_noise,
               filter->covariance);

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
        float   mag_norm)
{
    // reject magnetometer measurement (return early) as an outlier if the norm
    // is outside the expected range
    if (    mag_norm < filter->mag_norm_min
         || mag_norm > filter->mag_norm_max)
    {
        return 1;
    }

    int i; // loop variable for later matrix operations


    // innovation gate --------------------------------------------------------
    // For outlier rejection based on the innovation / residual and its covariance. This is especially important for magnetometer measurements which can be very noisy and have outliers due to magnetic disturbances in the environment. By calculating the Mahalanobis distance of the innovation, we can reject measurements that are unlikely given our current state estimate and covariance.

    // kalman gain
    float K[DIMS]   = {0};

    // predicted heading from the state estimate
    float theta_hat = filter->state.theta;

    // innovation / residual
    float y         = _wrap_angle(theta_mag - theta_hat);

    // magnetometer measurement model, which only measures the heading
    float H_mag     = 1.f;

    {
        // innovation covariance
        float S     = 0;
    
        // mahalanobis distance for outlier rejection
        float mahalanobis_distance = 0;
    
        
        /**
         * @brief calculate the innovation covariance S = H*P*H^T + R
         * @note in this case, it is a simple scalar since the measurement only
         *  considers 1 state in the state space: the heading. Under full state
         *  estimator conditions, we would use normal matrix multiplication
         *  operations.
         */
        S = H_mag * filter->covariance[M_22] * H_mag  +  filter->mag_noise;
    
        // calculate mahalanobis distance for outlier rejection
        // mahalanobis_distance = y * (1 / S) * y;
        mahalanobis_distance = (y * y) / S;
    
        // reject magnetometer update if an outlier
        if (mahalanobis_distance > filter->chi2_threshold) {
            return 1;
        }
    
        // Kalman gain K = P*H^T*S^-1
        // again, this is a simple scalar in this case since the measurement only considers 1 state in the state space: the heading. Under full state estimator conditions, we would use normal matrix multiplication operations.
        K[0]    = filter->covariance[M_02] / S;
        K[1]    = filter->covariance[M_12] / S;
        K[2]    = filter->covariance[M_22] * H_mag / S;

    }



    // state update on the manifold -------------------------------------------
    // [K_00 K_01 K_02] * y
    {
        // current state estimate
        state_se2_t* X_ = &(filter->state);

        state_se2_t delta_xi_struct = {
                .x      = K[0] * y,
                .y      = K[1] * y,
                .theta  = K[2] * y};
    
        float delta_xi_exp[TOTAL] = {0};

    
        // map the state increment from the tangent space to the manifold
        exp_se2(&delta_xi_struct, delta_xi_exp); 
    
        // convert the state increment from the matrix representation to the state_se2_t struct for easier composition with the current state estimate
        matrix_to_state(delta_xi_exp, &delta_xi_struct); 
    
        // compose the state increment with the current state estimate to get the updated state estimate
        compose_SE2(*X_, delta_xi_struct, X_);

    }


    // covariance update ------------------------------------------------------
    // this will take the Joseph form of the covariance update
    {

        // variable declarations for the covariance update
        float I[TOTAL]      = IDENTITY;
    
        float H[DIMS]       = {-0.f, -0.f, -H_mag};
    
        float KH[TOTAL]     = {0};
        
        float I_KH[TOTAL]   = {0};
    
        float C1[TOTAL]     = {0};
        float C2[TOTAL]     = {0};

    
        // calculate KH = K*H
        matmul_3_1x1_3(K, H, KH);
    
        // calculate I - KH
        // @note KH is already negated since H is negative
        matadd_3x3(I, KH, I_KH);
    
        // calculate congruence for the covariance update:
        //  1. (I - KH)*P*(I - KH)^T
        //  2. K*R*K^T
        //     a. R * K*K^T if R is a scalar
        // @note in the full state estimation case, R in (2) would be a full measurement noise covariance matrix with dimensions 3x3 so we would use the normal `congruence_3x3()` function 
        congruence_3x3(I_KH, filter->covariance, C1);
    
        // calculate K*R*K^T using the scalar R as `filter->mag_noise`
        matmul_3_1x1_3(K, K, C2);
    
        for (i = 0; i < TOTAL; i++) {
            C2[i] *= filter->mag_noise;
        }
    
        // final covariance update
        matadd_3x3(C1, C2, filter->covariance);

    }

    // update if successful
    return 0;
}

/** ---------------------------------------------------------
 *  HELPER FUNCTIONS
 */

void matrix_to_state(
        float           state_matrix[TOTAL],
        state_se2_t*    state)
{
    state->x      = state_matrix[T_x_];
    state->y      = state_matrix[T_y_];
    state->theta  = atan2f(state_matrix[R_10], state_matrix[R_00]);
}


void compose_SE2(
        state_se2_t     A,
        state_se2_t     B,
        state_se2_t*    state_out)
{
    float c = cosf(A.theta);
    float s = sinf(A.theta);

    state_out->x     =  c*B.x - s*B.y + A.x;
    state_out->y     =  s*B.x + c*B.y + A.y;
    state_out->theta = _wrap_angle(A.theta + B.theta);
}

void inverse_3x3(
        float   A[TOTAL],
        float   A_inv[TOTAL])
{

}



void matmul_3x3(
        float   A[TOTAL],
        float   B[TOTAL],
        float   AB[TOTAL])
{
    #define LOOP_UNROLLING 1

    // M_00, M_01, M_02
    // M_10, M_11, M_12
    // M_20, M_21, M_22

    #ifdef LOOP_UNROLLING

    // definitions
    float A_00 = A[M_00], A_01 = A[M_01], A_02 = A[M_02];
    float A_10 = A[M_10], A_11 = A[M_11], A_12 = A[M_12];
    float A_20 = A[M_20], A_21 = A[M_21], A_22 = A[M_22];


    // row 1
    AB[M_00] = A_00*B[M_00] + A_01*B[M_10] + A_02*B[M_20];
    AB[M_01] = A_00*B[M_01] + A_01*B[M_11] + A_02*B[M_21];
    AB[M_02] = A_00*B[M_02] + A_01*B[M_12] + A_02*B[M_22];

    // row 2
    AB[M_10] = A_10*B[M_00] + A_11*B[M_10] + A_12*B[M_20];
    AB[M_11] = A_10*B[M_01] + A_11*B[M_11] + A_12*B[M_21];
    AB[M_12] = A_10*B[M_02] + A_11*B[M_12] + A_12*B[M_22];

    // row 3
    AB[M_20] = A_20*B[M_00] + A_21*B[M_10] + A_22*B[M_20];
    AB[M_21] = A_20*B[M_01] + A_21*B[M_11] + A_22*B[M_21];
    AB[M_22] = A_20*B[M_02] + A_21*B[M_12] + A_22*B[M_22];

    #else 

    // single for loop method

    int i;

    // row X
    for (i = 0; i < TOTAL; i += DIMS) {
        float A_i0 = A[i + 0], A_i1 = A[i + 1], A_i2 = A[i + 2];

        AB[i + 0] = A_i0*B[M_00] + A_i1*B[M_10] + A_i2*B[M_20];
        AB[i + 1] = A_i0*B[M_01] + A_i1*B[M_11] + A_i2*B[M_21];
        AB[i + 2] = A_i0*B[M_02] + A_i1*B[M_12] + A_i2*B[M_22];
    }

    #endif

}


void matmul_3_1x1_3(
        float   A[DIMS],
        float   B[DIMS],
        float   AB[TOTAL])
{
    // [A_00
    //  A_10  * [B_00, B_01, B_02]
    //  A_20] 

    AB[M_00] = A[0]*B[0];   AB[M_01] = A[0]*B[1];   AB[M_02] = A[0]*B[2];

    AB[M_10] = A[1]*B[0];   AB[M_11] = A[1]*B[1];   AB[M_12] = A[1]*B[2];

    AB[M_20] = A[2]*B[0];   AB[M_21] = A[2]*B[1];   AB[M_22] = A[2]*B[2];

}


void congruence_3x3(
        float   A[TOTAL],
        float   B[TOTAL],
        float   ABA_T[TOTAL])
{

    float AB[TOTAL]     = {0};
    float A_T[TOTAL]    = {0};

    matmul_3x3(A, B, AB);

    transpose_3x3(A, A_T);

    matmul_3x3(AB, A_T, ABA_T);

}

inline void matadd_3x3(
        float   A[TOTAL],
        float   B[TOTAL],
        float   AB[TOTAL])
{
    #ifdef LOOP_UNROLLING

    AB[M_00] = A[M_00] + B[M_00];
    AB[M_01] = A[M_01] + B[M_01];
    AB[M_02] = A[M_02] + B[M_02];

    AB[M_10] = A[M_10] + B[M_10];
    AB[M_11] = A[M_11] + B[M_11];
    AB[M_12] = A[M_12] + B[M_12];

    AB[M_20] = A[M_20] + B[M_20];
    AB[M_21] = A[M_21] + B[M_21];
    AB[M_22] = A[M_22] + B[M_22];

    #else

    int i;

    for (i = 0; i < TOTAL; i++) {
        AB[i] = A[i] + B[i];
    }

    #endif
}


inline void transpose_3x3(
        float   matrix_in[TOTAL],
        float   matrix_out[TOTAL])
{

    // assigning lower triangular elements to upper
    matrix_out[R_10] = matrix_in[R_01];
    matrix_out[Z_20] = matrix_in[T_x_];
    matrix_out[Z_21] = matrix_in[T_y_];
    
    // assigning upper triangular elements to lower
    matrix_out[R_01] = matrix_in[R_10];
    matrix_out[T_x_] = matrix_in[Z_20];
    matrix_out[T_y_] = matrix_in[Z_21];

    // keeping diagonal elements the same
    matrix_out[R_00] = matrix_in[R_00];
    matrix_out[R_11] = matrix_in[R_11];
    matrix_out[I_22] = matrix_in[I_22];

}


void inEKF_SE2_get_state(
        InEKF_SE2_t*    filter,
        state_se2_t*    state_out)
{
    // state_out->x      = filter->state.x;
    // state_out->y      = filter->state.y;
    // state_out->theta  = filter->state.theta;

    memcpy(state_out, &filter->state, sizeof(state_se2_t));
}


// void inEKF_SE2_get_covariance_trace(
//         InEKF_SE2_t* filter,
//         float trace);


/**
 * @brief wraps an angle to the range [-pi, pi]
 * 
 * @param theta angle to be wrapped
 * @return wrapped angle in the range [-pi, pi]
 */
inline float _wrap_angle(
        float theta)
{
    while (theta > M_PI_F) {
        theta -= 2.0f * M_PI_F;
    }
    while (theta < -M_PI_F) {
        theta += 2.0f * M_PI_F;
    }
    
    return theta;
}

