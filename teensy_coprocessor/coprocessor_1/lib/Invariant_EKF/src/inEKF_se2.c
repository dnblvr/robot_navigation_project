/**
 * @file inEKF_se2.c
 * @author your name (you@domain.com)
 * @brief port specialized for the Teensy 4.0
 * @version 0.1
 * @date 2026-03-01
 */

 #include "inEKF_se2.h"


// ————————————————————————————————————————————————————————————————————————————
//
//  INVARIANT EKF FUNCTIONS
//
// ————————————————————————————————————————————————————————————————————————————


void inEKF_SE2_init(
        InEKF_SE2_t* filter,

        float           dt,
        float           L,
        se2_t*    process_noise,
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
    filter->mag_norm_min = 40.f;
    filter->mag_norm_max = 60.f;


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
    se2_t u = {0};

    
    float X_delta[TOTAL] = {0}; // state increment in the Lie algebra se(2)

    // adjoint and adjoint-transpose matrices for covariance propagation
    float Ad[TOTAL]     = {0};

    float c, s; // cosine and sine of the angular increment for the exponential map

    // —— control input ———————————————————————————————————————————————————————
    // for the dynamics of a differential-drive robot:
    //  - linear velocity v is the average of the left and right wheel
    //      velocities
    //  - angular velocity omega is the difference of the right and left wheel
    //      velocities divided by the wheelbase

    v               = (v_L + v_R) * 0.5f;
    omega_encoders  = (v_R - v_L) * filter->inv_L;
    omega           =   filter->alpha * omega_gyro
                      + (1.0f - filter->alpha) * omega_encoders;
    
    // u consists of [v*dt, 0, omega*dt] from the robot body-frame
    u.x     = v * filter->dt;
    u.y     = 0.0f;
    u.theta = omega * filter->dt;

    // translating the control input into the Lie Group se(2)
    exp_se2(&u, X_delta);


    // —— state propagation ———————————————————————————————————————————————————
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


    // —— covariance propagation ——————————————————————————————————————————————
    // P = Ad * P * Ad^T + Q
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


    // —— innovation gate —————————————————————————————————————————————————————
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
        S   = H_mag * filter->covariance[M_22] * H_mag  +  filter->mag_noise;
    
        // calculate mahalanobis distance for outlier rejection
        // mahalanobis_distance = y * (1 / S) * y;
        mahalanobis_distance = (y * y) / S;
    
        // reject magnetometer update if an outlier
        if (mahalanobis_distance > filter->chi2_threshold) {
            return 2;
        }
    
        // Kalman gain K = P*H^T*S^-1
        // again, this is a simple scalar in this case since the measurement only considers 1 state in the state space: the heading. Under full state estimator conditions, we would use normal matrix multiplication operations.
        K[0]    = filter->covariance[M_02] / S;
        K[1]    = filter->covariance[M_12] / S;
        K[2]    = filter->covariance[M_22] * H_mag / S;

    }



    // —— state update on the manifold ————————————————————————————————————————
    // [K_00 K_01 K_02] * y
    {
        // current state estimate
        se2_t* X_ = &(filter->state);

        se2_t delta_xi_struct = {
                .x      = K[0] * y,
                .y      = K[1] * y,
                .theta  = K[2] * y};
    
        float delta_xi_exp[TOTAL] = {0};

    
        // map the state increment from the tangent space to the manifold
        exp_se2(&delta_xi_struct, delta_xi_exp); 
    
        // convert the state increment from the matrix representation to the `se2_t` struct for easier composition with the current state estimate
        matrix_to_state(delta_xi_exp, &delta_xi_struct); 
    
        // compose the state increment with the current state estimate to get the updated state estimate
        compose_SE2(*X_, delta_xi_struct, X_);

    }


    // —— covariance update ———————————————————————————————————————————————————
    // this will take the Joseph form of the covariance update
    {

        // variable declarations for the covariance update
        float I[TOTAL]      = I_3x3; // row-major identity matrix
    
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

