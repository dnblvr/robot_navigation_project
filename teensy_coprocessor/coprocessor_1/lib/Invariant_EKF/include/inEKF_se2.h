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

#ifdef __cplusplus
extern "C" {
#endif

// ————————————————————————————————————————————————————————————————————————————
//
//  DATA STRUCTURES
//
// ————————————————————————————————————————————————————————————————————————————

/**
 * @brief alternate representation of pi, if not declared by the system
 */
#ifndef M_PI_F
  #define M_PI_F 3.1415927f
#endif

/**
 * @brief number of dimensions in the `SE(2)` state space
 */
#define DIMS    3

/**
 * @brief number of elements in the matrix representation of the `SE(2)`
 *  exponential map and adjoint map
 */
#define TOTAL   (DIMS*DIMS)

/**
 * @brief `SE(2)` state struct for a differential-drive robot with a 2D
 *  magnetometer for heading measurements
 * 
 * @param x x-position in the plane
 * @param y y-position in the plane
 * @param theta heading angle in the plane
 */
typedef struct {
    float   x;
    float   y;
    float   theta;
} state_se2_t;

/**
 * @brief identity element of `SE(2)` for state initialization and other
 *  operations
 */
#define IDENTITY_SE2 {0.0f, 0.0f, 0.0f}


/**
 * @brief zero element of `SE(2)` for state initialization and other
 *  operations
 */
#define Z_3x1 {0.0f, 0.0f, 0.0f}

/**
 * @brief identity element of `SE(2)` for matrix state initialization and other
 *  operations
 */
#define I_3x3 {1.f, 0.f, 0.f,   \
               0.f, 1.f, 0.f,   \
               0.f, 0.f, 1.f}

/**
 * @brief zero element of `SE(2)` for matrix state initialization and other
 *  operations
 */
#define Z_3x3 {0.f, 0.f, 0.f,   \
               0.f, 0.f, 0.f,   \
               0.f, 0.f, 0.f}

/**
 * @brief common indices for the rotation matrix elements in the `SE(2)`
 *  exponential map and adjoint map
 */
#define R_00 (3*0 + 0)  // 0
#define R_01 (3*0 + 1)  // 1
#define R_10 (3*1 + 0)  // 3
#define R_11 (3*1 + 1)  // 4

#define T_x_ (3*0 + 2)  // 2
#define T_y_ (3*1 + 2)  // 5

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
 * @brief indices for a 2x2 `V` matrix in row-major order, for ease of use in C
 */
#define V_00 (2*0 + 0)  // 0
#define V_01 (2*0 + 1)  // 1
#define V_10 (2*1 + 0)  // 2
#define V_11 (2*1 + 1)  // 3


/**
 * @brief macro to check if an angle is small enough to use the first-order
 *  Taylor expansion for the exponential map
 */
#define SMALL_ANGLE(angle) (fabsf(angle) < 1e-6f)

/**
 * @brief struct to hold all variables related to the invariant EKF for `SE(2)`
 *  state estimation and specifically for a differential-drive robot with a 2D
 *  magnetometer for heading measurements
 * 
 * @details The state is stored as a 1D array for ease of use in C, but the
 *  order of the elements is [x, y, theta] to match the `state_se2_t` struct
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
    state_se2_t state;

    // process noise covariance matrix, aka `Q`
    // state_se2_t process_noise;
    float process_noise[TOTAL];

    // innovation gate threshold for outlier rejection
    float chi2_threshold;

    // covariance in tangent space, aka `P`
    // state_se2_t covariance;
    float covariance[TOTAL];

    // measurement noise covariance matrix, aka `R`
    // state_se2_t measurement_noise;
    float mag_noise;

} InEKF_SE2_t;


// ————————————————————————————————————————————————————————————————————————————
//
//  LIE GROUP HELPER FUNCTIONS
//
// ————————————————————————————————————————————————————————————————————————————

/**
 * @brief hat operator for `SE(2)` Lie algebra
 * 
 * @details maps the Lie algebra element tau in the tangent space to a matrix
 *  in the Lie group `SE(2)` with generators corresponding to the order of the
 *  state vector, i.e. [x, y, theta]
 * 
 * @note `tau_wedge` is a 3x3 skew-symmetric matrix in the Lie group `SE(2)`
 * 
 * @param[in] tau       `state_se2_t` tangent space (Lie algebra `se(2)`) vector
 * @param[out] tau_wedge 3x3 matrix in the Lie group `SE(2)`
 */
inline void wedge_se2(
        const state_se2_t*  tau,
              float         tau_wedge[TOTAL])
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

/**
 * @brief vee operator for `SE(2)` Lie algebra, a.k.a. the inverse of the hat
 *  operator
 * 
 * @details maps a matrix in the Lie group `SE(2)` to a vector in the tangent
 *  space, i.e. the Lie algebra `se(2)`
 * 
 * @param[in]  tau_wedge 3x3 matrix in the Lie group `SE(2)`
 * @param[out] tau `state_se2_t` vector in the tangent space, aka the Lie
 *  algebra `se(2)`
 */
inline void vee_se2(
        const float         tau_wedge[TOTAL],
              state_se2_t*  tau)
{
    tau->x      =  tau_wedge[T_x_];
    tau->y      =  tau_wedge[T_y_];
    tau->theta  =  tau_wedge[R_10];
}


/**
 * @brief exponential map for `SE(2)` Lie algebra
 * 
 * @details maps a vector in the tangent space, i.e. the Lie algebra `se(2)`, to
 *  a matrix in the Lie group `SE(2)`
 * 
 * @param[in]  tau     state_se2_t vector in the tangent space, aka the Lie
 *  algebra `se(2)`
 * @param[out] exp_tau 3x3 matrix in the Lie group `SE(2)`
 */
void exp_se2(
        const state_se2_t*  tau,
              float         exp_tau[TOTAL]);


/**
 * @brief logarithm map for `SE(2)` Lie algebra
 * 
 * @details maps a matrix in the Lie group `SE(2)` to a vector in the tangent
 *  space, i.e. the Lie algebra `se(2)`
 * 
 * @param[in] exp_tau 3x3 matrix in the Lie group `SE(2)`
 * @param[out] tau `state_se2_t` vector in the tangent space, aka the Lie
 *  algebra `se(2)`
 */
void log_se2(
        const float         exp_tau[TOTAL],
              state_se2_t*  tau);


/**
 * @brief adjoint map for `SE(2)` Lie algebra
 * 
 * @details maps a matrix in the Lie group `SE(2)` to its adjoint representation
 *  in the Lie algebra `se(2)`
 * 
 * @param[in]  exp_tau     3x3 matrix in the Lie group `SE(2)`
 * @param[out] adj_exp_tau 3x3 matrix representing the adjoint in the `se(2)`
 *  Lie algebra 
 */
inline void adjoint_se2(
        float   exp_tau[TOTAL],
        float   adj_exp_tau[TOTAL])
{
    // rotation matrix part of the adjoint map is the same as the rotation
    // matrix part of the exponential map
    adj_exp_tau[R_00]  =  exp_tau[R_00];
    adj_exp_tau[R_01]  =  exp_tau[R_01];
    adj_exp_tau[R_10]  =  exp_tau[R_10];
    adj_exp_tau[R_11]  =  exp_tau[R_11];

    // translation part of the adjoint map is given by the skew-symmetric matrix
    // formed by the translation part of the exponential map
    adj_exp_tau[T_x_]  = -exp_tau[T_y_];
    adj_exp_tau[T_y_]  =  exp_tau[T_x_];

    // pre-fill the adjoint map matrix with the common elements
    adj_exp_tau[Z_20]  =  0.f;
    adj_exp_tau[Z_21]  =  0.f;
    adj_exp_tau[I_22]  =  1.f;
}


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
        state_se2_t*    process_noise,
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
 *  HELPER FUNCTIONS
 */

/**
 * @brief  helper function to wrap an angle to the range [-pi, pi]
 * 
 * @note this function will be used in the composition of the state increment
 *  along with other helper functions for `SE(2)` operations
 * 
 * @param[in] theta angle in radians
 * @return `float` wrapped angle in radians
 * @retval wrapped angle in the range [-pi, pi]
 */
inline float _wrap_angle(float theta)
{
    while (theta > M_PI_F) {
        theta  -= 2.0f * M_PI_F;
    }
    while (theta < -M_PI_F) {
        theta  += 2.0f * M_PI_F;
    }
    
    return theta;
}

/**
 * @brief helper function to convert a 3x3 matrix in row-major order to a
 *  `state_se2_t` struct, by extracting the translation and rotation components
 *  of the matrix
 * 
 * @param state_matrix 3x3 matrix in row-major order representing an `SE(2)`
 *  pose
 * @param state pointer to `state_se2_t` struct to store the converted state
 */
inline void matrix_to_state(
        const float         state_matrix[TOTAL],
              state_se2_t*  state)
{
    state->x        = state_matrix[T_x_];
    state->y        = state_matrix[T_y_];
    state->theta    = atan2f(state_matrix[R_10],
                             state_matrix[R_00]);
}

/**
 * @brief  
 * 
 * @param A 
 * @param B 
 * @param state_out 
 */
inline void compose_SE2(
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

/**
 * @brief  
 * 
 * @param A pose to "subtract" from B, i.e. the current state estimate
 * @param B pose that is one or more timesteps ahead of A
 * @param state_out delta pose that transforms A to B, i.e. the state increment
 *  from A to B, i.e.
 * 
 *  - `delta_X` = `inv(A)` @ `B`
 */
inline void difference_SE2(
        state_se2_t     A,
        state_se2_t     B,
        state_se2_t*    state_out)
{
    float c     = cosf(A.theta);
    float s     = sinf(A.theta);
    float dx    = B.x - A.x;
    float dy    = B.y - A.y;

    /**
     * @note follow this formula, as this produces the opposing rotation
     *  followed by the translation:
     * 
     * delta_x =   R^T_A @ (t_B - t_A)
     * delta_x = { cos(theta_A) sin(theta_A)    {dx
     *            -sin(theta_A) cos(theta_A) }   dy} 
     */
    state_out->x        =  c*dx + s*dy;
    state_out->y        = -s*dx + c*dy;
    state_out->theta    = _wrap_angle(B.theta - A.theta);
}

/**
 * @brief measures changes in position between two SE(2) poses, ignoring the heading component
 * 
 * @param A 
 * @param B 
 * @return float 
 */
inline float euclidean_distance_SE2(
        state_se2_t A,
        state_se2_t B)
{
    float dx = B.x - A.x;
    float dy = B.y - A.y;

    return sqrtf(dx*dx + dy*dy);
}
        

/** ---------------------------------------------------------
 *  HELPER FUNCTIONS - lie group operations
 */

/**
 * @brief matrix inverse for 3x3 matrices
 * 
 * @note this function, if implemented, will be heavily simplified and
 *  optimized. otherwise, I will add this for future use
 * 
 * @param A 
 * @param A_inv 
 */
inline void inverse_3x3(
        float   A[TOTAL],
        float   A_inv[TOTAL])
{

}


/**
 * @brief simple matrix multiplication for 3x3 matrices
 * 
 * @param A 
 * @param B 
 * @param AB 
 */
inline void matmul_3x3(
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


/**
 * @brief simple matrix multiplication for 3x3 matrix and 3x1 vector
 * 
 * @param[in]  A  input 3x3 matrix in row-major order
 * @param[in]  b  input 3x1 vector
 * 
 * @param[out] Ab  output 3x1 vector
 */
inline void matmul_3x3_3x1(
        const float A[TOTAL],
        const float b[DIMS],
              float Ab[DIMS])
{
    // M_00, M_01, M_02
    // M_10, M_11, M_12
    // M_20, M_21, M_22

    // definitions
    float b_0 = b[0], b_1 = b[1], b_2 = b[2];
    
    Ab[0]   = A[M_00]*b_0  +  A[M_01]*b_1  +  A[M_02]*b_2;
    Ab[1]   = A[M_10]*b_0  +  A[M_11]*b_1  +  A[M_12]*b_2;
    Ab[2]   = A[M_20]*b_0  +  A[M_21]*b_1  +  A[M_22]*b_2;
}

/**
 * @brief element-wise addition of two 3x3 matrices
 * 
 * @param A 
 * @param B 
 * @param AB 
 */
inline void matadd_3x3(
        const float A[TOTAL],
        const float B[TOTAL],
              float AB[TOTAL])
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

/**
 * @brief matrix multiplication for the specific 3x1 by 1x3 case
 * 
 * @note used for calculating `K*R*K^T` in the covariance update step of the
 *  magnetometer measurement update
 */
inline void matmul_3_1x1_3(
        const float   A[DIMS],
        const float   B[DIMS],
              float   AB[TOTAL])
{
    // [A_00
    //  A_10  * [B_00, B_01, B_02]
    //  A_20] 

    AB[M_00] = A[0]*B[0];   AB[M_01] = A[0]*B[1];   AB[M_02] = A[0]*B[2];

    AB[M_10] = A[1]*B[0];   AB[M_11] = A[1]*B[1];   AB[M_12] = A[1]*B[2];

    AB[M_20] = A[2]*B[0];   AB[M_21] = A[2]*B[1];   AB[M_22] = A[2]*B[2];

}

/**
 * @brief replicates the matrix product `A*B*A^T` for 3x3 matrices
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
 * @brief helper function to transpose a 3x3 `se(2)` matrix
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

/**
 * @brief retrieves the current state estimate from the filter struct
 * 
 * @param[in] filter `InEKF_SE2_t` struct containing the current state
 *      estimate, covariance, and other filter parameters
 * @param[out] state_out `state_se2_t` struct to store the retrieved state
 *      estimate
 */
inline void inEKF_SE2_get_state(
        InEKF_SE2_t*    filter,
        state_se2_t*    state_out)
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
