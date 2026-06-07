/**
 * @file state_se2.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1 
 */
#include <state_se2.h>


// ————————————————————————————————————————————————————————————————————————————
//
//  LIE GROUP HELPER FUNCTIONS
//
// ————————————————————————————————————————————————————————————————————————————


void exp_se2(
        const state_se2_t*  tau,
              float         exp_tau[TOTAL])
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
        exp_tau[T_x_]   = V[V_00]*v_vec[0]  +  V[V_01]*v_vec[1];
        exp_tau[T_y_]   = V[V_10]*v_vec[0]  +  V[V_11]*v_vec[1];
        
    }

    // pre-fill the exponential map matrix with the common elements
    exp_tau[Z_20]   =  0.f;
    exp_tau[Z_21]   =  0.f;
    exp_tau[I_22]   =  1.f;

}


void log_se2(
        const float         exp_tau[TOTAL],
              state_se2_t*  tau)
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
        tau->x  = V_inv[V_00]*t_x  +  V_inv[V_01]*t_y;
        tau->y  = V_inv[V_10]*t_x  +  V_inv[V_11]*t_y;

    }

    // pre-fill the exponential map matrix with the common elements
    tau->theta  = omega;

}

// ────────────────────────────────────────────────────────────────────────────
//
//  HELPER FUNCTIONS
//
// ────────────────────────────────────────────────────────────────────────────

float normalize_angle(float angle)
{
    while (angle > M_PI_F)
        angle -= 2.0f * M_PI_F;

    while (angle < -M_PI_F)
        angle += 2.0f * M_PI_F;
        
    return angle;
}


float pose_distance(
        const Pose* pose1,
        const Pose* pose2)
{
    float dx, dy;

    dx  = pose2->x - pose1->x;
    dy  = pose2->y - pose1->y;

    return sqrtf(dx*dx + dy*dy);
}


void transform_point_cloud(
        const PointCloud*   scan,
        const Pose*         pose,
              PointCloud*   out_scan)
{
    // counter
    uint32_t i;

    // If rotation is negligible, skip it for efficiency
    if (fabsf(pose->theta) < 1e-3f) {

        for (i = 0; i < scan->num_pts; i++) {
            out_scan->points[i].x   = scan->points[i].x + pose->x;
            out_scan->points[i].y   = scan->points[i].y + pose->y;
        }

        out_scan->num_pts   = scan->num_pts;
        return;
    }

    // Pre-compute cosine and sine of rotation angle
    float c = cosf(pose->theta);
    float s = sinf(pose->theta);
    
    // Apply rotation and translation to each point
    for (i = 0; i < scan->num_pts; i++) {

        float x = scan->points[i].x;
        float y = scan->points[i].y;
        
        out_scan->points[i].x   = c*x - s*y + pose->x;
        out_scan->points[i].y   = s*x + c*y + pose->y;
    }

    out_scan->num_pts   = scan->num_pts;
}


void compose_poses(
        const Pose*	p2,
        const Pose*	p1,
              Pose* result)
{

    float c = cosf(p1->theta);
    float s = sinf(p1->theta);
    
    result->x       = p1->x  +  c*p2->x  -  s*p2->y;
    result->y       = p1->y  +  s*p2->x  +  c*p2->y;
    result->theta   = normalize_angle(p1->theta + p2->theta);

}


void relative_pose(
        const Pose* p1,
        const Pose* p2,
              Pose* relative)
{

    float   dx  = p2->x - p1->x;
    float   dy  = p2->y - p1->y;
    float   c   = cosf( -p1->theta );
    float   s   = sinf( -p1->theta );
    
    relative->x     = c*dx - s*dy;
    relative->y     = s*dx + c*dy;
    relative->theta = normalize_angle(p2->theta - p1->theta);

}


void invert_pose(
        const Pose* p, 
              Pose* inv)
{
    float c = cosf(p->theta), s = sinf(p->theta);

    inv->x     = -( c*p->x + s*p->y );
    inv->y     =  ( s*p->x - c*p->y );
    inv->theta = normalize_angle( -p->theta );
}


// ────────────────────────────────────────────────────────────────────────────
// 
//  ERROR & JACOBIAN FUNCTIONS FOR POSE-POSE FACTOR
// 
// ────────────────────────────────────────────────────────────────────────────

void evaluate_error_pose_pose(
        const Pose* x_i,
        const Pose* x_j,
        const float z_ij[3],
              float error[3])
{
    
    float rel_x, rel_y, rel_theta;
    
    {
        float dx, dy, dtheta;
        float ci, si;
        
        // Compute the relative pose from i to j according to current estimate
        dx      = x_j->x     - x_i->x;
        dy      = x_j->y     - x_i->y;
        dtheta  = x_j->theta - x_i->theta;
        
        ci  = cosf( -x_i->theta );
        si  = sinf( -x_i->theta );
        
        // Transform difference into frame of x_i
        rel_x       =  ci*dx - si*dy;
        rel_y       =  si*dx + ci*dy;
        rel_theta   =  dtheta;
    }

    // Error is difference between predicted and observed
    error[0]    =  rel_x   -   z_ij[0];
    error[1]    =  rel_y   -   z_ij[1];
    error[2]    =  rel_theta - z_ij[2];
    
    // Normalize angle error
    error[2]    =  normalize_angle(error[2]);
}


void compute_jacobian_pose_pose(
        const Pose* x_i,
        const Pose* x_j,
              float A[3][3],
              float B[3][3])
{

    // helper variables
    float si, ci, dx, dy;

    si  = sinf(x_i->theta);
    ci  = cosf(x_i->theta);
    
    dx  = x_j->x - x_i->x;
    dy  = x_j->y - x_i->y;

    /**
     * Jacobian w.r.t. x_i (A)
     * e_x =  cos(θ_i)*dx + sin(θ_i)*dy - z_x
     * e_y = -sin(θ_i)*dx + cos(θ_i)*dy - z_y
     * e_θ =  dtheta - z_theta
     * 
     * row 1: derivatives of e_x
     *      - del e_x / del x_i = -cos(θ_i),
     *      - del e_x / del y_i = -sin(θ_i),
     *      - del e_x / del θ_i = -sin(θ_i)*dx + cos(θ_i)*dy
     * row 2: derivatives of e_y
     *      - del e_y / del x_i =  sin(θ_i),
     *      - del e_y / del y_i = -cos(θ_i),
     *      - del e_y / del θ_i = -cos(θ_i)*dx - sin(θ_i)*dy
     * row 3: derivatives of e_θ
     *      - del e_θ / del x_i =  0,
     *      - del e_θ / del y_i =  0,
     *      - del e_θ / del θ_i = -1
     */

    A[0][0] = -ci;
    A[0][1] = -si;
    A[1][0] =  si;
    A[1][1] = -ci;
    
    A[0][2] = -si*dx + ci*dy;
    A[1][2] = -ci*dx - si*dy;
    
    A[2][0] =  0.f;
    A[2][1] =  0.f;
    A[2][2] = -1.f;
    

    /**
     * Jacobian w.r.t. x_j (B)
     * 
     * e_x =  cos(θ_i)*dx + sin(θ_i)*dy - z_x
     * e_y = -sin(θ_i)*dx + cos(θ_i)*dy - z_y
     * e_θ =  dtheta - z_theta
     * 
     * row 1: derivatives of e_x
     *     - del e_x / del x_j =  cos(θ_i),
     *     - del e_x / del y_j =  sin(θ_i),
     *     - del e_x / del θ_j =  0
     * row 2: derivatives of e_y
     *     - del e_y / del x_j = -sin(θ_i),
     *     - del e_y / del y_j =  cos(θ_i),
     *     - del e_y / del θ_j =  0
     * row 3: derivatives of e_θ
     *     - del e_θ / del x_j =  0,
     *     - del e_θ / del y_j =  0,
     *     - del e_θ / del θ_j =  1
     */
    B[0][0] =  ci;
    B[0][1] =  si;
    B[1][0] = -si;
    B[1][1] =  ci;
    
    B[0][2] =  0.f;
    B[1][2] =  0.f;
    
    B[2][0] =  0.f;
    B[2][1] =  0.f;
    B[2][2] =  1.f;
}



// ────────────────────────────────────────────────────────────────────────────
// 
//  MATRIX HELPERS
// 
// ────────────────────────────────────────────────────────────────────────────



void congruence_3x3(
        float   A[TOTAL],
        float   B[TOTAL],
        float   A_B_AT[TOTAL])
{

    float AB[TOTAL]     = Z_3x3;
    float A_T[TOTAL]    = Z_3x3;

    matmul_3x3(A, B, AB);

    transpose_3x3(A, A_T);

    matmul_3x3(AB, A_T, A_B_AT);

}

void solve_3x3_system(
        const float A[TOTAL], 
        const float b[DIMS], 
              float x[DIMS])
{
    // for our PL-ICP: A == AT_A, b == AT_b

    // checks if system is solvable or unconstrained
    float denominator   = determinant_3x3(A);

    if ( SMALL_ANGLE(denominator) ) {
        // x = (float[3]){0, 0, 0};
        x[0] = 0.f; x[1] = 0.f; x[2] = 0.f;

        return;
    }

    
    // METHOD: unrolled operation for solving 3x3 system of equations. This requires reassigning col `i` of `A` with `b` for each variable `x_i`, then solving for `x_i` using Cramer's rule
    float   A_i[TOTAL];
    uint32_t i = 0;


    // ── remake matrix A_0 from A ────────────────────────────────────────────
    memcpy(A_i, A, sizeof(float)*TOTAL);
    A_i[M_00 + i] = b[0];   A_i[M_10 + i] = b[1];   A_i[M_20 + i] = b[2];

    // substitute `b` into `A_i` to compute `x_i`
    x[i]    = determinant_3x3(A_i) / denominator;


    // ── remake matrix A_1 from A ────────────────────────────────────────────
    i++;
    memcpy(A_i, A, sizeof(float)*TOTAL);
    A_i[M_00 + i] = b[0];   A_i[M_10 + i] = b[1];   A_i[M_20 + i] = b[2];

    // substitute `b` into `A_i` to compute `x_i`
    x[i]    = determinant_3x3(A_i) / denominator;


    // ── remake matrix A_2 from A ────────────────────────────────────────────
    i++;
    memcpy(A_i, A, sizeof(float)*TOTAL);
    A_i[M_00 + i] = b[0];   A_i[M_10 + i] = b[1];   A_i[M_20 + i] = b[2];

    // substitute `b` into `A_i` to compute `x_i`
    x[i]    = determinant_3x3(A_i) / denominator;

}