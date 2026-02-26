

#ifndef __INC_INEKF_SE2_H__
#define __INC_INEKF_SE2_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define DIMS 3
#define TOTAL 9

#ifdef __cplusplus
extern "C" {
#endif

struct {

} InEKF_SE2;

/**
 * @brief 
 * 
 * @param tau 
 * @param out 
 */
void wedge_se2(
        float tau[DIMS],
        float tau_wedge[TOTAL]);


/**
 * @brief 
 * 
 * @param tau_wedge 
 * @param tau 
 */
void vee_se2(
        float tau_wedge[TOTAL],
        float tau[DIMS]);


/**
 * @brief 
 * 
 * @param tau 
 * @param exp_tau 
 */
void exp_se2(
        float tau[DIMS],
        float exp_tau[TOTAL]);


/**
 * @brief 
 * 
 * @param exp_tau 
 * @param tau 
 */
void log_se2(
        float exp_tau[TOTAL],
        float tau[DIMS]);


/**
 * @brief 
 * 
 * @param exp_tau 
 * @param adj_exp_tau 
 */
void adjoint_se2(
        float exp_tau[TOTAL],
        float adj_exp_tau[TOTAL]);

// ----------------------------------------------------------------------------
//
//  INVARIANT EKF FUNCTIONS
//
// ----------------------------------------------------------------------------



void inEKF_SE2_predict(
        float state[TOTAL],
        float control[DIMS],
        float dt,
        float predicted_state[TOTAL]);

#ifdef __cplusplus
}
#endif

#endif /* __INC_INEKF_SE2_H__ */
