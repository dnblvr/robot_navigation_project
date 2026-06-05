/**
 * @file cholesky_decomposition.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * 
 */

#ifndef __CHOLESKY_DECOMPOSITION_H__
#define __CHOLESKY_DECOMPOSITION_H__

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Perform Cholesky decomposition of a symmetric positive definite
 *  matrix A.
 * 
 * @details This function decomposes the matrix A into the product of a lower
 *  triangular matrix L and its transpose.
 * 
 *      - `A = L * L^T`, where L is lower triangular.
 *      - Returns `0` on success, `-1` if the matrix is not positive definite.
 *      - Only the lower triangle of L is filled.
 * 
 * @param[in] stride set location of the largest possible element in each row of the statically-allocated square matrix.
 * @param[in] n number of `rows` and `cols` in matrices `A` and `L`
 * @param[in] A input square matrix
 * 
 * @param[out] L output lower triangular matrix
 * @return `int8_t`
 * @retval `0` on success
 * @retval `-1` if the matrix is not positive definite
 */
int8_t Cholesky_Decompose(
        int     stride,
        int     n,
        float*  A,
        float*  L);


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief this function solves solves `y` in equation `L @ y = b` f via forward
 *  substitution, where `L` is lower triangular.
 * 
 * @details `Cholesky_Decompose()` must be called first to compute `L` before
 * 
 * @param[in] stride set location of the largest possible element in each row of the statically-allocated square matrix
 * @param[in] n number of rows and columns in matrices `L`, `b`, and `y`
 * @param[in] L Lower triangular matrix from Cholesky decomposition
 * @param[in] b Right-hand side vector
 * 
 * @param[out] y Solution vector
 */
void Cholesky_Forward_Substitution(
        int     stride,
        int     n,
        float*  L,
        float*  b,
        float*  y);


/**
 * @brief This function solves `x` in equation `L^T @ x = y` via backward
 *  substitution, where `L` is lower triangular.
 * 
 * @param[in] stride set location of the largest possible element in each row of the statically-allocated square matrix
 * @param[in] n number of rows and columns in matrices `L`, `y`, and `x`
 * @param[in] L Lower triangular matrix from Cholesky decomposition
 * @param[in] y vector from forward substitution
 * 
 * @param[out] x Solution vector
 * 
 * @note `Cholesky_Forward_Substitution()` must be called first to compute `y`
 *  before calling this function to compute `x`.
 * 
 * @see `Cholesky_Forward_Substitution()` for forward substitution portion
 */
void Cholesky_Backward_Substitution(
        int     stride,
        int     n,
        float*  L,
        float*  y,
        float*  x);

#ifdef __cplusplus
}
#endif

#endif /* __CHOLESKY_DECOMPOSITION_H__ */
