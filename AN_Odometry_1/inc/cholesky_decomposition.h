/**
 * @file cholesky_decomposition.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * 
 */

#ifndef __INC_CHOLESKY_DECOMPOSITION_H__
#define __INC_CHOLESKY_DECOMPOSITION_H__

#include <math.h>
#include <stdio.h>
#include <stdint.h>



/**
 * @brief Perform Cholesky decomposition of a symmetric positive definite matrix A.
 * @details This function decomposes the matrix A into the product of a lower triangular 
 *          matrix L and its transpose.
 *        A = L * L^T, where L is lower triangular.
 *        Returns 0 on success, -1 if the matrix is not positive definite.
 *        Only the lower triangle of L is filled.
 * 
 * @param A     1D array representing the input matrix A (size n x n)
 * @param L     1D array representing the output lower triangular matrix L
 *                      (size n x n)
 * @param n     number of rows and columns in A and L
 * @return int8_t  returns 0 on success, but -1 if the matrix is not positive definite.
 */
int8_t cholesky_decompose(
        float*  A,
        float*  L,
        int     n);


// ----------------------------------------------------------------------------
//
//  HELPER FUNCTIONS
//
// ----------------------------------------------------------------------------

/**
 * @brief this function solves `L @ y = b` for y (forward substitution), where L is lower triangular.
 * 
 * @param L 
 * @param b 
 * @param y 
 * @param n number of rows and columns in A and L
 */
void forward_substitution(
        float*  L,
        float*  b,
        float*  y,
        int     n);


/**
 * @brief this function solves `L^T @ x = y` for x (backward substitution), where L is lower triangular.
 * 
 * @param L Lower triangular matrix from Cholesky decomposition
 * @param y y vector from forward substitution
 * @param x x vector to be solved
 * @param n number of rows and columns in L (and y, x)
 */
void backward_substitution(
    float*  L,
    float*  y,
    float*  x,
    int     n);


#endif /* __INC_CHOLESKY_DECOMPOSITION_H__ */
