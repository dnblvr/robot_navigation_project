

#ifndef __INC_MATRICES_H__
#define __INC_MATRICES_H__

#include <stdint.h>
#include <math.h>
#include <stdio.h>




/**
 * @brief 
 * 
 * @param rows 
 * @param cols 
 * @param matrix 
 */
void print_matrix(
        uint8_t rows,uint8_t cols,
        float   matrix[rows][cols]);


/**
 * @brief               Matrix transpose (result must be pre-allocated)
 * 
 * @param rows 
 * @param cols 
 * @param source 
 * @param destination 
 * @return uint8_t 
 */
uint8_t matrix_transpose(
        uint8_t rows, uint8_t cols,

        float        source[rows][cols],
        float   destination[cols][rows]);


/**
 * @brief               Matrix multiplication (result must be pre-allocated)
 * 
 * @param a_rows 
 * @param a_cols 
 * @param a 

 * @param b_rows 
 * @param b_cols 
 * @param b 

 * @param result 
 * @return uint8_t 
 */
uint8_t matrix_multiply(
        uint8_t a_rows, uint8_t a_cols, float a[a_rows][a_cols],
        uint8_t b_rows, uint8_t b_cols, float b[b_rows][b_cols],
        float result[a_rows][b_cols]);


/**
 * @brief Multiply two 3x3 homogeneous transformation matrices
 * 
 * @param A         First transformation matrix
 * @param B         Second transformation matrix
 * @param result    Output matrix = A × B
 */
void matrix_multiply_3x3(
        float A[3][3],
        float B[3][3],
        float result[3][3]);


/**
 * @brief               Matrix inverse (3x3 only)
 * 
 * @param m 
 * @param inv 
 * @return uint8_t 
 */
uint8_t matrix_inverse_3x3(
        float m[3][3],
        float inv[3][3]);

/**
 * @brief               Physical transpose (if required)
 * 
 * @param rows 
 * @param cols 
 * @param source 
 * @param destination 
 */
void physical_transpose(
        uint8_t rows,
        uint8_t cols,

        float source[rows][cols],
        float destination[cols][rows]);


#endif  /* __INC_MATRICES_H__ */
