

#ifndef __MATRICES_H__
#define __MATRICES_H__

#include <stdint.h>
#include <math.h>
#include <stdio.h>



// --- Matrix Struct & Macros ---

// #define MATRIX_STRUCT
#ifdef MATRIX_STRUCT

#define ENTRY_OF_(m, i,j)  ( (m).data[(i) * (m).cols  +  (j)] )

typedef struct {
  uint8_t rows,
          cols;
  float  *data;  // Use float to leverage MSP432's FPU
} Matrix;


void print_matrix(const Matrix *m);

// --- Matrix Operations ---

// Matrix transpose (result must be pre-allocated)
uint8_t matrix_transpose(
      const Matrix  *source, 
            Matrix  *destination);

// Matrix Multiplication (result must be pre-allocated)
uint8_t matrix_multiply(
      const Matrix  *a,
      const Matrix  *b,
            Matrix  *result);

// Matrix Inverse (3x3 only)
uint8_t matrix_inverse_3x3(
      const Matrix *m,
            Matrix *inv);

// Physical transpose (if required)
void physical_transpose(
      const Matrix *source,
            Matrix *destination);


#else

// --- Matrix Operations ---

/**
 * @brief 
 * 
 * @param rows 
 * @param cols 
 * @param matrix 
 */
void print_matrix(uint8_t rows, uint8_t cols, float matrix[rows][cols]);


/**
 * @brief               Matrix transpose (result must be pre-allocated)
 * 
 * @param rows 
 * @param cols 
 * @param source 
 * @param destination 
 * @return uint8_t 
 */
uint8_t matrix_transpose(uint8_t rows, uint8_t cols, float source[rows][cols], float destination[cols][rows]);


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
 * @brief               Matrix inverse (3x3 only)
 * 
 * @param m 
 * @param inv 
 * @return uint8_t 
 */
uint8_t matrix_inverse_3x3(float m[3][3], float inv[3][3]);

/**
 * @brief               Physical transpose (if required)
 * 
 * @param rows 
 * @param cols 
 * @param source 
 * @param destination 
 */
void physical_transpose(uint8_t rows, uint8_t cols, float source[rows][cols], float destination[cols][rows]);

#endif


#endif  // __MATRICES_H__