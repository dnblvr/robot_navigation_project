

#ifndef _MATRICES_H_
#define _MATRICES_H_

#include <stdint.h>
#include <math.h>
#include <stdio.h>

// --- Matrix Struct & Macros ---

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


#endif  // _MATRICES_H_