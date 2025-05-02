

#include "inc/matrices.h"

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

void print_matrix(const Matrix *m) {

  uint8_t i, j;

  printf("Matrix %dx%d:\n", m->rows, m->cols);

  for (i = 0; i < m->rows; i++) {
    for (j = 0; j < m->cols; j++) {

      // 8-width, 1 decimal places
      printf("%8.1f", ENTRY_OF_(*m, i, j));

    }
    printf("\n");

  }
  printf("\n");

}

// --- Matrix Operations ---

// Matrix transpose (result must be pre-allocated)
uint8_t matrix_transpose(
      const Matrix  *source, 
            Matrix  *destination)
{

  uint8_t i, j;

  if (   source->rows != destination->cols \
      || source->cols != destination->rows  ) {

    return 0;
  }

  for (i = 0; i < source->rows; i++) {
    for (j = 0; j < source->cols; j++) {
      ENTRY_OF_(*destination, j, i) = ENTRY_OF_(*source, i, j);
    }

    ENTRY_OF_(*destination, j, i) = ENTRY_OF_(*source, i, j);

  }

  return 1;
}

// Matrix Multiplication (result must be pre-allocated)
uint8_t matrix_multiply(
      const Matrix  *a,
      const Matrix  *b,
            Matrix  *result)
{

  uint8_t i, j, k;

  if (   (     a->cols != b->rows) \
      || (result->rows != a->rows) \
      || (result->cols != b->cols)  ) {
    
    return 0;
  }

  for (i = 0; i < a->rows; i++) {
    for (j = 0; j < b->cols; j++) {

      float sum = 0.0f;

      // for (k = 0; k < a->cols; k++) {
      //   sum += ENTRY_OF_(*a, i, k) * ENTRY_OF_(*b, k, j);
      // }

      sum =   ENTRY_OF_(*a, i, 0) * ENTRY_OF_(*b, 0, j) \
            + ENTRY_OF_(*a, i, 1) * ENTRY_OF_(*b, 1, j) \
            + ENTRY_OF_(*a, i, 2) * ENTRY_OF_(*b, 2, j);

      ENTRY_OF_(*result, i, j) = sum;
    }
  }
  return 1;
}

// Matrix Inverse (3x3 only)
uint8_t matrix_inverse_3x3(
      const Matrix *m,
            Matrix *inv)
{

  uint8_t i, j;

  // checks dimensions
  if (   (  m->rows != 3) \
      || (  m->cols != 3) \
      || (inv->rows != 3) \
      || (inv->cols != 3)  ) {
    
    return 0;
  }

  // Calculate determinant
  float det = 
        ENTRY_OF_(*m, 0, 0)*(   ENTRY_OF_(*m, 1, 1) * ENTRY_OF_(*m, 2, 2)   \
                              - ENTRY_OF_(*m, 1, 2) * ENTRY_OF_(*m, 2, 1))  \

      - ENTRY_OF_(*m, 0, 1)*(   ENTRY_OF_(*m, 1, 0) * ENTRY_OF_(*m, 2, 2)   \
                              - ENTRY_OF_(*m, 1, 2) * ENTRY_OF_(*m, 2, 0))  \

      + ENTRY_OF_(*m, 0, 2)*(   ENTRY_OF_(*m, 1, 0) * ENTRY_OF_(*m, 2, 1)   \
                              - ENTRY_OF_(*m, 1, 1) * ENTRY_OF_(*m, 2, 0));
  
  // Check for singularity
  if (fabsf(det) < 1e-6) return 0;

  // Compute adjugate (transposed cofactors)
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      float c = 
          ENTRY_OF_(*m, (i+1)%3, (j+1)%3) * ENTRY_OF_(*m, (i+2)%3, (j+2)%3)
        - ENTRY_OF_(*m, (i+1)%3, (j+2)%3) * ENTRY_OF_(*m, (i+2)%3, (j+1)%3);
      
      ENTRY_OF_(*inv, j, i) = c/det;  // Transpose during assignment
    }
  }
  return 1;
}

// Physical transpose (if required)
void physical_transpose(
      const Matrix *source,
            Matrix *destination)
{

  uint8_t i;

  for (i = 0; i < source->rows; i++) {

    // for (j = 0; j < source->cols; j++) {
    //   ENTRY_OF_(*destination, j, i) = ENTRY_OF_(*source, i, j);
    // }

    ENTRY_OF_(*destination, 0, i) = ENTRY_OF_(*source, i, 0);
    ENTRY_OF_(*destination, 1, i) = ENTRY_OF_(*source, i, 1);
    ENTRY_OF_(*destination, 2, i) = ENTRY_OF_(*source, i, 2);
  }
}
