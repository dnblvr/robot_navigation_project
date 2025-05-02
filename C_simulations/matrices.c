

#include "matrices.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>


// #define MATRIX_STRUCT
#ifdef MATRIX_STRUCT

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
#else

// --- Matrix Operations ---



// Matrix multiplication (result must be pre-allocated)
uint8_t matrix_multiply(
    uint8_t a_rows, uint8_t a_cols, float a[a_rows][a_cols],
    uint8_t b_rows, uint8_t b_cols, float b[b_rows][b_cols],
    float result[a_rows][b_cols])
{
if (a_cols != b_rows) {
    return 0;
}

for (uint8_t i = 0; i < a_rows; i++) {
    for (uint8_t j = 0; j < b_cols; j++) {
        float sum = 0.0f;
        for (uint8_t k = 0; k < a_cols; k++) {
            sum += a[i][k] * b[k][j];
        }
        result[i][j] = sum;
    }
}
return 1;
}

void print_matrix(uint8_t rows, uint8_t cols, float matrix[rows][cols]) {
    printf("Matrix %dx%d:\n", rows, cols);
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < cols; j++) {
            // 8-width, 1 decimal place
            printf("%8.1f", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

// Matrix inverse (3x3 only)
uint8_t matrix_mult_3x3(
    float a[3][3], float b[3][3],

    float result[3][3])
{
    uint8_t i;
    float sum = 0.0f;

    for (i = 0; i < 3; i++) {

        sum =  a[i][0] * b[0][0]
             + a[i][1] * b[1][0]
             + a[i][2] * b[2][0];
        result[i][0] = sum;

        sum =  a[i][0] * b[0][1]
             + a[i][1] * b[1][1]
             + a[i][2] * b[2][1];
        result[i][1] = sum;

        sum =  a[i][0] * b[0][2]
             + a[i][1] * b[1][2]
             + a[i][2] * b[2][2];
        result[i][2] = sum;

    }
    return 1;
}

// Matrix inverse (3x3 only)
uint8_t matrix_inverse_3x3(float m[3][3], float inv[3][3]) {
    // Calculate determinant
    float det =   m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    // Check for singularity
    if (fabsf(det) < 1e-6) return 0;

    // Compute adjugate (transposed cofactors)
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            float c =   m[(i + 1) % 3][(j + 1) % 3] * m[(i + 2) % 3][(j + 2) % 3]
                      - m[(i + 1) % 3][(j + 2) % 3] * m[(i + 2) % 3][(j + 1) % 3];
            inv[j][i] = c / det;  // Transpose during assignment
        }
    }
    return 1;
}

// Matrix transpose (result must be pre-allocated)
uint8_t matrix_transpose(uint8_t rows, uint8_t cols, float source[rows][cols], float destination[cols][rows]) {
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < cols; j++) {
            destination[j][i] = source[i][j];
        }
    }
    return 1;
}


void print_matrix(uint8_t rows, uint8_t cols, float matrix[rows][cols]) {
    printf("Matrix %dx%d:\n", rows, cols);
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < cols; j++) {
            // 8-width, 1 decimal place
            printf("%1.2f\t", matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

// Matrix inverse (3x3 only)
uint8_t matrix_mult_3x3(
    float a[3][3], float b[3][3],

    float result[3][3])
{
    uint8_t i;
    float sum = 0.0f;

    for (i = 0; i < 3; i++) {

        sum =  a[i][0] * b[0][0]
             + a[i][1] * b[1][0]
             + a[i][2] * b[2][0];
        result[i][0] = sum;

        sum =  a[i][0] * b[0][1]
             + a[i][1] * b[1][1]
             + a[i][2] * b[2][1];
        result[i][1] = sum;

        sum =  a[i][0] * b[0][2]
             + a[i][1] * b[1][2]
             + a[i][2] * b[2][2];
        result[i][2] = sum;

    }
    return 1;
}

// Matrix inverse (3x3 only)
uint8_t matrix_inverse_3x3(float m[3][3], float inv[3][3]) {
    // Calculate determinant
    float det =   m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    // Check for singularity
    if (fabsf(det) < 1e-6) return 0;

    // Compute adjugate (transposed cofactors)
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            float c =   m[(i + 1) % 3][(j + 1) % 3] * m[(i + 2) % 3][(j + 2) % 3]
                      - m[(i + 1) % 3][(j + 2) % 3] * m[(i + 2) % 3][(j + 1) % 3];
            inv[j][i] = c / det;  // Transpose during assignment
        }
    }
    return 1;
}

// Matrix transpose (result must be pre-allocated)
uint8_t matrix_transpose(uint8_t rows, uint8_t cols, float source[rows][cols], float destination[cols][rows]) {
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < cols; j++) {
            destination[j][i] = source[i][j];
        }
    }
    return 1;
}

// Matrix multiplication (result must be pre-allocated)
uint8_t matrix_multiply(
    uint8_t a_rows, uint8_t a_cols, float a[a_rows][a_cols],
    uint8_t b_rows, uint8_t b_cols, float b[b_rows][b_cols],
    float result[a_rows][b_cols])
{
    if (a_cols != b_rows) {
        return 0;
    }

    for (uint8_t i = 0; i < a_rows; i++) {
        for (uint8_t j = 0; j < b_cols; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < a_cols; k++) {
                sum += a[i][k] * b[k][j];
            }
            result[i][j] = sum;
        }
    }
    return 1;
}


#endif