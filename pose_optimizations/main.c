
// #include "src/matrices.c"
#include "inc/matrices.h"
#include "inc/ekf.h"
#include <math.h>  // Required for fabsf()


// --- Example Usage ---
int main() {
  float data_a[9]       = {1,2,3,4,5,6,7,8,9};          // 3x3 matrix
  float data_b[9]       = {1,0,0,0,1,0,0,0,1};          // Identity matrix
  float data_c[12]      = {1,2,3,4,5,6,7,8,9,10,11,12}; // 3x3 matrix
  float data_result[9]  = {0};
  float data_inv[9]     = {0};

  float data_transposed[9]      = {0};
  float data_4x3_transposed[12] = {0};

  Matrix mat_a      = {3,3, data_a};
  Matrix mat_b      = {3,3, data_b};
  Matrix mat_c      = {3,4, data_c};
  Matrix mat_result = {3,3, data_result};
  Matrix mat_inv    = {3,3, data_inv};

  // Same size for square matrices
  Matrix mat_transposed     = {3, 3, data_transposed};
  Matrix mat_4x3_transposed = {4, 3, data_4x3_transposed};


  printf("Original matrices:\n");
  print_matrix(&mat_a);
  print_matrix(&mat_b);
  print_matrix(&mat_c);

  // Matrix multiplication
  if (matrix_multiply(&mat_a, &mat_a, &mat_result)) {
    printf("Multiplication result:\n");
    print_matrix(&mat_result);
  }

  // Matrix inverse
  if (matrix_inverse_3x3(&mat_b, &mat_inv)) {
    printf("Inverse of identity matrix:\n");
    print_matrix(&mat_inv);
  }

  // physical transpose demonstration
  if (matrix_transpose(&mat_a, &mat_transposed)) {
    printf("Physically transposed matrix:\n");
    print_matrix(&mat_transposed);
  }

  // physical transpose demonstration
  if (matrix_transpose(&mat_c, &mat_4x3_transposed)) {
    printf("Physically transposed matrix:\n");
    print_matrix(&mat_4x3_transposed);
  }

  return 0;
}