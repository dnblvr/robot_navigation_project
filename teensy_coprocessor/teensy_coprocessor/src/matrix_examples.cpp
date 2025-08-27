
#include "matrix_examples.h"


template<typename Derived>
void print_matrix(
    const Eigen::MatrixBase<Derived>& matrix)
{
  
  Serial.println();

  for (int i = 0; i < matrix.rows(); ++i) {

    for (int j = 0; j < matrix.cols(); ++j) {

      // Format numbers to 3 decimal places for better readability
      char buffer[20];
      sprintf(buffer, " %8.2f", (float)matrix(i, j));
      Serial.print(buffer);

    }

    Serial.println();

  }

}



void matrix_examples() {

  // generating our matrices of floats ---------------------------------------
  Eigen::Matrix <float, 3, 3> matrix_A;
  matrix_A.setZero();

  Eigen::Matrix3f matrix_B;
  matrix_B.setZero();

  Eigen::MatrixXf matrix_C(5, 5);

  // float i = 0;
  // for (int row = 0; row < matrix_A.rows(); ++row) {
  //   for (int col = 0; col < matrix_A.cols(); ++col) {
  //     matrix_A(row, col) = i++;
  //   }
  // }
  matrix_C <<  1.f,  2.f,  3.f,  4.f,  5.f,
               6.f,  7.f,  8.f,  9.f, 10.f,
              11.f, 12.f, 13.f, 14.f, 15.f,
              16.f, 17.f, 18.f, 19.f, 20.f,
              21.f, 22.f, 23.f, 24.f, 25.f;

  Eigen::MatrixXf matrix_D = matrix_C.block(2,2, 2,2);

  // create a diagonal matrix given a vector
  int i = 1;
  // float i = 1.f;
  Eigen::VectorXf vector(5);

  // vector << 1, 2, 3, 4, 5;
  vector                          << i++, i++, i++, i++, i++;
  Eigen::MatrixXf diagonal_matrix = vector.asDiagonal();

  // matrix operations
  Eigen::MatrixXf matrix_sum    = matrix_C + diagonal_matrix;
  Eigen::MatrixXf scalar_matrix = matrix_sum * 5.1f;

  // double matrices
  Eigen::MatrixXd double_matrix(3, 3);
  double_matrix << 1.1, 2.2, 3.3,
                   4.4, 5.5, 6.6,
                   7.7, 8.8, 9.9;


  // printing our matrices ---------------------------------------------------
  
  // matrix instantiation prints ---------------------------

  Serial.print("\nMatrix A:");
  print_matrix(matrix_A);

  Serial.print("\nMatrix B:");
  print_matrix(matrix_B);

  Serial.print("\nMatrix C:");
  print_matrix(matrix_C);

  Serial.print("\nMatrix D:");
  print_matrix(matrix_D);
  
  Serial.print("\nDiagonal Matrix:");
  print_matrix(diagonal_matrix);

  // matrix operation prints ------------------------------

  Serial.print("\nAddition Matrix:\n");
  print_matrix(matrix_sum);

  Serial.print("\nScaled Matrix:");
  print_matrix(scalar_matrix);

  Serial.print("\nDouble Matrix:\n");
  print_matrix(double_matrix);
}