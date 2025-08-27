
#ifndef MATRIX_EXAMPLES_H
#define MATRIX_EXAMPLES_H

#include <Arduino.h>
// #include <ArduinoEigen.h>
// #include <ArduinoEigenSparse.h>
#include <ArduinoEigenDense.h>




/**
 * @brief prints the matrix to the serial console
 * @note this function is a template function, so it can be used with any Eigen
 *        matrix type 
 * 
 * @todo to better format the output, we could use sprintf() instead to format, then send to Serial.print()
 * 
 * @tparam Derived    the matrix type, of which MatrixBase is the base class
 * @param[in] matrix  matrix of any Eigen matrix type (float, double, etc.)
 */
template<typename Derived>
void print_matrix(const Eigen::MatrixBase<Derived>& matrix);




/**
 * @brief function which stores the examples of matrix instantiation,
 *        operations and printing
 */
void matrix_examples();

#endif // MATRIX_EXAMPLES_H