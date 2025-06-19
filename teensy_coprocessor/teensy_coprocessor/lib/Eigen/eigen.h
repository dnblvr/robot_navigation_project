
/**
 * @file eigen.h
 * @brief Custom Eigen library header for Teensy microcontroller
 * @author Gian Fajardo
 * @date 18-06-2025
 *
 * This port of the Eigen library, through this header file, includes the
 * necessary Eigen library headers and provides customizations for the Teensy
 * microcontroller environment.
 * 
 * @note This file was informed by Brian R. Taylor (@brtaylor) at the PJRC forums.
 * @see https://github.com/bolderflight/eigen/blob/main/src/eigen.h
 * @see https://forum.pjrc.com/index.php?threads/how-to-install-use-eigen-template-library.41929/post-133230
 * 
 */

#ifndef EIGEN_H
#define EIGEN_H

/*
* Guarantee that the Eigen code is licensed under the MPL2 and possibly more permissive licenses (like BSD)
*/
#define EIGEN_MPL2_ONLY

/* Disable asserts to speed up runtime */
#define EIGEN_NO_DEBUG 1

/* Include core header */
#include <Eigen/Core>

#endif // EIGEN_H