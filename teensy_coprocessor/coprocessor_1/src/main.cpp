
// Uncomment exactly ONE of the following to select which implementation to
// build.  Comment out all of them to get a blank sketch.
//
// #define FLOATING_POINT 1   // ICM-20948 orientation via Madgwick / ECF filter
// #define FIXED_POINT    1   // fixed-point IMU pipeline (WIP)
#define RPLIDAR_IMPL   1      // RPLiDAR C1 Arduino port — test harness


#ifdef RPLIDAR_IMPL
#include "rplidar_impl.h"
#elif defined(FLOATING_POINT)
#include "floating_point_impl.h"
#else
#include "fixed_point_impl.h"
#endif