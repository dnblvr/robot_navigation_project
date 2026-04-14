
// Uncomment exactly ONE of the following to select which implementation to
// build.  Comment out all of them to get a blank sketch.
//
// #define FLOATING_POINT 1   // ICM-20948 orientation via Madgwick / ECF filter
// #define FIXED_POINT    1   // fixed-point IMU pipeline (WIP)
#define RPLIDAR_IMPL   1      // RPLiDAR C1 Arduino port — test harness
// #define LiDAR_MAPPER   1      // RPLiDAR C1 Arduino port — test harness


#if defined(RPLIDAR_IMPL)
#include "rplidar_impl.h"
#endif

#if defined(LiDAR_MAPPER)
#include "LiDAR_Mapper.h"
#endif

#if defined(FLOATING_POINT)
#include "floating_point_impl.h"
#endif

#if defined(FIXED_POINT)
#include "fixed_point_impl.h"
#endif

#if (defined(RPLIDAR_IMPL) + defined(LiDAR_MAPPER) + defined(FLOATING_POINT) + defined(FIXED_POINT)) > 1
    #error "Multiple implementations selected — please uncomment exactly one #define at the top of main.cpp"
#endif