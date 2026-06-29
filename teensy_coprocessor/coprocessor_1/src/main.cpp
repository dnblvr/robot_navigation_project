/**
 * @file main.cpp
 * @author Gian Fajardo (gianfajardo.prim@gmail.com)
 * @brief 
 * @version 0.1
 *
 *  This file serves as the entry point for the Teensy coprocessor firmware.
 *  It conditionally includes one of several implementations based on which
 *  #define is active at the top of this file. Only one implementation should
 *  be active at a time.
 *
 *  @note 
 */


#if defined(FL_POINT)
#include "floating_point_impl.h"
#endif

#if defined(FX_POINT)
#include "fixed_point_impl.h"
#endif

#if defined(BEEP_TEST)
#include "beep_test.h"
#endif

#if defined(IMU_MAG_CAL)
#include "imu_mag_cal.h"
#endif

#if defined(RPLIDAR_IMPL)
#include "rplidar_impl.h"
#endif

#if defined(LiDAR_MAPPER)
#include "LiDAR_Mapper.h"
#endif

#if defined(SCAN_REPLAY)
#include "scan_replay.h"
#endif

#if ( 0 \
      + defined(FL_POINT)       \
      + defined(FX_POINT)       \
      + defined(BEEP_TEST)      \
      + defined(IMU_MAG_CAL)    \
      + defined(RPLIDAR_IMPL)   \
      + defined(LiDAR_MAPPER)   \
      + defined(SCAN_REPLAY)    \
    ) > 1
      
    #error "Multiple implementations selected — please uncomment exactly one #define at the top of main.cpp"
#endif