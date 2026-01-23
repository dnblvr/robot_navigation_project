/**
 * @file    Dynamic_Window_Approach.h
 * @brief   Implementation of the Dynamic Window Approach (DWA) for local planning
 */

#ifndef __INC_DYNAMIC_WINDOW_APPROACH__
#define __INC_DYNAMIC_WINDOW_APPROACH__


#include "data_structures.h"
#include <float.h>
#include <math.h>

/**
 * @brief  
 */
typedef struct {
    float   min_speed,
            max_speed;


    float 	max_omega;
    float 	max_accel;
    float 	max_omega_accel;

    float 	v_resolution;
    float 	omega_resolution;
    float 	dt;
    float 	predict_time;
    float 	robot_radius;

    float 	to_goal_cost_gain;
    float 	speed_cost_gain;
    float 	obstacle_cost_gain;

} DWA_Config;


/**
 * @brief  
 */
typedef struct  {

    float x;
    float y;
    float yaw;
    float v;          
    float omega;

} RobotState;



/**
 * @brief Maximum trajectory storage length.
 */
#define MAX_TRAJECTORY_LEN 10


/**
 * @brief a string of points representing a curved trajectory
 * 
 * @note This must be generated locally so that multiple trajectories can be stored and discarded as needed
 * 
 */
typedef struct {

    Point2D points[MAX_TRAJECTORY_LEN];
    uint32_t num_points;

} Predicted_Path;


#endif /* __INC_DYNAMIC_WINDOW_APPROACH__ */
