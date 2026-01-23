/**
 * @file    Dynamic_Window_Approach.c
 * @brief   Implementation of the Dynamic Window Approach (DWA) for local
 */

#include "inc/Dynamic_Window_Approach.h"

/**
 * @brief 
 * 
 * @param cfg 
 */
void Config_DWA(DWA_Config* cfg) {

    cfg->max_speed          =  1.0;
    cfg->min_speed          = -0.5;
    cfg->max_omega          =  40.0 * DEG_TO_RAD;
    cfg->max_accel          =  0.2;
    cfg->max_omega_accel    =  40.0 * DEG_TO_RAD;

    // cfg->v_resolution       =  0.01;
    cfg->omega_resolution   =  0.1 * DEG_TO_RAD;
    cfg->dt                 =  0.5;
    cfg->predict_time       =  3.0;
    cfg->robot_radius       =  0.5;

    cfg->to_goal_cost_gain  =  0.1;
    cfg->speed_cost_gain    =  1.0;
    cfg->obstacle_cost_gain =  0.5;

}


void Config_Robot_State(RobotState* state) {

    state->x     = 0;
    state->y     = 0;
    state->yaw   = 0;
    state->v     = 0;
    state->omega = 0;
}


void DWA_Control(
        RobotState*         state,
        const DWA_Config*   cfg,
        const Point2D       goal,
        const PointCloud*   obstacles,

        float*              best_u,
        Predicted_Path*     best_trajectory)
{
    // counter
    uint32_t i, j, num_trajectories = 10;

    //
    float v_min     = state->v     - cfg->dt * cfg->max_accel;
    float v_max     = state->v     + cfg->dt * cfg->max_accel;
    float omega_min = state->omega - cfg->dt * cfg->max_omega_accel;
    float omega_max = state->omega + cfg->dt * cfg->max_omega_accel;

    float v_min     = max( cfg->min_speed, v_min);
    float v_max     = min( cfg->max_speed, v_max);
    float omega_min = max(-cfg->max_omega, omega_min);
    float omega_max = min( cfg->max_omega, omega_max);


    // initialize: the lowest cost as high and the power to both motors as 0.f
    float best_cost   = FLT_MAX;
    float best_u[2]   = {0.0, 0.0};

    // Predicted_Path best_trajectory = np.array([[state->x, state->y]]);
    Predicted_Path best_trajectory;


    // todo need to make the arange function & for-loop work in C lol

    float v, omega;
    // for v in np.arange(dw["v_min"], dw["v_max"], cfg->v_resolution) {
    for (i = 0; i < num_trajectories; i++)
    {
        v = v_min + i * (v_max - v_min) / (num_trajectories - 1);

        // for omega in np.arange(dw["omega_min"], dw["omega_max"], cfg->omega_resolution) {
        for (j = 0; j < num_trajectories; j++) {

            float omega = omega_min + j * (omega_max - omega_min) / (num_trajectories - 1);

            
            trajectory = predict_trajectory(state, v, omega, cfg);

            to_goal_cost    = \
                cfg->to_goal_cost_gain * calc_to_goal_cost(&trajectory,
                                                           goal);
            
            speed_cost      = \
                cfg->speed_cost_gain * (cfg->max_speed - trajectory[-1, 3]);
            
            obstacle_cost   = \
                cfg->obstacle_cost_gain * calc_obstacle_cost(&trajectory,
                                                             obstacles,
                                                             cfg);
            
            final_cost = to_goal_cost + speed_cost + obstacle_cost;

            if (final_cost != FLT_MAX) {

                // pygame_traj_points  = \
                //     [to_pygame(pos) for pos in trajectory[:, 0:2]]

                // pygame.draw.lines(screen,
                //                   LIGHT_GRAY,
                //                   False,
                //                   pygame_traj_points,
                //                   1)
            }

            if (final_cost < best_cost) {

                best_cost = final_cost;
                best_u[0], best_u[1] = (float)v, (float)omega;
                best_trajectory = trajectory;
            }
        }
    }
    
    // return best_u, best_trajectory

}


def predict_trajectory(
        state, 
        v, 
        omega, 
        config) -> ndarray:
{

    // todo fix the function signature to C syntax

    // todo fix this state representation to C syntax
    current_state   = np.array([state->x,
                                state->y,
                                state->yaw,
                                state->v,
                                state->omega], dtype=np.float16)
    
    // trajectory  = np.array(current_state, dtype=np.float16)
    Predicted_Path trajectory;
    trajectory.points[0].x = current_state[0];
    float time = 0
    
    while (time <= cfg->predict_time) {
        

        current_state[2]   += omega * cfg->dt  // yaw
        current_state[0]   += v * math.cos(current_state[2]) * cfg->dt  // x
        current_state[1]   += v * math.sin(current_state[2]) * cfg->dt  // y
        current_state[3]    = v  // v
        current_state[4]    = omega  // omega
        
        trajectory = np.vstack((trajectory, current_state))
        time += cfg->dt
    }
    
    return trajectory;
}


/**
 * @brief 
 * 
 */
float calc_obstacle_cost(
        Predicted_Path* trajectory,
        PointCloud* obstacles,
        DWA_Config* cfg
    ) -> float:

    float min_dist = FLT_MAX;

    // todo fix the loop indexing on both trajectory and obstacles
    // todo figure out how to pass obstacles and also how to simplify the process of accessing their data. the problem is that obstacles is a PointCloud struct and would be expensive to process all 100 points per scan
    for i in range(len(trajectory)) {

        for o_x, o_y, radius in obstacles {
            
            float dx, dy;

            // compute the distance from the robot position to the obstacle
            // no need to fix, as this is valid C syntax
            dx, dy = trajectory[i, 0] - o_x, trajectory[i, 1] - o_y;

            dist = sqrtf(dx*dy + dy*dy);
            
            // early-return if a collision is detected
            if (dist <= cfg->robot_radius + radius) {
                return FLT_MAX;
            }
            
            min_dist = min(min_dist, dist);
        }
    }


    return 1.0 / min_dist;


/**
 * @brief 
 * 
 * @param trajectory 
 * @param goal 
 * @return float 
 */
float calc_to_goal_cost(
        Predicted_Path* trajectory, 
        Point2D* goal)
{
    // todo : fix the trajectory indexing
    float dx            = goal[0] - trajectory[-1, 0];
    float dy            = goal[1] - trajectory[-1, 1];
    float error_angle   = math.atan2(dy, dx);
    float cost_angle    = error_angle - trajectory[-1, 2] ;
    
    float cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)));

    return cost;
}


/**
 * @brief function that updates the robot state based on the control inputs
 * 
 * @param state 
 * @param u         velocity of the differential drive robot [v, omega]
 * @param dt 
 * @param new_state 
 */
void motion(RobotState* state, float u[2], float dt) {
    state->yaw     += u[1] * dt
    state->x       += u[0] * math.cos(state->yaw) * dt
    state->y       += u[0] * math.sin(state->yaw) * dt
    state->v        = u[0]
    state->omega    = u[1]
}

