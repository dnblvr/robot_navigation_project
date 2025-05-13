#ifndef __GRAPH_NODES_H__
#define __GRAPH_NODES_H__

#include <stdint.h>
#include <stdlib.h>


// --- Pose State (x_i) ---
typedef struct {
    float   x,
            y,
            theta;  // radians
} PoseState;

// --- Control Input / Odometry Edge (u_i) ---
typedef struct {
    float   dx,
            dy,
            dtheta; // Change in orientation
} OdometryEdge;

// --- Landmark Measurement (m_i) ---
typedef struct {
    uint8_t id;     // Unique ID of the landmark
    float   x;      // x-coordinate of the landmark (in the global frame)
    float   y;      // y-coordinate of the landmark (in the global frame)
} LandmarkMeasurement;


// --- Observation (z_i) ---
typedef struct {
    LandmarkMeasurement *landmark;  // Pointer to the observed landmark
    float   range,                  // Distance to the landmark
            bearing;                // Angle to the landmark (relative to the robot's orientation)
} Observation;


// --- Graph Node ---
typedef struct GraphNode {

    PoseState       pose;               // Robot pose (x_i)
    OdometryEdge    odometry;           // Odometry edge (u_i)
    Observation    *observations;       // Array of observations (z_i)
    uint8_t         num_observations;   // Number of observations

    struct GraphNode  *next;            // Pointer to the next graph node (linked list structure)

} GraphNode;



// update functions -------------------------------------



/**
 * @brief   Function to add a new observation to the current GraphNode
 * 
 * @param node 
 * @param new_observation 
 */
void add_observation(
    GraphNode  *node,
    Observation new_observation);


/**
 * @brief   Function to check if a landmark is close to an already-observed one
 * 
 * @param existing_landmark 
 * @param new_landmark 
 * @param radius 
 * @return uint8_t 
 */
uint8_t is_landmark_close(
    LandmarkMeasurement *existing_landmark,
    LandmarkMeasurement *new_landmark,

    float radius);



/**
 * @brief   Function to update the GraphNode with new observations
 * 
 * @param node 
 * @param new_observations 
 * @param num_new_observations 
 * @param radius_of_influence 
 */
void update_graphnode_with_observations(
    GraphNode   *node,
    Observation *new_observations,

    uint8_t num_new_observations,
    float   radius_of_influence);


GraphNode* predict_motion(GraphNode *current_node, OdometryEdge control_input);
float cost_function(GraphNode *graph_head);


#endif // __GRAPH_NODES_H__