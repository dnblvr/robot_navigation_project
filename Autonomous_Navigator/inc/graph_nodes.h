#ifndef __GRAPH_NODES_H__
#define __GRAPH_NODES_H__


#include "matrices.h"

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
    int     landmark_index;         // Index of the landmark in the global list
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
 * @brief Check if two landmarks are within a Mahalanobis distance threshold.
 * 
 * @param existing_landmark   Pointer to the first landmark (with x, y)
 * @param new_landmark        Pointer to the second landmark (with x, y)
 * @param covariance          2x2 covariance matrix (row-major: [ [a, b], [c, d] ])
 * @param threshold           Mahalanobis distance threshold
 * @return uint8_t            1 if within threshold, 0 otherwise
 */
uint8_t within_mahalanobis_distance(
    LandmarkMeasurement *existing_landmark,
    LandmarkMeasurement *new_landmark,
    
    float   covariance[2][2],
    float   threshold);



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


GraphNode* predict_motion(
    GraphNode *current_node,
    OdometryEdge control_input);



#endif // __GRAPH_NODES_H__
