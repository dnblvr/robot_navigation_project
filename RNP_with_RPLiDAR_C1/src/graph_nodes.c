
#include "../inc/graph_nodes.h"
#include <math.h>
#include <stdio.h>


// Function to add a new observation to the current GraphNode
void add_observation(
    GraphNode  *node,
    Observation new_observation) {
    
    // Allocate memory for the new observation array
    node->observations = realloc(node->observations, sizeof(Observation) * (node->num_observations + 1));
    if (node->observations == NULL) {
        perror("Failed to allocate memory for observations");
        return;
    }

    // Add the new observation to the array
    node->observations[node->num_observations] = new_observation;
    node->num_observations++;
}

// Function to check if a landmark has already been observed
uint8_t is_landmark_close(
    LandmarkMeasurement *existing_landmark,
    LandmarkMeasurement *new_landmark,

    float radius) {
    
        
    // Check if either landmark is NULL
    if (existing_landmark == NULL || new_landmark == NULL) {
        return 0;  // Not close if either landmark is NULL
    }

    // Calculate the Euclidean distance between the two landmarks
    float   dx = existing_landmark->x - new_landmark->x,    
            dy = existing_landmark->y - new_landmark->y;

    float   distance = sqrtf(dx*dx + dy*dy);

    // Check if the distance is within the radius of influence
    return (distance <= radius);
}


// Function to update the GraphNode with new observations
void update_graphnode_with_observations(
    GraphNode   *node,
    Observation *new_observations,

    uint8_t num_new_observations,
    float   radius_of_influence) {

    uint8_t i, j;
    
    // Check if the node or new observations are NULL
    for (i = 0; i < num_new_observations; i++) {
        Observation *new_observation = &new_observations[i];
        int is_close = 0;

        // Check if the new landmark is close to any already-observed landmark
        for (j = 0; j < node->num_observations; j++) {

            uint8_t expression = is_landmark_close(
                node->observations[j].landmark,
                new_observation->landmark,
                radius_of_influence);
            
            if (expression) {
                is_close = 1;
                break;
            }
        }

        // If the landmark is not close to any existing one, add it as a new observation
        if (!is_close) {
            add_observation(node, *new_observation);
        }
    }
}


// Function to predict motion and add a new GraphNode
GraphNode* predict_motion(
    GraphNode   *current_node,
    OdometryEdge control_input) {
    
    // Allocate memory for the new node
    GraphNode *new_node = malloc(sizeof(GraphNode));
    if (new_node == NULL) {
        perror("Failed to allocate memory for new GraphNode");
        return NULL;
    }

    // Predict the new pose based on the current pose and control input
    new_node->pose.x = current_node->pose.x + control_input.dx*cosf(current_node->pose.theta)
                                            - control_input.dy*sinf(current_node->pose.theta);

    new_node->pose.y = current_node->pose.y + control_input.dx*sinf(current_node->pose.theta)
                                            + control_input.dy*cosf(current_node->pose.theta);

    new_node->pose.theta = current_node->pose.theta + control_input.dtheta;

    // Initialize the new node
    new_node->odometry          = control_input;
    new_node->observations      = NULL;
    new_node->num_observations  = 0;
    new_node->next              = NULL;

    // Link the new node to the current node
    current_node->next = new_node;

    return new_node;
}


// Function to compute the cost function for the graph
float cost_function(GraphNode *graph_head) {
    float total_cost = 0.0f;


    // Traverse the graph
    GraphNode *current = graph_head;
    while (current != NULL && current->next != NULL) {

        // Compute the odometry error
        GraphNode *next = current->next;
        float predicted_x = current->pose.x + current->odometry.dx*cosf(current->pose.theta)
                                            - current->odometry.dy*sinf(current->pose.theta);

        float predicted_y = current->pose.y + current->odometry.dx*sinf(current->pose.theta)
                                            + current->odometry.dy*cosf(current->pose.theta);
                                            
        float predicted_theta = current->pose.theta + current->odometry.dtheta;

        float error_x       = next->pose.x - predicted_x;
        float error_y       = next->pose.y - predicted_y;
        float error_theta   = next->pose.theta - predicted_theta;

        total_cost += error_x*error_x + error_y*error_y + error_theta*error_theta;

        // Compute the landmark observation error
        for (uint8_t i = 0; i < current->num_observations; i++) {
            Observation *obs        = &current->observations[i];
            float predicted_range   = sqrtf(    (obs->landmark->x - current->pose.x)*(obs->landmark->x - current->pose.x)
                                             +  (obs->landmark->y - current->pose.y)*(obs->landmark->y - current->pose.y));
            float predicted_bearing =   atan2f(obs->landmark->y - current->pose.y, obs->landmark->x - current->pose.x)
                                      - current->pose.theta;

            float error_range   = obs->range    - predicted_range;
            float error_bearing = obs->bearing  - predicted_bearing;

            total_cost   +=   error_range*error_range
                            + error_bearing*error_bearing;
        }

        current = next;
    }

    return total_cost;
}
