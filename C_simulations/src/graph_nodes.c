
#include "graph_nodes.h"
#include <math.h>
#include <stdio.h>


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

/**
 * @brief Check if two landmarks are within a Mahalanobis distance threshold.
 */
uint8_t within_mahalanobis_distance(
        LandmarkMeasurement *existing_landmark,
        LandmarkMeasurement *new_landmark,
        
        float   covariance[2][2],
        float   threshold)
{
    float   dx, dy, diff[2], det, inv_cov[2][2],
            temp0, temp1, mahalanobis_sq;

    if (existing_landmark == NULL || new_landmark == NULL) {
        return 0;
    }

    // Compute difference vector
    dx = existing_landmark->x - new_landmark->x;
    dy = existing_landmark->y - new_landmark->y;

    diff[0] = dx;   diff[1] = dy;

    // Invert 2x2 covariance matrix
    det = covariance[0][0]*covariance[1][1] - covariance[0][1]*covariance[1][0];
    if (fabsf(det) < 1e-12f) return 0; // Avoid division by zero

    // Compute inverse covariance matrix
    inv_cov[0][0] =  covariance[1][1] / det;
    inv_cov[0][1] = -covariance[0][1] / det;
    inv_cov[1][0] = -covariance[1][0] / det;
    inv_cov[1][1] =  covariance[0][0] / det;

    // Compute Mahalanobis distance squared
    temp0   = inv_cov[0][0]*diff[0] + inv_cov[0][1]*diff[1];
    temp1   = inv_cov[1][0]*diff[0] + inv_cov[1][1]*diff[1];
    mahalanobis_sq  = diff[0]*temp0 + diff[1]*temp1;

    // Compare to threshold squared
    return (mahalanobis_sq <= threshold*threshold) ? 1 : 0;
}


void update_graphnode_with_observations(
        GraphNode   *node,
        Observation *new_observations,

        uint8_t num_new_observations,
        float   threshold)
{

    uint8_t i, j;

    float covariance[2][2] = { {1.0f, 0.0f}, {0.0f, 1.0f} };
    
    // Check if the node or new observations are NULL
    for (i = 0; i < num_new_observations; i++) {
        Observation *new_observation = &new_observations[i];
        int is_close = 0;

        // Check if the new landmark is close to any already-observed landmark
        for (j = 0; j < node->num_observations; j++) {

            uint8_t expression = within_mahalanobis_distance(
                node->observations[j].landmark,
                new_observation->landmark,
                covariance,
                threshold);
            
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
