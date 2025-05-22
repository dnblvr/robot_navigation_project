
#include "graphslam.h"

#include "graph_nodes.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// Define these as needed for your problem size
#define MAX_POSES 100
#define STATE_SIZE (MAX_POSES * 3)

// --- High-level Gauss-Newton SLAM functions ---

// 1. Build the state vector from the linked list
void build_state_vector(GraphNode *head, float *state_vector, int *num_poses);

// 2. Linearize all constraints (compute errors and Jacobians)
void linearize_constraints(GraphNode *head, float *state_vector, float *b, float H[STATE_SIZE][STATE_SIZE], int num_poses);

// 3. Solve the linear system H dx = -b for dx
void solve_linear_system(
    float   H[STATE_SIZE][STATE_SIZE],
    float  *b,
    float  *dx,
    int     state_size);

// 4. Update the state vector with dx
void update_state_vector(
    float  *state_vector,
    float  *dx,
    int     state_size);

// 5. Write the updated state vector back to the linked list
void write_state_to_graph(
    GraphNode *head,
    float  *state_vector,
    int     num_poses);

// 6. Main Gauss-Newton loop
void gauss_newton_slam(GraphNode *head, int max_iterations, float tol) {
    float state_vector[STATE_SIZE];
    float b[STATE_SIZE];
    float H[STATE_SIZE][STATE_SIZE];
    float dx[STATE_SIZE];
    int num_poses = 0;

    build_state_vector(head, state_vector, &num_poses);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Zero out b and H
        for (int i = 0; i < STATE_SIZE; ++i) {
            b[i] = 0.0f;
            for (int j = 0; j < STATE_SIZE; ++j)
                H[i][j] = 0.0f;
        }

        linearize_constraints(head, state_vector, b, H, num_poses);
        solve_linear_system(H, b, dx, num_poses * 3);
        update_state_vector(state_vector, dx, num_poses * 3);
        write_state_to_graph(head, state_vector, num_poses);

        // Check for convergence
        float norm = 0.0f;
        for (int i = 0; i < num_poses * 3; ++i)
            norm += dx[i] * dx[i];
        norm = sqrtf(norm);
        if (norm < tol)
            break;
    }
}

void example_graph_nodes(void) {

    LandmarkMeasurement *landmark1 = malloc(sizeof(LandmarkMeasurement));
    landmark1->id = 1;
    landmark1->x = 10.0f;
    landmark1->y = 15.0f;

    GraphNode *node1 = malloc(sizeof(GraphNode));
    node1->pose.x = 5.0f;
    node1->pose.y = 5.0f;
    node1->pose.theta = 0.0f;

    
    
    // Add an observation of the landmark
    node1->num_observations = 1;
    node1->observations = malloc(sizeof(Observation) * node1->num_observations);
    node1->observations[0].landmark = landmark1;
    node1->observations[0].range = 7.0f;
    node1->observations[0].bearing = 0.5f;

    GraphNode *node2    = malloc(sizeof(GraphNode));
    node2->pose.x       = 6.0f;
    node2->pose.y       = 6.0f;
    node2->pose.theta   = 0.1f;
    
    // Add an observation of the same landmark
    node2->num_observations = 1;
    node2->observations = malloc(sizeof(Observation) * node2->num_observations);
    node2->observations[0].landmark = landmark1;  // Same landmark as node1
    node2->observations[0].range    = 6.5f;
    node2->observations[0].bearing  = 0.4f;
    
    // Link the nodes
    node1->next = node2;
    node2->next = NULL;
}