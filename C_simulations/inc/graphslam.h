
#ifndef __GRAPHSLAM_H__
#define __GRAPHSLAM_H__

#include "graph_nodes.h"
#include <stddef.h>


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
    GraphNode  *head,
    float      *state_vector,
    int         num_poses);

// 6. Main Gauss-Newton loop
void gauss_newton_slam(GraphNode *head, int max_iterations, float tol);


void example_graph_nodes(void);

#endif // __GRAPHSLAM_H__