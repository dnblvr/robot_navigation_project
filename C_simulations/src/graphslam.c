
#include "graphslam.h"



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

    GraphNode *node2 = malloc(sizeof(GraphNode));
    node2->pose.x = 6.0f;
    node2->pose.y = 6.0f;
    node2->pose.theta = 0.1f;
    
    // Add an observation of the same landmark
    node2->num_observations = 1;
    node2->observations = malloc(sizeof(Observation) * node2->num_observations);
    node2->observations[0].landmark = landmark1;  // Same landmark as node1
    node2->observations[0].range = 6.5f;
    node2->observations[0].bearing = 0.4f;
    
    // Link the nodes
    node1->next = node2;
    node2->next = NULL;
}