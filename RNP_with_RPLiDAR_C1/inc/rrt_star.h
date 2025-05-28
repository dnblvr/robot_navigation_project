

#ifndef __RRT_STAR_H__
#define __RRT_STAR_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/**
 * @param   MAX_NEARBY: Limits the maximum number of nearby nodes considered during rewiring.
 * @details RRT* improves paths by checking connections to nearby nodes. Checking all nearby
 *          nodes would be computationally expensive, so MAX_NEARBY ensures the algorithm
 *          remains efficient while still improving path quality.
 *
 * @param   MAX_ITER: Maximum number of iterations (samples) the algorithm will attempt.
 * @details Acts as a computational budget. Larger values allow more thorough exploration but
 *          increase runtime. If no path is found by MAX_ITER, the algorithm terminates.
 *
 * @param   STEP_SIZE: Maximum distance the tree expands toward a new sample in one step.
 * @details Controls how "aggressively" the tree grows. Smaller steps are safer in cluttered
 *          environments but may take longer to explore; larger steps speed up exploration but risk collisions.
 *
 * @param   RADIUS: Search radius for nearby nodes during rewiring.
 * @details Determines how far the algorithm looks to optimize paths. Larger radii improve optimality but increase computation. In theory, RRT* uses a shrinking radius as n → ∞, but a fixed radius is often used in practice for simplicity.
 *
 * @param   GOAL_RADIUS: Proximity threshold to declare the goal reached.
 * @details Avoids requiring exact goal reachability. A larger radius terminates the search earlier but may result in suboptimal final paths.
 *
 * Key Tradeoffs:
 *    Larger STEP_SIZE/RADIUS:  Faster exploration, but higher collision risk.
 *    Smaller STEP_SIZE/RADIUS: Safer paths, but slower convergence.
 *    Higher MAX_ITER:          Better paths at the cost of computation time.
 *
 * Relation to the Original Paper (Karaman & Frazzoli):
 * 
 * The paper suggests dynamically adjusting RADIUS as γ*(log(n)/n)^(1/d) (where d is the workspace dimension). Your implementation simplifies this with a fixed radius, which is common in practical implementations for computational efficiency. The other parameters (STEP_SIZE, MAX_ITER) are heuristic choices to balance performance and optimality.
 *
 */

#define MAX_NEARBY  30
#define MAX_ITER    500
#define STEP_SIZE   3
#define RADIUS      8.0
#define GOAL_RADIUS 3.0


typedef struct Node {
    double  x,
            y;

    struct Node*    parent;
    double          cost;

} Node;


/**
 * @brief This structure describes a wall that starts in (x,y) and has dimensions width and height.
 */
typedef struct Obstacle {
  double  x,
          y,
          width,
          height;
} Obstacle;


typedef struct Tree {
  Node* nodes;
  int   num_nodes;
  int   capacity;
} Tree;



/**
 * @brief Initialize tree with start node
 * 
 * @param tree 
 * @param start_x 
 * @param start_y 
 */
 void initialize_tree(
    Tree*   tree,
    double  start_x,
    double  start_y);

/**
 * @brief this function adds a node to the tree
 * 
 * @param tree tree to which the node will be added
 * @param node node to be added
 */
void add_node_to_tree(
    Tree* tree,
    Node* node);

/**
 * @brief calculates the distance between two nodes
 * 
 * @param a 
 * @param b 
 * @return double euclidean distance between the two nodes
 */
double distance(
    Node* a,
    Node* b);


// 
/**
 * @brief Check if a point is inside an obstacle
 * 
 * @param x     
 * @param y     
 * @param obs   
 * @return int  
 */
int point_in_obstacle(
    double    x,
    double    y,
    Obstacle* obs);


int is_collision_free(
    Node*   from,
    Node*   to,

    Obstacle* obstacles,
    int       num_obstacles);



Node* nearest_node(
    Tree*   tree,
    double  x,
    double  y);



void steer(
    Node*   from,
    double  x, 
    double  y, 
    Node*   new_node);



void find_nearby_nodes(
    Tree*   tree,
    Node*   node,
    double  radius,
    Node**  nearby,
    int*    count);


void rewire(
    Tree*     tree,
    Node*     new_node,
    Node**    nearby,
    int       count,
    Obstacle* obs,
    int       num_obs);




/**
 * @brief Extracts the optimal path from the tree by tracing parent pointers from the goal node
 * 
 * @param tree                      The generated RRT* tree
 * @param goal_x, @param goal_y     coordinates of the goal
 * 
 * @param goal_radius  Acceptable radius to consider a node as the goal
 * @param path         Output array of Node* (should be allocated by caller, or you can malloc inside)
 * @param path_length  Output: number of nodes in the path
 * @return uint8_t     1 if path found, 0 otherwise
 */
uint8_t extract_path(
    Tree* tree,
    double goal_x,
    double goal_y,
    double goal_radius,
    Node** path,
    int* path_length);


/**
 * @brief Runs RRT* and extracts the optimal path as a sequence of nodes
 * 
 * @param start_x, @param start_y   Start position
 * @param  goal_x, @param  goal_y   Goal position
 * @param obstacles          Array of obstacles
 * @param num_obstacles      Number of obstacles
 * @param path               Output: array of Node* (allocated inside)
 * @param path_length        Output: number of nodes in the path
 * @return uint8_t           1 if path found, 0 otherwise
 */
uint8_t find_optimal_path(
        double start_x, double start_y,
        double goal_x,  double  goal_y,

        Obstacle* obstacles, int num_obstacles,

        Node*** path, int* path_length,
        Tree** tree_out);

// --------------------------------------------------------------------------

/**
* @todo: make a struct which has the following properties:
* 
// Example obstacle
Obstacle  obstacles[]   = { {20, 5, 10, 10},
                            {5, 20, 40, 30}}; 
int       num_obstacles = 2;

double  goal_x  = 20.0,
        goal_y  = 50.0;

        

* by implementing this, it should be easy to iterate through
* all the simulations in a sequential order
*/


// typedef struct {
// } Simulation_Profile;

/**
 * @brief this function saves the RRT* tree and obstacles to a JSON file.
 * 
 * @param tree          The RRT* tree containing nodes and edges
 * @param obstacles     Array of obstacles (rectangular Obstacle structs)
 * @param num_obstacles Number of obstacles in the array

 * @param goal_x, @param goal_y goal position

 * @param goal_radius   acceptable radius to consider a node as the goal
 * @param filename      Name of the output JSON file
 */
void save_tree_and_obstacles(
        Tree*     tree,
        Obstacle* obstacles,

        int     num_obstacles,
        double  goal_x,
        double  goal_y,
        double  goal_radius,

        const char* filename);

void rrt_star_demo(void);

#endif // __RRT_STAR_H__