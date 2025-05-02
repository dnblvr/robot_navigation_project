

#ifndef __RRT_STAR_H__
#define __RRT_STAR_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/*
1. MAX_NEARBY: Limits the maximum number of nearby nodes considered during rewiring.
    RRT* improves paths by checking connections to nearby nodes. Checking all nearby nodes would be computationally expensive, so MAX_NEARBY ensures the algorithm remains efficient while still improving path quality.

2. MAX_ITER: Maximum number of iterations (samples) the algorithm will attempt.
    Acts as a computational budget. Larger values allow more thorough exploration but increase runtime. If no path is found by MAX_ITER, the algorithm terminates.

3. STEP_SIZE: Maximum distance the tree expands toward a new sample in one step.
    Controls how "aggressively" the tree grows. Smaller steps are safer in cluttered environments but may take longer to explore; larger steps speed up exploration but risk collisions.

4. RADIUS: Search radius for nearby nodes during rewiring.
    Determines how far the algorithm looks to optimize paths. Larger radii improve optimality but increase computation. In theory, RRT* uses a shrinking radius as n → ∞, but a fixed radius is often used in practice for simplicity.

5. GOAL_RADIUS: Proximity threshold to declare the goal reached.
    Avoids requiring exact goal reachability. A larger radius terminates the search earlier but may result in suboptimal final paths.

Key Tradeoffs:
    Larger STEP_SIZE/RADIUS:  Faster exploration, but higher collision risk.
    Smaller STEP_SIZE/RADIUS: Safer paths, but slower convergence.
    Higher MAX_ITER:          Better paths at the cost of computation time.

Relation to the Original Paper (Karaman & Frazzoli):

The paper suggests dynamically adjusting RADIUS as γ*(log(n)/n)^(1/d) (where d is the workspace dimension). Your implementation simplifies this with a fixed radius, which is common in practical implementations for computational efficiency. The other parameters (STEP_SIZE, MAX_ITER) are heuristic choices to balance performance and optimality.

*/

#define MAX_NEARBY  30
#define MAX_ITER    500
#define STEP_SIZE   3
#define RADIUS      8.0
#define GOAL_RADIUS 3.0


typedef struct Node {
  double  x,
          y;
  
  struct Node*  parent;
  double        cost;

} Node;


/**
 * @brief This structure describes a wall that starts in (x,y) and has dimensions width and height.
 * 
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


void add_node_to_tree(
      Tree* tree,
      Node* node);


double distance(
      Node* a,
      Node* b);


// Check if a point is inside an obstacle
int point_in_obstacle(
      double    x,
      double    y,
      Obstacle* obs);



int is_collision_free(
  Node*     from,
  Node*     to,
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


// Add this new function to save visualization data
/**
* 
*/
void save_tree_and_obstacles(
  Tree*     tree,
  Obstacle* obstacles,

  int     num_obstacles,
  double  goal_x,
  double  goal_y,
  double  goal_radius,

  const char* filename);


/**
 * @brief Main function to run the RRT* algorithm demo
 * 
 */
void rrt_star_demo(void);

#endif // __RRT_STAR_H__