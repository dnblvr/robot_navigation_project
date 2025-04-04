

#ifndef __INC_RRT_STAR_H__
#define __INC_RRT_STAR_H__

#define MAX_NEARBY  50
#define MAX_ITER    1000
#define STEP_SIZE   5.0
#define RADIUS      20.0
#define GOAL_RADIUS 5.0


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
  Tree* tree, double start_x, double start_y);


/**
* @brief Add a node to the tree
* 
* @param tree 
* @param node 
*/
void add_node_to_tree(
  Tree* tree,
  Node* node);

/**
* @brief Euclidean distance between two nodes
* 
* @param a 
* @param b 
* @return double 
*/
double distance(Node* a, Node* b);

/**
* @brief Collision check between two nodes
* 
* @param from 
* @param to 
* @param obstacles 
* @param num_obstacles 
* @return int 
*/
int is_collision_free(
  Node*     from,
  Node*     to,
  Obstacle* obstacles,
  int       num_obstacles);

/**
* @brief Find nearest node in the tree
* 
* @param tree 
* @param x 
* @param y 
* @return Node* 
*/
Node* nearest_node(
  Tree*   tree,
  double  x,
  double  y);

/**
* @brief Steer from nearest node towards the sample point
* 
* @param from 
* @param x 
* @param y 
* @param new_node 
*/
void steer(
  Node*   from,
  double  x, 
  double  y, 
  Node*   new_node);

/**
* @brief   Find nearby nodes within a radius
* 
* @param tree 
* @param node 
* @param radius 
* @param nearby 
* @param count 
*/
void find_nearby_nodes(
  Tree*   tree,
  Node*   node,
  double  radius,
  Node**  nearby,
  int*    count);


/**
* @brief   Rewire the tree to optimize paths
* 
* @param tree 
* @param new_node 
* @param nearby 
* @param count 
* @param obs 
* @param num_obs 
*/
void rewire(
  Tree*     tree,
  Node*     new_node,
  Node**    nearby,
  int       count,
  Obstacle* obs,
  int       num_obs);


void save_tree_and_obstacles(
  Tree*     tree,
  Obstacle* obstacles,

  int     num_obstacles,
  double  goal_x,
  double  goal_y,
  double  goal_radius,

  const char* filename);


// void save_to_json(Tree*, Obstacle*, int, double, double, double, const char*);
void save_to_json(
  Tree* tree,
  Obstacle* obstacles,
  int num_obstacles,
  double goal_x,
  double goal_y,
  double goal_radius,
  const char* filename);

#endif