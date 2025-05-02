
#include "../inc/rrt_star.h"


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
      double  start_y)
{

  tree->num_nodes       = 1;
  tree->capacity        = 1;

  tree->nodes           = (Node*)malloc(sizeof(Node));
  tree->nodes[0].x      = start_x;
  tree->nodes[0].y      = start_y;
  tree->nodes[0].parent = NULL;
  tree->nodes[0].cost   = 0.0;

}


void add_node_to_tree(
      Tree* tree,
      Node* node)
{

  if (tree->num_nodes >= tree->capacity) {
    tree->capacity *= 2;
    tree->nodes = (Node*)realloc(tree->nodes, tree->capacity * sizeof(Node));
  }

  tree->nodes[tree->num_nodes] = *node;
  tree->num_nodes++;

}


double distance(
      Node* a,
      Node* b)
{
  double  dx = a->x - b->x,
          dy = a->y - b->y;
  return sqrt(dx*dx + dy*dy);
}


// Check if a point is inside an obstacle
int point_in_obstacle(
      double    x,
      double    y,
      Obstacle* obs)
{

  return ((x >= obs->x) && (x <= obs->x + obs->width ) &&
          (y >= obs->y) && (y <= obs->y + obs->height));

}



int is_collision_free(
      Node*     from,
      Node*     to,      
      Obstacle* obstacles,
      int       num_obstacles)
{

  for (int i = 0; i < num_obstacles; i++) {

    if (point_in_obstacle(from->x, from->y, &obstacles[i]) ||
        point_in_obstacle(to->x, to->y, &obstacles[i])        )
      
      return 0;
    }

    double  dx      = to->x - from->x,
            dy      = to->y - from->y,
            length  = sqrt(dx*dx + dy*dy);

    int     steps   = (int)(length / 1.0);

    for (int step = 1; step <= steps; step++) {

      double  t = (double)step/steps,
              x = from->x + dx*t,
              y = from->y + dy*t;

      for (int i = 0; i < num_obstacles; i++) {

        if (point_in_obstacle(x, y, &obstacles[i]))
          return 0;
    }
  }
  return 1;
}



Node* nearest_node(
      Tree*   tree,
      double  x,
      double  y)
{

  Node*   nearest   = NULL;
  double  min_dist  = INFINITY;

  // cycles through the 
  for (int i = 0; i < tree->num_nodes; i++) {

    Node* node = &tree->nodes[i];
    double dist = pow(node->x - x, 2) + pow(node->y - y, 2);

    if (dist < min_dist) {
      min_dist = dist;
      nearest = node;
    }
  }

  return nearest;
}



void steer(
      Node*   from,
      double  x, 
      double  y, 
      Node*   new_node)
{

  double  dx    = x - from->x,
          dy    = y - from->y,
          dist  = sqrt(dx*dx + dy*dy);


  if (dist <= STEP_SIZE) {
    new_node->x = x;
    new_node->y = y;

  } else {
    new_node->x = from->x + (dx/dist)*STEP_SIZE;
    new_node->y = from->y + (dy/dist)*STEP_SIZE;

  }
}



void find_nearby_nodes(
      Tree*   tree,
      Node*   node,
      double  radius,
      Node**  nearby,
      int*    count)
{

  *count = 0;

  for (int i = 0; i < tree->num_nodes; i++) {

    if (distance(node, &tree->nodes[i]) <= radius) {

      nearby[(*count)++] = &tree->nodes[i];
      if (*count >= MAX_NEARBY) break;
    }
  }
}


void rewire(
      Tree*     tree,
      Node*     new_node,
      Node**    nearby,
      int       count,
      Obstacle* obs,
      int       num_obs)
{

  for (int i = 0; i < count; i++) {

    // calculates the new cost of the 
    Node*   node      = nearby[i];
    double  new_cost  = new_node->cost + distance(new_node, node);

    if (new_cost < node->cost &&
        is_collision_free(new_node, node, obs, num_obs))
    {
      node->parent  = new_node;
      node->cost    = new_cost;
    }
  }
}


// Add this new function to save visualization data
/**
 * 
 */
void save_to_json(
      Tree*     tree,
      Obstacle* obstacles,

      int     num_obstacles,
      double  goal_x,
      double  goal_y,
      double  goal_radius,

      const char* filename)
{
    FILE* fp = fopen(filename, "w");
    if (!fp) {
      perror("Failed to create JSON file");
      return;
    }

    fprintf(fp, "{\n");
    
    // Save obstacles
    fprintf(fp, "\"obstacles\": [\n");
    for (int i = 0; i < num_obstacles; i++) {
      fprintf(fp, "  {\"x\": %.2f, \"y\": %.2f, \"width\": %.2f, \"height\": %.2f}%s\n",
              obstacles[i].x, obstacles[i].y,
              obstacles[i].width, obstacles[i].height,
              (i < num_obstacles-1) ? "," : "");
    }
    fprintf(fp, "],\n");

    // Save tree nodes
    fprintf(fp, "\"nodes\": [\n");
    for (int i = 0; i < tree->num_nodes; i++) {
      fprintf(fp, "  {\"x\": %.2f, \"y\": %.2f}%s\n",
              tree->nodes[i].x, tree->nodes[i].y,
              (i < tree->num_nodes-1) ? "," : "");
    }
    fprintf(fp, "],\n");

    // Save edges
    fprintf(fp, "\"edges\": [\n");
    int edge_count = 0;
    for (int i = 1; i < tree->num_nodes; i++) {  // Skip root node
      if (tree->nodes[i].parent) {
        fprintf(fp, "  {\"x1\": %.2f, \"y1\": %.2f, \"x2\": %.2f, \"y2\": %.2f}%s\n",
                tree->nodes[i].parent->x, tree->nodes[i].parent->y,
                tree->nodes[i].x, tree->nodes[i].y,
                (edge_count < tree->num_nodes-2) ? "," : "");
        edge_count++;
      }
    }
    fprintf(fp, "],\n");

    // Save goal
    fprintf(fp, "\"goal\": {\"x\": %.2f, \"y\": %.2f, \"radius\": %.2f}\n", 
            goal_x, goal_y, goal_radius);

    fprintf(fp, "}\n");
    fclose(fp);
}


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


void rrt_star_demo() { 
  
  srand(time(NULL));

  Tree tree;

  initialize_tree(&tree, 0.0, 0.0); // Start at (0,0)


  // Example obstacle
  Obstacle  obstacles[]   = { {20, 5, 10, 10},
                              {5, 20, 40, 30}}; 
  int       num_obstacles = 2;

  double  goal_x  = 20.0,
          goal_y  = 50.0;

  int iter;
  for (iter = 0; iter < MAX_ITER; iter++) {

  // Sample with 10% goal bias
  double  sample_x,
          sample_y;


  if ( (double)rand()/RAND_MAX < 0.1 ) {
      sample_x = goal_x;
      sample_y = goal_y;

  } else {
      sample_x = (double)rand()/RAND_MAX * 100;
      sample_y = (double)rand()/RAND_MAX * 100;

  }

  Node* nearest = nearest_node(&tree, sample_x, sample_y);
  Node  new_node;
  steer(nearest, sample_x, sample_y, &new_node);


  if ( is_collision_free(nearest, &new_node, obstacles, num_obstacles) ) {

      Node* nearby[MAX_NEARBY];
      int   count;
      find_nearby_nodes(&tree, &new_node, RADIUS, nearby, &count);


      // Choose best parent
      Node*   best_parent = nearest;
      double  min_cost    = nearest->cost + distance(nearest, &new_node);


      for (int i = 0; i < count; i++) {

          double cost = nearby[i]->cost + distance(nearby[i], &new_node);

          if (cost < min_cost && is_collision_free(nearby[i], &new_node, obstacles, num_obstacles)) {

              min_cost    = cost;
              best_parent = nearby[i];

          }
      }

      new_node.parent = best_parent;
      new_node.cost   = min_cost;

      add_node_to_tree(&tree, &new_node);
      rewire(&tree, &tree.nodes[tree.num_nodes-1], nearby, count, obstacles, num_obstacles);

      // Check goal proximity
      if ( sqrt(pow(new_node.x - goal_x, 2) + pow(new_node.y - goal_y, 2)) < GOAL_RADIUS ) {
          printf("Path found to goal!\n");
          break;
      }
  }
  }
  printf("number of iterations: %d", iter);

  save_tree_and_obstacles(&tree, obstacles, num_obstacles, goal_x, goal_y, GOAL_RADIUS, "rrt_tree.json");

  free(tree.nodes);
}