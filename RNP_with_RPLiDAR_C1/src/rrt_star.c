
// #include "../inc/rrt_star.h"
#include "rrt_star.h"


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
		tree->nodes 	= (Node*)realloc(tree->nodes, tree->capacity * sizeof(Node));
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

    if (	point_in_obstacle(from->x, from->y, &obstacles[i])
        || 	point_in_obstacle(  to->x,   to->y, &obstacles[i]))
      
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

			if ( point_in_obstacle(x, y, &obstacles[i]) )
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

	double	dx    = x - from->x,
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

        if (    new_cost < node->cost
            &&  is_collision_free(new_node, node, obs, num_obs) )
        {
            node->parent  = new_node;
            node->cost    = new_cost;
        }
    }
}

/**
 * @brief Extracts the optimal path from the tree by tracing parent pointers from the goal node.
 */
uint8_t extract_path(
    Tree* 	tree,
    double  goal_x, double  goal_y,

    double  goal_radius,
    Node**  path,
    int*    path_length)
{

    // Find the node closest to the goal within goal_radius
    Node* goal_node = NULL;
    double min_dist = goal_radius;
    for (int i = 0; i < tree->num_nodes; i++) {
        double dist = sqrt(pow(tree->nodes[i].x - goal_x, 2) + pow(tree->nodes[i].y - goal_y, 2));
        if (dist <= min_dist) {
            min_dist = dist;
            goal_node = &tree->nodes[i];
        }
    }

    // If no node is within the goal radius, return failure
    if (!goal_node) {
        *path_length = 0;
        return 0; // No path found
    }

    // Trace back from goal_node to start using parent pointers
    int max_path = tree->num_nodes;
    Node** temp_path = (Node**)malloc(max_path * sizeof(Node*));
    int count = 0;
    Node* current = goal_node;
    while (current) {
        temp_path[count++] = current;
        current = current->parent;
    }

    // Reverse the path to go from start to goal
    for (int i = 0; i < count; i++) {
        path[i] = temp_path[count - i - 1];
    }
    *path_length = count;
    free(temp_path);
    return 1;
}

/**
 * @brief Runs RRT* and extracts the optimal path as a sequence of nodes.
 */
uint8_t find_optimal_path(
    double start_x, double start_y,
    double  goal_x, double  goal_y,

    Obstacle* obstacles, int num_obstacles,
	
    Node*** path, int* path_length,
    Tree** tree_out)
{
    // Initialize random seed
    srand(time(NULL));
    Tree* tree = (Tree*)malloc(sizeof(Tree));
    initialize_tree(tree, start_x, start_y);

    // flag to indicate if a path to the goal was found
    uint8_t found = 0;
    for (int iter = 0; iter < MAX_ITER; iter++) {

        double sample_x, sample_y;

        // Sample a point in the space, with a 10% chance of sampling the goal
        if ((double)rand()/RAND_MAX < 0.1) {
            sample_x = goal_x;
            sample_y = goal_y;
        
        // Otherwise, sample a random point in the space
        } else {
            sample_x = (double)rand()/RAND_MAX * 100;
            sample_y = (double)rand()/RAND_MAX * 100;
        }

        // find the nearest node in the tree to the sampled point
        Node* nearest = nearest_node(tree, sample_x, sample_y);
        Node new_node;
        steer(nearest, sample_x, sample_y, &new_node);


        // Check if the new node lands within any of the obstacles
        if ( is_collision_free(nearest, &new_node, obstacles, num_obstacles) ) {

            float   dx, dy,
                    distance_to_goal;

            Node*   nearby[MAX_NEARBY];
            Node*   best_parent;
            int     count;
            double  min_cost;


            find_nearby_nodes(tree, &new_node, RADIUS, nearby, &count);


            best_parent = nearest;
            min_cost    = nearest->cost + distance(nearest, &new_node);

            // Find the best parent node among nearby nodes
            for (int i = 0; i < count; i++) {
                double cost = nearby[i]->cost + distance(nearby[i], &new_node);

                if (    cost < min_cost
                    &&  is_collision_free(nearby[i], &new_node, obstacles, num_obstacles))
                {
                    min_cost    = cost;
                    best_parent = nearby[i];
                }
            }

            new_node.parent = best_parent;
            new_node.cost   = min_cost;

            add_node_to_tree(tree, &new_node);
            rewire(tree, &tree->nodes[tree->num_nodes-1], nearby, count, obstacles, num_obstacles);


            // Check if the new node is close enough to the goal
            dx  = new_node.x - goal_x;
            dy  = new_node.y - goal_y;
            distance_to_goal = sqrt(dx*dx + dy*dy);
            if (distance_to_goal < GOAL_RADIUS) {
                found = 1;
                break;
            }
        }
    }

    // if a path to the goal was found, extract it
    if (found) {
        *path       = (Node**)malloc(tree->num_nodes * sizeof(Node*));
        int success = extract_path(tree, goal_x, goal_y, GOAL_RADIUS, *path, path_length);
        *tree_out   = tree;
        return success;

    // if no path was found, clean up and return failure
    } else {
        *path_length = 0;
        *tree_out = tree;
        return 0;
    }
}


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

        const char* filename)
{
    // Open file for writing
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


void rrt_star_demo(void) { 

    // Example obstacle
    Obstacle  obstacles[]   = { {20, 5, 10, 10},
                                {5, 20, 40, 30}}; 
    int       num_obstacles = 2;

    double  start_x = 0.0,
            start_y = 0.0,
            goal_x  = 20.0,
            goal_y  = 50.0;

    Node** path = NULL;
    int path_length = 0;
    Tree* tree = NULL;

    uint8_t found = find_optimal_path(
        start_x, start_y,
        goal_x, goal_y,
        obstacles, num_obstacles,
        &path, &path_length,
        &tree
    );

    if (found && path_length > 0) {
        printf("Path found to goal!\n");
        printf("Path length: %d\n", path_length);
        for (int i = 0; i < path_length; i++) {
            printf("Node %d: (%.2f, %.2f)\n", i, path[i]->x, path[i]->y);
        }
    } else {
        printf("No path found to goal.\n");
    }

    // Visualize the tree and obstacles
    if (tree) {
        save_tree_and_obstacles(tree, obstacles, num_obstacles, goal_x, goal_y, GOAL_RADIUS, "rrt_tree.json");
        free(tree->nodes);
        free(tree);
    }
    if (path) free(path);
}