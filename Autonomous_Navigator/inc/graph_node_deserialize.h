
/**
 * 
 * 
 */

#ifndef __GRAPH_NODE_DESERIALIZE_H__
#define __GRAPH_NODE_DESERIALIZE_H__

#include <string.h>
#include <stdlib.h>

#include "graph_nodes.h"

/**
 * @brief This is a helper function that parses Q15.16 hex string to float.
 * 
 * @param hex 
 * @return float 
 */
static float q15_16_hex_to_float(const char *hex);



/**
 * @brief This function receives GraphNode list from file.
 * 
 * @param filename 
 * @return GraphNode* 
 */
GraphNode* receive_graphnodes_from_file(const char *filename);


#endif // __GRAPH_NODE_DESERIALIZE_H__
