
#ifndef __GRAPH_NODE_SERIALIZE_H__
#define __GRAPH_NODE_SERIALIZE_H__


#include <string.h>

#include "graph_nodes.h"


/**
 * @brief this function converts a float value to Q15.16 fixed-point representation.
 * 
 * @param val 
 * @return int32_t 
 */
static int32_t float_to_q15_16(float val);



/**
 * @brief this function Writes Q15.16 as 8 hex chars (MSB first)
 * 
 * @param f 
 * @param val 
 */
static void write_q15_16_hex(FILE *f, float val);



// 
/**
 * @brief this function sends GraphNode list to file
 * 
 * @param head 
 * @param filename 
 */
void send_graphnodes_to_file(GraphNode *head, const char *filename);


#endif // __GRAPH_NODE_SERIALIZE_H__
