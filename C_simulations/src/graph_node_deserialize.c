
#include "graph_node_deserialize.h"

#include <string.h>
#include <stdlib.h>

// Helper: Parse Q15.16 hex string to float
static float q15_16_hex_to_float(const char *hex) {
    int32_t val = 0;

    for (int i = 0; i < 8; ++i) {
        char c = hex[i];
        val  <<= 4;

        if (c >= '0' && c <= '9') { 
            val |= (c - '0');

        } else if (c >= 'A' && c <= 'F') {
            val |= (c - 'A' + 10);

        } else if (c >= 'a' && c <= 'f') {
            val |= (c - 'a' + 10);

        }
    }
    return val / 65536.0f;
}

// Receive GraphNode list from file
GraphNode* receive_graphnodes_from_file(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) return NULL;

    char line[1024];
    GraphNode *head = NULL, *tail = NULL;

    while (fgets(line, sizeof(line), f)) {
        char *token = strtok(line, ";");
        while (token) {

            if (strncmp(token, "GN,", 3) == 0) {

                GraphNode *node = calloc(1, sizeof(GraphNode));
                char *p = token + 3;
                char *fields[7];

                for (int i = 0; i < 7; ++i) {
                    fields[i] = strsep(&p, ",");
                }

                node->pose.x        = q15_16_hex_to_float(fields[0]);
                node->pose.y        = q15_16_hex_to_float(fields[1]);
                node->pose.theta    = q15_16_hex_to_float(fields[2]);

                node->odometry.dx       = q15_16_hex_to_float(fields[3]);
                node->odometry.dy       = q15_16_hex_to_float(fields[4]);
                node->odometry.dtheta   = q15_16_hex_to_float(fields[5]);

                node->num_observations  = (uint8_t)strtol(fields[6], NULL, 16);
                node->observations      = calloc(node->num_observations, sizeof(Observation));

                if (!head) head = node;
                if (tail) tail->next = node;
                tail = node;
            
            } else if (     (strncmp(token, "OB,", 3) == 0)
                        &&  tail )
            {

                char *p = token + 3;
                char *fields[3];

                for (int i = 0; i < 3; ++i)
                    fields[i] = strsep(&p, ",");

                int idx = tail->num_observations++;

                tail->observations  = realloc(tail->observations, tail->num_observations * sizeof(Observation));
                Observation *obs    = &tail->observations[idx];

                obs->landmark_index = (int)strtol(fields[0], NULL, 16);
                obs->range          = q15_16_hex_to_float(fields[1]);
                obs->bearing        = q15_16_hex_to_float(fields[2]);
            }

            token = strtok(NULL, ";");
        }
    }
    fclose(f);
    return head;
}