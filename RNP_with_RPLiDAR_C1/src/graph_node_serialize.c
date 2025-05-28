
#include "graph_node_serialize.h"

#include "graph_nodes.h"
#include <stdio.h>
#include <stdint.h>

// Helper: Convert float to Q15.16
static int32_t float_to_q15_16(float val) {
    return (int32_t)(val * 65536.0f);
}

// Helper: Write Q15.16 as 8 hex chars (MSB first)
static void write_q15_16_hex(FILE *f, float val) {
    int32_t q = float_to_q15_16(val);
    for (int i = 3; i >= 0; --i)
        fprintf(f, "%02X", (q >> (i*8)) & 0xFF);
}

// Send GraphNode list to file
void send_graphnodes_to_file(GraphNode *head, const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) return;

    for (GraphNode *node = head; node; node = node->next) {
        // GN header
        fprintf(f, "GN,");
        write_q15_16_hex(f, node->pose.x);     fprintf(f, ",");
        write_q15_16_hex(f, node->pose.y);     fprintf(f, ",");
        write_q15_16_hex(f, node->pose.theta); fprintf(f, ",");
        write_q15_16_hex(f, node->odometry.dx);fprintf(f, ",");
        write_q15_16_hex(f, node->odometry.dy);fprintf(f, ",");
        write_q15_16_hex(f, node->odometry.dtheta);fprintf(f, ",");
        fprintf(f, "%02X;", node->num_observations);

        // Observations
        for (int i = 0; i < node->num_observations; ++i) {
            Observation *obs = &node->observations[i];
            fprintf(f, "OB,");
            fprintf(f, "%02X,", obs->landmark_index);
            write_q15_16_hex(f, obs->range);   fprintf(f, ",");
            write_q15_16_hex(f, obs->bearing); fprintf(f, ";");
        }
    }
    // End of data
    fprintf(f, "ED\r\n");
    fclose(f);
}