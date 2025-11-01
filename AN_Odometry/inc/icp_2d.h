#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

typedef struct {
    float x, y;
} Point2D;

// Helper: Find the index of the closest point in target for a given source point
int find_closest_point(Point2D src, Point2D* target, int target_size);

// Helper: Compute centroid of a point set
void compute_centroid(Point2D* pts, int n, Point2D* centroid);

// Main ICP function
void icp_2d(
    Point2D* source, int source_size,
    Point2D* target, int target_size,
    int     max_iterations,
    float   tolerance,
    float  *out_R,   // 2x2 rotation matrix (row-major)
    float  *out_t);  // 2x1 translation vector
