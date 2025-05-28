#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Structure for a 2D point
typedef struct {
    float x, y;
} Point2D;

// Structure for a 2D line segment (endpoints)
typedef struct {
    Point2D p1;
    Point2D p2;
    float length;
    float angle; // orientation in radians
} LineSegment2D;

// Compute length and angle for a line segment
void compute_line_properties(LineSegment2D* line) {
    float dx = line->p2.x - line->p1.x;
    float dy = line->p2.y - line->p1.y;
    line->length = sqrtf(dx*dx + dy*dy);
    line->angle = atan2f(dy, dx);
}

// Match line segments between two scans based on length and angle thresholds
int match_line_segments(
    LineSegment2D* scan1, int n1,
    LineSegment2D* scan2, int n2,
    int* matches, // Output: matches[i] = index in scan2 matched to scan1[i], or -1
    float length_thresh,
    float angle_thresh)
{
    // counter variables
    int     i, j;


    int num_matches = 0;
    for (i = 0; i < n1; i++) {
        int best_j = -1;
        float best_score = 1e6;
        for (j = 0; j < n2; j++) {
            float d_length = fabsf(scan1[i].length - scan2[j].length);
            float d_angle = fabsf(scan1[i].angle - scan2[j].angle);
            if (d_angle > M_PI) d_angle = 2*M_PI - d_angle;
            if (d_length < length_thresh && d_angle < angle_thresh) {
                float score = d_length + d_angle;
                if (score < best_score) {
                    best_score = score;
                    best_j = j;
                }
            }
        }
        matches[i] = best_j;
        if (best_j != -1) num_matches++;
    }
    return num_matches;
}

// Estimate transformation (dx, dy, dtheta) from matched line segments (simple version)
void estimate_transformation_from_lines(
    LineSegment2D* scan1, LineSegment2D* scan2,
    int* matches, int n1,
    float* dx, float* dy, float* dtheta)
{
    // counter variables
    int     i;

    // For simplicity, use centroids and average angle difference
    float cx1 = 0, cy1 = 0, cx2 = 0, cy2 = 0, sum_angle = 0;
    int count = 0;
    for (i = 0; i < n1; i++) {
        int j = matches[i];
        if (j != -1) {
            float mx1 = 0.5f * (scan1[i].p1.x + scan1[i].p2.x);
            float my1 = 0.5f * (scan1[i].p1.y + scan1[i].p2.y);
            float mx2 = 0.5f * (scan2[j].p1.x + scan2[j].p2.x);
            float my2 = 0.5f * (scan2[j].p1.y + scan2[j].p2.y);
            cx1 += mx1; cy1 += my1;
            cx2 += mx2; cy2 += my2;
            float d_ang = scan2[j].angle - scan1[i].angle;
            if (d_ang > M_PI) d_ang -= 2*M_PI;
            if (d_ang < -M_PI) d_ang += 2*M_PI;
            sum_angle += d_ang;
            count++;
        }
    }
    if (count > 0) {
        cx1 /= count; cy1 /= count;
        cx2 /= count; cy2 /= count;
        *dx = cx2 - cx1;
        *dy = cy2 - cy1;
        *dtheta = sum_angle / count;
    } else {
        *dx = 0; *dy = 0; *dtheta = 0;
    }
}

// Main function to perform line segment matching between two scans
void line_segment_matching(
    LineSegment2D* scan1, int n1,
    LineSegment2D* scan2, int n2,
    float length_thresh,
    float angle_thresh,
    float* dx, float* dy, float* dtheta)
{
    // counter variables
    int     i, j;

    // Compute properties for all lines
    for (i = 0; i < n1; i++) compute_line_properties(&scan1[i]);
    for (j = 0; j < n2; j++) compute_line_properties(&scan2[j]);

    // Match lines
    int* matches = (int*)malloc(n1 * sizeof(int));
    match_line_segments(scan1, n1, scan2, n2, matches, length_thresh, angle_thresh);

    // Estimate transformation
    estimate_transformation_from_lines(scan1, scan2, matches, n1, dx, dy, dtheta);

    free(matches);
}