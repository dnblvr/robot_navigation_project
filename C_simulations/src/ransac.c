
#include "ransac.h"


// Function to calculate the distance of a point from a line
float point_line_distance(float x, float y, ParametricLine* line) {
    // Compute the distance of a point (x, y) from a parametric line
    // Line equation: x(t) = x0 + t * dx, y(t) = y0 + t * dy
    // Distance formula: |(dy * (x - x0) - dx * (y - y0))| / sqrt(dx^2 + dy^2)

    return  fabs(line->dy*(x - line->x0)  -  line->dx*(y - line->y0)) /

            sqrt(line->dx*line->dx + line->dy*line->dy);
}


uint8_t ransac_detect_lines(float points[][2], uint8_t *num_points, ParametricLine lines[MAX_LINES]) {
    uint8_t num_lines   = 0,
            max_inliers = 0;

    while (*num_points > 0 && num_lines < MAX_LINES) {
        ParametricLine best_model = {0};
        max_inliers = 0;

        // Dynamically allocate memory for inliers
        uint8_t *inliers = (uint8_t *)malloc(*num_points * sizeof(uint8_t));
        if (!inliers) {
            perror("Failed to allocate memory for inliers");
            return num_lines;
        }

        // Run RANSAC to find the best line
        for (int iter = 0; iter < RANSAC_MAX_ITER; iter++) {
            // Randomly select two points
            uint8_t idx1 = rand() % *num_points;
            uint8_t idx2 = rand() % *num_points;

            if (idx1 == idx2) continue;  // Ensure two distinct points

            // Compute parametric line model
            float x1 = points[idx1][0], y1 = points[idx1][1];
            float x2 = points[idx2][0], y2 = points[idx2][1];

            ParametricLine model;
            model.x0 = x1;
            model.y0 = y1;
            model.dx = x2 - x1;
            model.dy = y2 - y1;

            // Count inliers
            uint8_t current_inliers = 0;
            for (uint8_t i = 0; i < *num_points; i++) {
                float x = points[i][0], y = points[i][1];
                if (point_line_distance(x, y, &model) < INLIER_THRESHOLD) {
                    inliers[current_inliers++] = i;
                }
            }

            // Update best model if more inliers are found
            if (current_inliers > max_inliers) {
                max_inliers = current_inliers;
                best_model = model;
            }
        }

        // If no inliers were found, stop
        if (max_inliers == 0) {
            free(inliers);
            break;
        }

        // Save the detected line
        lines[num_lines++] = best_model;

        // Remove inliers from the dataset
        float new_points[*num_points - max_inliers][2];
        uint8_t new_num_points = 0;
        for (uint8_t i = 0; i < *num_points; i++) {
            uint8_t is_inlier = 0;
            for (uint8_t j = 0; j < max_inliers; j++) {
                if (i == inliers[j]) {
                    is_inlier = 1;
                    break;
                }
            }
            if (!is_inlier) {
                new_points[new_num_points][0] = points[i][0];
                new_points[new_num_points][1] = points[i][1];
                new_num_points++;
            }
        }

        // Update the dataset
        for (uint8_t i = 0; i < new_num_points; i++) {
            points[i][0] = new_points[i][0];
            points[i][1] = new_points[i][1];
        }
        *num_points = new_num_points;

        // Free dynamically allocated memory
        free(inliers);
    }

    return num_lines;
}



int find_intersection(
    ParametricLine* line1,
    ParametricLine* line2,

    float* x, float* y) {
    
    
    // Solve for intersection of two parametric lines
    // Line 1: x1(t1) = x0_1 + t1 * dx1, y1(t1) = y0_1 + t1 * dy1
    // Line 2: x2(t2) = x0_2 + t2 * dx2, y2(t2) = y0_2 + t2 * dy2

    float det = line1->dx * line2->dy - line1->dy * line2->dx;

    // Check if lines are parallel (det == 0)
    if (fabs(det) < 1e-6) {
        return 0;  // No intersection (parallel or coincident lines)
    }

    // Solve for t1 and t2
    float t1 = ((line2->x0 - line1->x0) * line2->dy - (line2->y0 - line1->y0) * line2->dx) / det;

    // Compute intersection point
    *x = line1->x0 + t1 * line1->dx;
    *y = line1->y0 + t1 * line1->dy;

    return 1;  // Intersection found
}