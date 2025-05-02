
#ifndef __RANSAC_H__
#define __RANSAC_H__

#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
// #include "inc/matrices.h"

// #define DEBUG_MODE
#define RANSAC_MAX_ITER  100
#define INLIER_THRESHOLD 0.5  // Adjust based on sensor noise
#define MAX_LINES        10   // Maximum number of lines to detect

typedef struct {
    float m;  // Slope
    float c;  // Intercept
} LineModel;

float point_line_distance(float x, float y, LineModel* line);

uint8_t ransac_detect_lines(float points[][2], uint8_t *num_points, LineModel lines[MAX_LINES]);

void example_ransac(void);
void example_ransac(void);

#endif /* __RANSAC_H__ */