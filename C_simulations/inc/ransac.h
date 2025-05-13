
#ifndef __RANSAC_H__
#define __RANSAC_H__

#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>


#define DEBUG_MODE
#define RANSAC_MAX_ITER  300
#define INLIER_THRESHOLD 0.5  // Adjust based on sensor noise
#define MAX_LINES        10   // Maximum number of lines to detect


// Parametric line model
/**
 * @struct  ParametricLine
 * @brief   this structure represents a parametric line in 2D space.
 * 
 */
typedef struct {

    /** @brief initial coordinates of the line */
    float x0, y0;  // point on the line

    /** @brief direction vector of the line */
    float dx, dy;  // direction vector

} ParametricLine;


// Function prototypes -----------------------------------------------------------------------------


/**
 * @brief   Determines the perpendicular distance from a point to a line. This is
 *          used in determining outliers from inliers in the RANSAC algorithm.
 * 
 * @param   x       
 * @param   y       
 * @param   line    
 * @return  float   
 */
float point_line_distance(
    float x, float y,
    
    ParametricLine* line);


/**
 * @brief 
 * 
 * @param points 
 * @param num_points 
 * @param lines 
 * @return uint8_t 
 */
uint8_t ransac_detect_lines(float points[][2], uint8_t *num_points, ParametricLine lines[MAX_LINES]);


/**
 * @brief 
 * 
 * @param line1 
 * @param line2 
 * @param x 
 * @param y 
 * @return int 
 */
int find_intersection(ParametricLine* line1,
                      ParametricLine* line2,
                      
                      float* x, float* y);


#endif /* __RANSAC_H__ */