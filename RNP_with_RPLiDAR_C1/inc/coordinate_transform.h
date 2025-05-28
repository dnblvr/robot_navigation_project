
#ifndef __COORDINATE_TRANSFORM_H__
#define __COORDINATE_TRANSFORM_H__

#include "graph_nodes.h"
#include "matrices.h"

#include <stdint.h>
#include <math.h>



/**
 * @brief           Transform a 2D point using a transformation matrix
 * 
 * @param theta     Rotation angle in radians
 * @param T         Translation vector (2D)
 * @returns matrix  Transformation matrix (3x3)
 */
void make_transformation_matrix(float theta, float T[2], float matrix[3][3]);


/**
 * @brief           Transform a 2D point using a transformation matrix
 * 
 * @param theta     Rotation angle in radians
 * @param T         Translation vector (2D)
 * @returns matrix  Transformation matrix (3x3)
 */
 void make_transformation_matrix_pose(PoseState *pose, float matrix[3][3]);


/**
 * @brief       Convert spherical coordinates to Cartesian coordinates
 * 
 * @param pos   
 * @param radii 
 * @param phis  
 * @param theta 
 */
void spherical_to_cartesian_average(float *pos, float *radii, float *phis, float theta);




/**
 * @brief 
 * 
 */
void example_coord_transform(void);

/**
 * @brief           Print a 3xN matrix
 * 
 * @param three_by_n_matrix 
 * @param n 
 */
// void print_coords(float three_by_n_matrix[3][], uint8_t n);


/**
 * @brief 
 * 
 */
void example_coord_transform_2(void);

#endif // __COORDINATE_TRANSFORM_H__