
#ifndef __COORDINATE_TRANSFORM_H__
#define __COORDINATE_TRANSFORM_H__

//#include "graph_nodes.h"
//#include "matrices.h"

#include "data_structures.h"

#include <stdint.h>
#include <math.h>


#define M_PI_F 3.1415927f

#define DEG_TO_RAD M_PI_F/180.f



/**
 * @brief           Transform a 2D point using a transformation matrix
 * 
 * @param theta     Rotation angle in radians
 * @param T         Translation vector (2D)
 * @returns matrix  Transformation matrix (3x3)
 */
//void make_transformation_matrix(
//        float    theta,
//        float        T[2],
//        float   matrix[3][3]);


/**
 * @brief           Create a transformation matrix from a pose
 * 
 * @param pose      Pointer to the pose state
 * @param matrix    Output transformation matrix (3x3)
 */
 void make_transformation_matrix_pose(
        Pose  *pose,
        float       matrix[3][3]);


/**
 * @brief       Convert spherical coordinates to Cartesian coordinates
 * 
 * @param pos   Output array of size 2 [x, y]
 * @param radii Radius in spherical coordinates
 * @param phis  Angle in spherical coordinates (in radians)
 * @param theta Angle in spherical coordinates (in radians)
 */
//void spherical_to_cartesian_average(
//        float  *pos,
//        float  *radii,
//        float  *phis,
//        float   theta);


/**
 * @brief       Convert polar coordinates to Cartesian coordinates
 * 
 * @param pos    Output array of size 2 [x, y]
 * @param radius Radius in polar coordinates
 * @param phis   Angle in polar coordinates (in radians)
 */
// inline void polar_to_cartesian(float *pos, float radius, float phis);


/**
 * @brief Convert polar coordinates to Cartesian coordinates
 * 
 * @param distance_angle Array of size 2 where:
 *                          distance_angle[0] = radius (distance)
 *                          distance_angle[1] = phis (angle in radians)
 * @param pos            Output array of size 3 [x, y, 1]
 */
inline void polar_to_cartesian(
        float   distance_angle[2],
        float   pos[3]);

/**
 * @brief 
 * 
 */
//void example_coord_transform(void);

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
//void example_coord_transform_2(void);

#endif // __COORDINATE_TRANSFORM_H__
