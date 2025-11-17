

#ifndef __INC_DATA_STRUCTURES_H__
#define __INC_DATA_STRUCTURES_H__

#include <stdbool.h>
#include <stdint.h>


#define OUTPUT_BUFFER 60

typedef struct {
    float x, y;
} Point2D;




/**
 * @brief Pose state representation (x, y, theta)
 */
typedef struct {
    float       x;
    float       y;
    float       theta;
    uint32_t    timestamp;
} Pose;


/**
 * @brief Point cloud data structure
 */
typedef struct {
    Point2D points[OUTPUT_BUFFER];
    int     num_points;
} PointCloud;


/**
 * @brief ICP alignment result
 */
typedef struct {
    float   dx;         // Translation in x
    float   dy;         // Translation in y
    float   dtheta;     // Rotation change
    float   confidence; // Match quality [0-1]
    bool    valid;      // Whether result is valid
} ICPResult;


#endif // __INC_DATA_STRUCTURES_H__
