
#ifndef __COORDINATE_TRANSFORM_H__
#define __COORDINATE_TRANSFORM_H__

#include <stdint.h>
#include <math.h>

typedef struct {
    float x;
    float y;
} Position;

void spherical_to_cartesian_average(float *pos, float *radii, float *phis, float theta);

void example_coord_transform(void);

#endif // __COORDINATE_TRANSFORM_H__