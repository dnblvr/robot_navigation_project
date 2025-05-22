
#ifndef __GRAPHSLAM_H__
#define __GRAPHSLAM_H__

#include "graph_nodes.h"
#include <stddef.h>


void jacobian_error_wrt_xi(const float x_i[3], const float z_ij[2], float J[3][3]) {
    float r = z_ij[0];
    float phi = z_ij[1];
    float theta = x_i[2];
    float c = cosf(theta + phi);
    float s = sinf(theta + phi);

    // Partial derivatives of m_x and m_y w.r.t x_i, y_i, theta_i
    // m_x = x_i + r * cos(theta_i + phi)
    // m_y = y_i + r * sin(theta_i + phi)

    // d(m_x)/d(x_i) = 1
    // d(m_x)/d(y_i) = 0
    // d(m_x)/d(theta_i) = -r * sin(theta_i + phi)

    // d(m_y)/d(x_i) = 0
    // d(m_y)/d(y_i) = 1
    // d(m_y)/d(theta_i) = r * cos(theta_i + phi)

    // Fill in the Jacobian
    J[0][0] = -1;           // de1/dx_i
    J[0][1] = 0;            // de1/dy_i
    J[0][2] = r * s;        // de1/dtheta_i

    J[1][0] = 0;            // de2/dx_i
    J[1][1] = -1;           // de2/dy_i
    J[1][2] = -r * c;       // de2/dtheta_i

    J[2][0] = 0;            // de3/dx_i
    J[2][1] = 0;            // de3/dy_i
    J[2][2] = -1;           // de3/dtheta_i
}



void example_graph_nodes(void);

#endif // __GRAPHSLAM_H__