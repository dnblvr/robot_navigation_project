
#include "coordinate_transform.h"


void make_transformation_matrix_3x3(float theta, float T[2], float matrix[3][3]) {
    // Initialize to identity
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            matrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Fill in rotation and translation
    matrix[0][0] =  cosf(theta);
    matrix[0][1] = -sinf(theta);
    matrix[0][2] = *(T);  // tx
    
    matrix[1][0] = sinf(theta);
    matrix[1][1] = cosf(theta);
    matrix[1][2] = *(T + 1);  // ty

}

// ---------------------------------------------------------------

// typedef struct {
//     float x;
//     float y;
// } Position;




void spherical_to_cartesian_average(float *pos, float *radii, float *phis, float theta) {

    uint8_t i, j;

    for (j = 0; j < 8; j++) {
        for (i = 0; i < 2; i++) {
            *(pos+i) = (    *(radii + i) * sinf(*(phis + i))) / 2.0f;
        
            *(pos+i) = (    *(radii + i) * sinf(*(phis + i)) * sinf(theta)) / 2.0f;
        }
    }
}



// #ifdef __CORDINATE_TRANSFORM_H__∂∂
void example_coord_transform(void) {
    uint8_t i;
    float pos[2][2]  = { {0.0f, 0.0f}, {0.0f, 0.0f} };
    float radii[2]   = {5.0f, 5.1f};
    float phis[2]    = {M_PI/4, M_PI/4 + 0.2};  // Example angles in radians
    float theta     = M_PI/6;  // Another angle

    spherical_to_cartesian_average(&(pos[0][0]), radii, phis, theta);

    for (i = 0; i < 2; i++) {
        printf("Position %d: x=%.2f, y=%.2f\n", i+1, pos[i][0], pos[i][1]);
    }
    printf("%.1f\n", theta);
    // return;

    

    float x_matrix[3][8] = {
        {1.0f,   2.0f,   3.0f,   4.0f,   5.0f,   6.0f,   7.0f,   8.0f},     // x-coordinates
        {5.577f, 5.962f, 6.346f, 6.731f, 7.115f, 7.500f, 7.885f, 8.269f},   // y-coordinates
        {1.f,    1.f,    1.f,    1.f,    1.f,    1.f,    1.f,    1.f}
        // Add more points as needed
    };

    print_matrix(3, 8, x_matrix);  // Print the input matrix



    float T_matrix[3][3]        = {{0,0,0},{0,0,0},{0,0,0}};  // Initialize a 3x3 matrix
    float T_inv_matrix[3][3]    = {{0,0,0},{0,0,0},{0,0,0}};  // Initialize a 3x3 matrix
    float output_matrix[3][8]   = {{0,0,0},{0,0,0},{0,0,0}};  // Initialize a 3x3 matrix for output
    float T_vec[2] = {1.0f, 2.0f};

    make_transformation_matrix_3x3(M_PI/6.f, T_vec, T_matrix);

    printf("Transformation Matrix:\n");
    print_matrix(3, 3, T_matrix);  // Print the transformation matrix

    matrix_inverse_3x3(T_matrix, T_inv_matrix);  // Invert the transformation matrix

    matrix_multiply(3, 3, T_inv_matrix, 3, 8, x_matrix, output_matrix);  // Apply inverse transformation

    printf("Transformed Matrix:\n");
    print_matrix(3, 8, output_matrix);
}