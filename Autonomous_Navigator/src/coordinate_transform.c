
#include "./inc/coordinate_transform.h"


void make_transformation_matrix(float theta, float T[2], float matrix[3][3]) {
    // counter variables
    uint8_t i, j;

    /**
     * @brief in the format:
     *      [ cos(theta) -sin(theta) tx ]
     *      [ sin(theta)  cos(theta) ty ]
     *      [      0           0      1 ]
     */

    // Initialize to identity
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            matrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Fill in rotation and translation
    matrix[0][0] =  cosf(theta);
    matrix[0][1] = -sinf(theta);
    matrix[0][2] = *(T);        // tx
    
    matrix[1][0] =  sinf(theta);
    matrix[1][1] =  cosf(theta);
    matrix[1][2] = *(T + 1);    // ty

}

void make_transformation_matrix_pose(
    PoseState  *pose,
    float       matrix[3][3])
{
    /**
     * @brief in the format:
     *      [ cos(theta) -sin(theta) tx ]
     *      [ sin(theta)  cos(theta) ty ]
     *      [      0           0      1 ]
     */

    uint8_t i, j;

    // Initialize to identity
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            matrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Fill in rotation and translation
    matrix[0][0] =  cosf(pose->theta);
    matrix[0][1] = -sinf(pose->theta);
    matrix[0][2] =  pose->x;    // tx
    
    matrix[1][0] =  sinf(pose->theta);
    matrix[1][1] =  cosf(pose->theta);
    matrix[1][2] =  pose->y;    // ty

}



void spherical_to_cartesian_average(float *pos, float *radii, float *phis, float theta) {

    uint8_t i, j;

    for (j = 0; j < 8; j++) {
        for (i = 0; i < 2; i++) {
            *(pos+i) = (    *(radii + i) * sinf(*(phis + i))) / 2.0f;
        
            *(pos+i) = (    *(radii + i) * sinf(*(phis + i)) * sinf(theta)) / 2.0f;
        }
    }
}

inline void cylindrical_to_cartesian(float *pos, float radius, float phis) {

    // Assuming single point conversion
    pos[0] = radius * cosf(phis); // x = r * cos(phi)
    pos[1] = radius * sinf(phis); // y = r * sin(phi)
}




void example_coord_transform(void) {
    
    uint8_t i;

    float pos[2][2] = { {0.0f, 0.0f}, {0.0f, 0.0f} };
    float radii[2]  = {5.0f, 5.1f};
    float phis[2]   = {M_PI/4, M_PI/4 + 0.2};  // Example angles in radians
    float theta     =  M_PI/6;  // Another angle
  
    spherical_to_cartesian_average(&(pos[0][0]), radii, phis, theta);
  
    for (i = 0; i < 2; i++) {
        printf("Position %d: x=%.2f, y=%.2f\n", i+1, pos[i][0], pos[i][1]);
    }
    printf("%.1f\n", theta);
    // return;
  
    
  
    float x_matrix[3][8] = {
        {1.0f,   2.0f,   3.0f,   4.0f,   5.0f,   6.0f,   7.0f,   8.0f   },  // x-coordinates
        {5.577f, 5.962f, 6.346f, 6.731f, 7.115f, 7.500f, 7.885f, 8.269f },  // y-coordinates
        {1.f,    1.f,    1.f,    1.f,    1.f,    1.f,    1.f,    1.f    }
        // Add more points as needed
    };
  
    print_matrix(3, 8, x_matrix);  // Print the input matrix
  
    
  
    float T_matrix[3][3]        = {{0,0,0},{0,0,0},{0,0,0}};  // Initialize a 3x3 matrix
    float T_inv_matrix[3][3]    = {{0,0,0},{0,0,0},{0,0,0}};  // Initialize a 3x3 matrix
    float output_matrix[3][8]   = {};                         // Initialize a 3x3 matrix for output
    float T_vec[2]              = {1.0f, 2.0f};
  
    make_transformation_matrix(M_PI/6.f, T_vec, T_matrix);
  
    printf("Transformation Matrix:\n");
    print_matrix(3, 3, T_matrix);  // Print the transformation matrix
  
    matrix_inverse_3x3(T_matrix, T_inv_matrix);  // Invert the transformation matrix
  
    matrix_multiply(3, 3, T_inv_matrix, 
                    3, 8, x_matrix,
                    output_matrix);  // Apply inverse transformation
  
    printf("Transformed Matrix:\n");
    print_matrix(3, 8, output_matrix);
}
  

// void print_coords(float three_by_n_matrix[3][], uint8_t n) {

//     // Print the coordinates in the format "Position i: x=..., y=..."
//     uint8_t i;
//     for (i = 0; i < n; i++) {
//         printf("\t%.2f,\t%.2f\n", i+1, three_by_n_matrix[i][0], three_by_n_matrix[i][1]);
//     }
// }




void example_coord_transform_2(void) {

	int i;

	float T_vec[2] = {11.5f, 11.f};  // Translation vector
	float theta = 42 * M_PI / 180; // Rotation angle in radians
	float T_matrix[3][3] = {0};    // Transformation
//	float T_inv_matrix[3][3] = {0};    // Transformation
	

	float output_matrix[3][15] = {0};  // Output matrix for transformed points

	float sensor_points_matrix_r0[3][15] = {
		{14.34,  11.51,  10.95,  10.28,  9.26,   8.15,   6.65,   7.90,   10.59,  12.72,  14.96,  13.86,  12.67,  12.41,  11.00},
		{-3.50,  11.44,  9.92,   9.18,   7.00,   5.31,   2.90,   1.90,   0.56,   -0.96,  -1.76,  -5.08,  -6.70,  -8.48,  -11.25},
		{1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00,   1.00}
	};
  
    for (i = 0; i < 2; i++) {
        printf("Position %d: x=%.2f, y=%.2f\n", i+1, sensor_points_matrix_r0[i][0], sensor_points_matrix_r0[i][1]);
    }
    printf("%.1f\n", theta);
    // return;
  
    print_matrix(3, 15, sensor_points_matrix_r0);
    
    // float T_vec[2] = {11.5f, 11.f};  // Translation vector
	// float theta = 42 * M_PI / 180;   // Rotation angle in radians
    make_transformation_matrix(theta, T_vec, T_matrix);

  
    printf("Transformation Matrix:\n");
    print_matrix(3, 3, T_matrix);  // Print the transformation matrix
  
  
    matrix_multiply(3, 3,  T_matrix, 
                    3, 15, sensor_points_matrix_r0,
                    output_matrix);  // Apply inverse transformation
  
    printf("Transformed Matrix:\n");
    print_matrix(3, 15, output_matrix);


	float sensor_points_matrix_t0[3][15] = {
		{24.5,	12.4,	13,		13,		13.7,	14,		14.5,	16.1,	19,		21.6,	23.8,	25.2,	25.4,	26.4,	27.2},
		{18,	27.2,	25.7,	24.7,	22.4,	20.4,	17.6,	17.7,	18.5,	18.8,	19.7,	16.5,	14.5,	13,		10	},
		{1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	,	1	}
	};
    
    printf("Expected Output Matrix:\n");
    print_matrix(3, 15, sensor_points_matrix_t0);

}
