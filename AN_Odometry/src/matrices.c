

#include "./inc/matrices.h"


// --- Matrix Operations ---


void print_matrix(
        uint8_t rows, uint8_t cols,
        float   matrix[rows][cols])
{
    // counter variables
    uint8_t i, j;

    printf("Matrix %dx%d:\n", rows, cols);
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            // 8-width, 1 decimal place
            printf("%1.2f%s\t", matrix[i][j], (j < cols - 1) ? ", " : "");
        }
        printf("\n");
    }
    printf("\n");
}

// Matrix inverse (3x3 only)
uint8_t matrix_mult_3x3(
        float        a[3][3],
        float        b[3][3],

        float   result[3][3])
{
    uint8_t i;
    float sum = 0.0f;

    for (i = 0; i < 3; i++) {

        sum =  a[i][0] * b[0][0]
             + a[i][1] * b[1][0]
             + a[i][2] * b[2][0];
        result[i][0] = sum;

        sum =  a[i][0] * b[0][1]
             + a[i][1] * b[1][1]
             + a[i][2] * b[2][1];
        result[i][1] = sum;

        sum =  a[i][0] * b[0][2]
             + a[i][1] * b[1][2]
             + a[i][2] * b[2][2];
        result[i][2] = sum;

    }
    return 1;
}

// Matrix inverse (3x3 only)
uint8_t matrix_inverse_3x3(
        float     m[3][3],
        float   inv[3][3])
{

    // counter variables
    uint8_t i, j;

    // Calculate determinant
    float det =   m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    // Check for singularity
    if (fabsf(det) < 1e-6) return 0;

    // Compute adjugate (transposed cofactors)
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            float c =   m[(i + 1) % 3][(j + 1) % 3] * m[(i + 2) % 3][(j + 2) % 3]
                      - m[(i + 1) % 3][(j + 2) % 3] * m[(i + 2) % 3][(j + 1) % 3];
            inv[j][i] = c / det;  // Transpose during assignment
        }
    }
    return 1;
}


// Matrix transpose (result must be pre-allocated)
uint8_t matrix_transpose(
        uint8_t rows, uint8_t cols,

        float        source[rows][cols],
        float   destination[cols][rows])
{

    // counter variables
    uint8_t i, j;

    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            destination[j][i] = source[i][j];
        }
    }
    return 1;
}

// Matrix multiplication (result must be pre-allocated)
uint8_t matrix_multiply(
    uint8_t a_rows, uint8_t a_cols, float a[a_rows][a_cols],
    uint8_t b_rows, uint8_t b_cols, float b[b_rows][b_cols],
    float result[a_rows][b_cols])
{

    // counter variables
    uint8_t i, j, k;

    if (a_cols != b_rows) {
        return 0;
    }

    for (i = 0; i < a_rows; i++) {
        for (j = 0; j < b_cols; j++) {
            float sum = 0.0f;
            for (k = 0; k < a_cols; k++) {
                sum += a[i][k] * b[k][j];
            }
            result[i][j] = sum;
        }
    }
    return 1;
}

